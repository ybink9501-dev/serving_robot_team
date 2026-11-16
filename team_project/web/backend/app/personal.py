"""Opt-in web gateway. The personal Fleet is the ONLY execution dispatcher."""
import asyncio
import importlib.util
import io
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
import uuid
from contextlib import asynccontextmanager, suppress
from urllib.request import Request as HttpRequest, urlopen

from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import JSONResponse, Response
from PIL import Image
from starlette.websockets import WebSocketDisconnect
import yaml


MODES = ('KITCHEN_PICKUP', 'DELIVER_DIRECT', 'TABLE_COLLECT')
TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')
SIGNALS = {'LOAD': ('WAIT_LOAD', 'kitchen', ('KITCHEN_PICKUP',)),
           'COMPLETE': ('WAIT_HANDOVER', 'table', ('KITCHEN_PICKUP', 'DELIVER_DIRECT')),
           'COLLECTED': ('WAIT_COLLECTION', 'table', ('TABLE_COLLECT',))}


def load_shared(root, name):
    path = Path(root) / 'ros_ws/src/cafe_autonomy/cafe_autonomy' / (name + '.py')
    spec = importlib.util.spec_from_file_location('_personal_' + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Problem(Exception):
    def __init__(self, code, status=409):
        self.code, self.status = code, status


class FleetHttp:
    def __init__(self, url):
        from urllib.parse import urlparse
        parsed = urlparse(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost'):
            raise ValueError('Personal Fleet must use local HTTP')
        self.url = url.rstrip('/')

    def call(self, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = HttpRequest(self.url + path, data=data, headers={'Content-Type': 'application/json'})
        with urlopen(req, timeout=4) as response:
            return json.load(response)

    def status(self):
        return self.call('/api/status')

    def send(self, body):
        return self.call('/api/request', body)


class Gateway:
    def __init__(self, settings, transport=None):
        self.settings = settings
        self.profile = load_shared(settings.repo, 'profile_contract').load_profile(
            settings.repo, settings.personal_config)
        self.refuse_home = load_shared(settings.repo, 'home_policy').home_refusal
        self.destinations = yaml.safe_load(Path(self.profile['waypoints_file']).read_text())['destinations']
        # Match cafe_autonomy.waypoints.table_destinations: plain poses are valid too.
        self.tables = [k for k in self.destinations if k != 'kitchen' and not k.endswith('_HOME')]
        self.transport = transport or FleetHttp(settings.personal_url)
        self.lock = threading.RLock()
        db = settings.personal_db or str(Path(settings.repo) / '_local_only/web_personal/backend.sqlite3')
        if db == ':memory:' and transport is None:
            raise ValueError('Personal operation requires a persistent database')
        if db != ':memory:':
            Path(db).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(db, check_same_thread=False)
        self.db.executescript('''
          CREATE TABLE IF NOT EXISTS orders (id TEXT PRIMARY KEY, client TEXT UNIQUE,
            fingerprint TEXT NOT NULL, data TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS outbox (seq INTEGER PRIMARY KEY AUTOINCREMENT,
            key TEXT UNIQUE, body TEXT NOT NULL, state TEXT NOT NULL, reply TEXT);
          CREATE TABLE IF NOT EXISTS identity (revision TEXT PRIMARY KEY);
        ''')
        identity = self.db.execute('SELECT revision FROM identity').fetchone()
        if identity and identity[0] != self.profile['profile_revision']:
            self.db.close()
            raise ValueError('Gateway database belongs to another profile; use a separate database')
        self.db.execute('INSERT OR IGNORE INTO identity VALUES (?)', (self.profile['profile_revision'],))
        self.db.commit()
        self.remote = {}; self.token = None; self.fresh_at = 0.; self.error = 'CONNECTING'
        self.closed = False

    def close(self):
        with self.lock:
            self.closed = True
            self.db.close()

    def rows(self):
        return [json.loads(row[0]) for row in self.db.execute('SELECT data FROM orders ORDER BY rowid')]

    def save(self, task):
        self.db.execute('UPDATE orders SET data=? WHERE id=?', (json.dumps(task), task['task_id']))

    def ready(self):
        return self.error is None and time.monotonic() - self.fresh_at < 2.

    def require_ready(self):
        if not self.ready():
            raise Problem(self.error or 'FLEET_STALE', 503)

    def enqueue(self, key, body):
        blob = json.dumps(body, sort_keys=True)
        old = self.db.execute('SELECT body,state,reply FROM outbox WHERE key=?', (key,)).fetchone()
        if old:
            if old[0] != blob:
                raise Problem('REQUEST_ID_CONFLICT')
            return {'submission': old[1], 'reply': json.loads(old[2]) if old[2] else None}
        self.db.execute('INSERT INTO outbox(key,body,state) VALUES (?,?,?)', (key, blob, 'PENDING'))
        return {'submission': 'PENDING', 'reply': None}

    def create(self, body):
        with self.lock, self.db:
            if not isinstance(body, dict) or body.get('type') != 'SERVE':
                raise Problem('BAD_BODY', 422)
            if any(k in body for k in ('robot_id', 'x', 'y', 'pose', 'goal', 'position')):
                raise Problem('FORBIDDEN_FIELD', 422)
            client, mode, table = body.get('client_request_id'), body.get('mode'), body.get('destination_id')
            if not isinstance(client, str) or not 1 <= len(client) <= 128:
                raise Problem('BAD_REQUEST_ID', 422)
            if mode not in MODES or table not in self.tables:
                raise Problem('INVALID_MISSION', 422)
            fp = json.dumps([self.profile['profile_revision'], mode, table])
            old = self.db.execute('SELECT fingerprint,data FROM orders WHERE client=?', (client,)).fetchone()
            if old:
                if old[0] != fp:
                    raise Problem('REQUEST_ID_CONFLICT')
                return json.loads(old[1]), False
            self.require_ready()
            tid = 'task-' + uuid.uuid4().hex[:24]
            rid = 'backend-' + self.profile['site_id'] + '-' + uuid.uuid4().hex
            now = time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime()) + '.000Z'
            task = dict(task_id=tid, client_request_id=client, request_id=rid, type='SERVE',
                        mode=mode, destination_id=table, robot_id=None, status='QUEUED',
                        submission='PENDING', created_at=now, updated_at=now,
                        profile_revision=self.profile['profile_revision'], mission_phase=None)
            self.db.execute('INSERT INTO orders VALUES (?,?,?,?)', (tid, client, fp, json.dumps(task)))
            self.enqueue('submit:' + rid, dict(action='SUBMIT', request_id=rid, mode=mode, table=table, side='auto'))
            return task, True

    def get_task(self, tid):
        row = self.db.execute('SELECT data FROM orders WHERE id=?', (tid,)).fetchone()
        if not row:
            raise Problem('TASK_NOT_FOUND', 404)
        return json.loads(row[0])

    def signal(self, tid, body):
        if not isinstance(body, dict) or body.get('action') not in SIGNALS:
            raise Problem('BAD_SIGNAL', 422)
        with self.lock, self.db:
            task = self.get_task(tid); action = body['action']
            wire = dict(action=action, source=body.get('source'), request_id=task['request_id'])
            key = action + ':' + task['request_id']
            if self.db.execute('SELECT 1 FROM outbox WHERE key=?', (key,)).fetchone():
                return self.enqueue(key, wire)
            self.require_ready()
            state, source, modes = SIGNALS[action]
            if wire['source'] != source or task['mode'] not in modes:
                raise Problem('WRONG_SIGNAL_ORIGIN_OR_MODE', 422)
            st = self.remote.get('supervisors', {}).get(task['robot_id'], {})
            meta = self.remote.get('pose_metadata', {}).get(task['robot_id'], {})
            if meta.get('pose_age_s', 999) >= 1. or meta.get('status_age_s', 999) >= 2.:
                raise Problem('ROBOT_STATE_STALE', 503)
            if (st.get('request_id') != task['request_id'] or st.get('state') != state
                    or not st.get('confirmation_pose_valid')):
                raise Problem('NOT_IN_CONFIRMATION_STATE')
            return self.enqueue(key, wire)

    def cancel(self, tid):
        with self.lock, self.db:
            task = self.get_task(tid)
            if task['status'] in TERMINAL:
                raise Problem('TASK_FINISHED')
            self.require_ready()
            self.enqueue('cancel:' + task['request_id'], dict(action='CANCEL', request_id=task['request_id']))
            task['cancel_requested'] = True
            self.save(task)
            return task

    def home(self, robot, client):
        if not isinstance(client, str) or not 1 <= len(client) <= 128:
            raise Problem('BAD_REQUEST_ID', 422)
        if robot not in self.profile['robots']:
            raise Problem('ROBOT_NOT_FOUND', 404)
        with self.lock, self.db:
            key = 'home:' + robot + ':' + client
            row = self.db.execute('SELECT state,reply FROM outbox WHERE key=?', (key,)).fetchone()
            if row:
                return {'submission': row[0], 'reply': json.loads(row[1]) if row[1] else None}
            self.require_ready()
            # Covers another new-ID manual HOME before its acceptance appears in a snapshot.
            for blob, state in self.db.execute('SELECT body,state FROM outbox'):
                sent = json.loads(blob)
                if sent.get('mode') == 'GO_HOME' and sent.get('robot_id') == robot and state in ('PENDING', 'UNKNOWN'):
                    raise Problem('ALREADY_RETURNING')
            pose = self.remote.get('robot_poses', {}).get(robot)
            meta = self.remote.get('pose_metadata', {}).get(robot, {})
            home = self.destinations[robot + '_HOME']
            if meta.get('pose_age_s', 999) >= 1. or meta.get('status_age_s', 999) >= 2.:
                raise Problem('NOT_LOCALIZED')
            reason = self.refuse_home(robot, self.remote.get('requests', {}),
                                      self.remote.get('supervisors', {}), pose,
                                      (home['x'], home['y']), .1, self.remote.get('queued', []))
            if reason:
                raise Problem(reason)
            return self.enqueue(key, dict(action='SUBMIT', request_id='manual-home-' + uuid.uuid4().hex,
                                           robot_id=robot, mode='GO_HOME'))

    def pump(self):
        # One worker and one lock preserve durable FIFO and prevent concurrent senders.
        with self.lock:
            if self.closed:
                return
            try:
                snapshot = self.transport.status()
                if not isinstance(snapshot, dict):
                    raise ValueError('Invalid snapshot')
                if (snapshot.get('protocol_version') != 1 or snapshot.get('site_id') != self.profile['site_id']
                        or snapshot.get('profile_revision') != self.profile['profile_revision']):
                    self.error = 'PROFILE_MISMATCH'
                    return
                token = (snapshot.get('session_id'), snapshot.get('status_seq'))
                if not token[0] or not isinstance(token[1], int):
                    self.error = 'INVALID_FLEET_STATUS'
                    return
                if token != self.token:
                    self.token, self.fresh_at = token, time.monotonic()
                self.remote = snapshot; self.error = None
            except (OSError, ValueError, TypeError) as exc:
                self.error = 'FLEET_UNAVAILABLE'
                return
            if not self.ready():
                return
            with self.db:
                pending = self.db.execute("SELECT key,body FROM outbox WHERE state IN ('PENDING','UNKNOWN') ORDER BY seq LIMIT 1").fetchone()
                if pending:
                    key, blob = pending; wire = json.loads(blob)
                    try:
                        reply = self.transport.send(wire)
                        if not isinstance(reply, dict):
                            raise ValueError('Invalid acknowledgement')
                        if reply.get('action') != wire['action'] or reply.get('request_id') != wire['request_id']:
                            raise ValueError('Invalid Fleet acknowledgement')
                        state = 'ACCEPTED' if reply.get('ok') is True else 'REJECTED'
                        self.db.execute('UPDATE outbox SET state=?,reply=? WHERE key=?', (state, json.dumps(reply), key))
                    except (OSError, ValueError, TypeError):
                        self.db.execute("UPDATE outbox SET state='UNKNOWN' WHERE key=?", (key,))
                for task in self.rows():
                    ack = self.db.execute('SELECT state,reply FROM outbox WHERE key=?', ('submit:' + task['request_id'],)).fetchone()
                    task['submission'] = ack[0]
                    if ack[0] == 'REJECTED':
                        task['detail'] = json.loads(ack[1]).get('error', 'REJECTED')
                        task['status'] = 'FAILED'
                    live = snapshot.get('requests', {}).get(task['request_id'])
                    if live:
                        task['robot_id'] = live['robot']
                        phase = snapshot.get('supervisors', {}).get(live['robot'], {})
                        task['mission_phase'] = phase.get('state') if phase.get('request_id') == task['request_id'] else None
                        # Never infer completion from IDLE, a HOME id, or an HTTP acknowledgement.
                        if live['state'] in ('QUEUED', 'ASSIGNED', 'RUNNING', *TERMINAL):
                            task['status'] = live['state']
                        task['received_seq'] = live.get('received_seq')
                    task['updated_at'] = time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime()) + '.000Z'
                    self.save(task)

    def robots(self):
        result = []
        for robot in self.profile['robots']:
            st = self.remote.get('supervisors', {}).get(robot, {})
            pose = self.remote.get('pose_metadata', {}).get(robot, {})
            if not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k]) for k in ('x', 'y', 'yaw')):
                continue
            phase = st.get('state', 'UNKNOWN')
            known = ('IDLE', 'COMPLETED', 'CANCELED', 'GOING_PICKUP', 'GOING_TABLE', 'GOING_DIRECT',
                     'GOING_KITCHEN', 'WAIT_LOAD', 'WAIT_HANDOVER', 'WAIT_COLLECTION', 'RETURNING_HOME',
                     'RECOVERY_WAIT', 'YIELDING', 'YIELD_WAIT', 'ERROR')
            online = self.ready() and pose.get('pose_age_s', 999) < 1. and pose.get('status_age_s', 999) < 2.
            state = ('ERROR' if not online or phase not in known or phase == 'ERROR' else
                     'RETURNING' if phase == 'RETURNING_HOME' else
                     'SERVING' if phase in ('WAIT_HANDOVER', 'WAIT_COLLECTION') else
                     'PAUSED' if phase in ('YIELDING', 'YIELD_WAIT', 'RECOVERY_WAIT') else
                     'IDLE' if phase in ('IDLE', 'COMPLETED', 'CANCELED') else 'NAVIGATING')
            task = next((t for t in self.rows() if t['request_id'] == st.get('request_id')), None)
            result.append(dict(type='robot_state', robot_id=robot, state=state, mission_state=phase,
                               mission_phase=phase, request_id=st.get('request_id'),
                               task_id=task['task_id'] if task else None,
                               destination_id=task['destination_id'] if task else None,
                               online=online, battery_percent=100, telemetry='FRESH' if online else 'STALE',
                               pose={k: pose[k] for k in ('x', 'y', 'yaw')}))
        return result

    def view(self):
        with self.lock:
            tasks = self.rows()
            for task in tasks:
                task['telemetry'] = 'FRESH' if self.ready() else 'STALE'
                task['signals'] = [dict(action=json.loads(blob)['action'], submission=state,
                                        reply=json.loads(reply) if reply else None)
                                   for blob,state,reply in self.db.execute('SELECT body,state,reply FROM outbox')
                                   if json.loads(blob).get('request_id') == task['request_id']
                                   and json.loads(blob)['action'] != 'SUBMIT']
            controls = [dict(key=key, submission=state, reply=json.loads(reply) if reply else None)
                        for key,state,reply in self.db.execute("SELECT key,state,reply FROM outbox WHERE key LIKE 'home:%'")]
            return dict(mode='personal', execution_owner='personal_fleet',
                        site_id=self.profile['site_id'], profile_revision=self.profile['profile_revision'],
                        ready=self.ready(), error=self.error if self.error else (None if self.ready() else 'FLEET_STALE'),
                        tasks=tasks, controls=controls, robots=self.robots(), fleet=self.remote)


def create_personal_app(settings, transport=None):
    gateway = Gateway(settings, transport)

    @asynccontextmanager
    async def lifespan(app):
        async def poll():
            while True:
                await asyncio.to_thread(gateway.pump)
                await asyncio.sleep(.25)
        worker = asyncio.create_task(poll())
        try:
            yield
        finally:
            worker.cancel()
            with suppress(asyncio.CancelledError):
                await worker
            await asyncio.to_thread(gateway.close)

    app = FastAPI(lifespan=lifespan, title='serving-robot personal gateway')
    app.state.gateway = gateway
    from .personal_site_setup import PersonalSiteSetup,mount_site_setup,runtime_mode,seed_semantics
    app.state.site_setup = PersonalSiteSetup(gateway,settings)
    mount_site_setup(app,app.state.site_setup)

    @app.exception_handler(Problem)
    async def problem(request, exc):
        return JSONResponse({'code': exc.code, 'message': exc.code}, status_code=exc.status)

    @app.get('/api/capabilities')
    def capabilities():
        return {'mode': 'personal', 'execution_owner': 'personal_fleet', 'mission_modes': MODES, 'robot_modes': ['mapping','operation'], 'map_setup': True}

    @app.get('/api/personal/status')
    def status():
        return gateway.view()

    @app.get('/api/tasks')
    def tasks():
        return gateway.view()['tasks']

    @app.post('/api/tasks')
    def create(body: dict):
        if runtime_mode(settings.repo)=='mapping' or app.state.site_setup.mapping.active:raise Problem('ROBOT_IN_MAPPING_MODE')
        task, new = gateway.create(body)
        return JSONResponse(task, status_code=202 if new else 200)

    @app.post('/api/tasks/{tid}/signals')
    def signal(tid: str, body: dict):
        if runtime_mode(settings.repo)=='mapping' or app.state.site_setup.mapping.active:raise Problem('ROBOT_IN_MAPPING_MODE')
        return gateway.signal(tid, body)

    @app.post('/api/tasks/{tid}/cancel')
    def cancel(tid: str):
        return gateway.cancel(tid)

    @app.get('/api/robots')
    def robots():
        return gateway.view()['robots']

    @app.post('/api/robots/{robot}/home')
    def home(robot: str, body: dict):
        if runtime_mode(settings.repo)=='mapping' or app.state.site_setup.mapping.active:raise Problem('ROBOT_IN_MAPPING_MODE')
        return gateway.home(robot, body.get('client_request_id'))

    @app.websocket('/ws/state')
    async def websocket(ws: WebSocket):
        await ws.accept()
        try:
            while True:
                state = await asyncio.to_thread(gateway.view)
                for robot in state['robots']:
                    await ws.send_json(robot)
                for task in state['tasks']:
                    await ws.send_json({**task, 'type': 'task_state'})
                await asyncio.sleep(.5)
        except (RuntimeError, OSError, WebSocketDisconnect):
            pass

    @app.get('/api/mapping/status')
    def mapping_status():
        return app.state.site_setup.mapping_status()

    @app.get('/api/stores/{store}/map')
    def map_info(store: str):
        # Existing frontend store id is a display alias; active site identity is explicit.
        with Image.open(gateway.profile['image_path']) as image:
            width, height = image.size
        meta = yaml.safe_load(Path(gateway.profile['map_yaml']).read_text())
        return dict(store_id=store, site_id=gateway.profile['site_id'], map_revision=1,
                    image_url=f'/api/stores/{store}/map.png', width=width, height=height,
                    resolution=meta['resolution'], origin=dict(zip(('x', 'y', 'yaw'), meta['origin'])), frame_id='map')

    @app.get('/api/stores/{store}/map.png')
    def map_png(store: str):
        with Image.open(gateway.profile['image_path']) as image:
            buf = io.BytesIO(); image.convert('L').save(buf, format='PNG')
        return Response(buf.getvalue(), media_type='image/png')

    @app.get('/api/stores/{store}/semantics')
    def semantics(store: str):
        return app.state.site_setup.load(store) or seed_semantics(gateway,store)

    @app.api_route('/api/{path:path}', methods=['POST', 'PUT', 'DELETE', 'PATCH'])
    def unsupported(path: str):
        raise Problem('UNSUPPORTED_IN_PERSONAL_MODE')

    return app
