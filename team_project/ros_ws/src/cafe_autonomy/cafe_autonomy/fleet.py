"""Two robot dispatcher: idempotent requests, state confirmation, traffic control, local web UI."""
import json
import time
import math
import sqlite3
import threading
import queue
import hashlib
import uuid
import zlib
from .waypoints import read_waypoints, table_destinations, available_approach
from collections import deque
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from .map_corridors import discover_corridors, same_corridor
from .corridor_priority import CorridorArrivalPriority, forward_entry_observed, priority_yield_options, reservation_intents
from .runtime_config import load_limits
from .home_policy import home_refusal
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
import yaml
from .policy import Task, FleetCore, ROBOT_IDS, distance, may_confirm, ZoneArbiter, in_bounds, should_return_home
from .dynamic_policy import mutual_peer_block
from .yield_policy import candidate_poses, assess_path, choose_yield, yield_goal_clear, passed_blocking_position, opposed_corridor_paths, reserved_winner_clear, in_rectangle, reservation_setup_pending, preentry_pause_acknowledged


def jparse(text):
    try: return json.loads(text)
    except (ValueError,TypeError): return {}

class Fleet(Node):
    def __init__(self):
        super().__init__('cafe_fleet')
        self.declare_parameter('database_path',str(Path.home()/'cafe_autonomy_fleet.sqlite3'))
        self.declare_parameter('http_port',8765)
        self.site_id=self.declare_parameter('site_id','').value
        self.profile_revision=self.declare_parameter('profile_revision','').value
        self.session_id=uuid.uuid4().hex;self.status_seq=0
        self.declare_parameter('runtime_limits_file','')
        self.declare_parameter('zones_file','')
        self.declare_parameter('waypoints_file','')
        self.auto_home_when_idle=self.declare_parameter('auto_home_when_idle',True).value
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        zone_file=self.get_parameter('zones_file').value
        zone_map=yaml.safe_load(Path(zone_file).read_text(encoding='utf8'))['zones'] if zone_file else {}
        if not isinstance(zone_map,dict):raise ValueError('Fleet zones must be a mapping')
        for name,b in zone_map.items():
            if not isinstance(b,list) or len(b)!=4 or not (b[0]<b[1] and b[2]<b[3]):
                raise ValueError('Invalid zone rectangle '+name)
        self.core=FleetCore();self.zones=ZoneArbiter(zones=zone_map); self.status={r:{'state':'UNKNOWN'} for r in ROBOT_IDS}
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.status_wall={}
        self.poses={};self.pose_wall={};self.odom_wall={};self.scan_wall={}
        self.safety={};self.last_error=None;self.dispatch_started={};self.replies={};self.terminal_seen=set()
        self.traffic={r:True for r in ROBOT_IDS}; self.person_zones={}
        self.recovery=None;self.zone_config=yaml.safe_load(Path(zone_file).read_text())
        self.blocked_since={};self.block_alerted=set();self.yield_attempted=set()
        self.route_paths={};self.route_wall={};self.route_context={};self.preentry_retry_after=0.
        self.arrival_priority=CorridorArrivalPriority();self.arrival_ready=False;self.map_signature=None;self.map_change_hold=False
        preentry=self.zone_config.get('preentry',{})
        self.discover_corridors=preentry.get('enabled',True) and preentry.get('discover_from_map',True)
        self.auto_regions=set();self.discovered_route_contexts=set();self.auto_region_serial=0
        self.preentry_zones=preentry.get('zones',list(zone_map)) if preentry.get('enabled',True) else []
        if not isinstance(self.preentry_zones,list) or any(z not in zone_map for z in self.preentry_zones):raise ValueError('Invalid preentry corridor names')
        waypoint_file=self.get_parameter('waypoints_file').value
        destinations=read_waypoints(waypoint_file)
        self.destinations=destinations
        self.table_names=table_destinations(destinations)
        self.home_poses=[(destinations[r+'_HOME']['x'],destinations[r+'_HOME']['y']) for r in ROBOT_IDS]
        self.pose_yaw={};self.pose_history={r:deque(maxlen=120) for r in ROBOT_IDS}
        self.yield_probe=None;self.yield_generation=0;self.last_yielder=None
        self.yield_grid=None
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/robot_01/map',self.on_yield_map,map_qos)
        self.yield_planners={r:ActionClient(self,ComputePathToPose,f'/{r}/compute_path_to_pose') for r in ROBOT_IDS}
        self.commands={r:self.create_publisher(String,f'/{r}/serving/request',10) for r in ROBOT_IDS}
        self.complete={r:self.create_publisher(String,f'/{r}/serving/complete',10) for r in ROBOT_IDS}
        self.holds={r:self.create_publisher(Bool,f'/{r}/traffic_hold',10) for r in ROBOT_IDS}
        self.pub=self.create_publisher(String,'/fleet/status',10)
        self.reply_pub=self.create_publisher(String,'/fleet/reply',10)
        self.create_subscription(String,'/fleet/request',self.on_request,10)
        for r in ROBOT_IDS:
            self.create_subscription(NavPath,f'/{r}/plan',lambda m,bot=r:self.on_route(bot,m),10)
            self.create_subscription(String,f'/{r}/serving/status',lambda m,bot=r:self.on_status(bot,m),10)
            self.create_subscription(String,f'/{r}/navigation_event',lambda m,bot=r:self.on_navigation(bot,m),10)
            self.create_subscription(String,f'/{r}/dynamic_safety',lambda m,bot=r:self.dynamic.update({bot:jparse(m.data)}),10)
            self.create_subscription(String,f'/{r}/dynamic_event',lambda m,bot=r:self.on_dynamic_event(bot,m),20)
            self.create_subscription(Odometry,f'/{r}/odom',lambda m,bot=r:self.on_odom(bot,m),10)
            self.create_subscription(PoseWithCovarianceStamped,f'/{r}/amcl_pose',lambda m,bot=r:self.on_amcl(bot,m),10)
            # The safety node publishes regular heartbeats, distinct from geometric scan distances.
            self.create_subscription(String,f'/{r}/safety_status',lambda m,bot=r:self.on_safety(bot,m),10)
        dbpath=self.get_parameter('database_path').value
        Path(dbpath).parent.mkdir(parents=True,exist_ok=True)
        self.db=sqlite3.connect(dbpath,check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, stamp REAL, event TEXT, payload TEXT)')
        self.db.execute('CREATE TABLE IF NOT EXISTS requests (request_id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL, task TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS signals (request_id TEXT, action TEXT, fingerprint TEXT, reply TEXT, PRIMARY KEY(request_id,action))')
        if 'applied' not in [r[1] for r in self.db.execute('PRAGMA table_info(signals)')]:self.db.execute('ALTER TABLE signals ADD COLUMN applied INTEGER NOT NULL DEFAULT 0')
        self.db.commit();self.database_lock=threading.Lock();self.signal_sent_wall={}
        for receipt,rid,fingerprint,payload in self.db.execute('SELECT rowid,request_id,fingerprint,task FROM requests ORDER BY rowid'):
            task=Task(**json.loads(payload))
            if task.received_seq is None:
                task.received_seq=receipt;self.save_task(task)
            if task.state in ('ASSIGNED','RUNNING'):task.state='RECOVERY_REQUIRED'
            self.core.tasks[rid]=task
            if task.state=='QUEUED':self.core.queue.append(rid)
        self.dynamic={};self.dynamic_events={};self.navigation_events={}
        self.http_queue=queue.Queue();self.latest_status={};self.snapshot_lock=threading.Lock()
        self.http_port=int(self.get_parameter('http_port').value)
        self.start_http()
        self.create_timer(0.25,self.tick)
        self.get_logger().info('Fleet started; requests require a matching supervisor IDLE heartbeat')

    def on_route(self,robot,msg):
        if msg.header.frame_id!='map':return
        self.route_paths[robot]=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.route_wall[robot]=time.monotonic()
        state=self.status.get(robot,{})
        self.route_context[robot]=(state.get('request_id'),tuple(state.get('target_pose') or ()))
        key=(self.map_signature,self.route_context[robot])
        if self.discover_corridors and self.yield_grid is not None and state.get('request_id') and key not in self.discovered_route_contexts:
            regions=discover_corridors(self.yield_grid,self.route_paths[robot],
                2*self.limits['dynamic_obstacles']['robot_radius_m'],2*self.limits['fleet']['robot_separation_m'])
            for bounds in regions:
                if any(same_corridor(bounds,b) for b in self.zones.zones.values()):continue
                self.auto_region_serial+=1;name='map_corridor_'+str(self.auto_region_serial)
                self.zones.zones[name]=bounds;self.preentry_zones.append(name);self.auto_regions.add(name)
                self.record('MAP_CORRIDOR_DISCOVERED',{'zone':name,'bounds':bounds,'map_key':str(self.map_signature)})
            if regions:self.discovered_route_contexts.add(key)

    def on_yield_map(self,msg):
        if msg.header.frame_id!='map':return
        p=msg.info.origin.position;q=msg.info.origin.orientation
        signature=(msg.info.width,msg.info.height,msg.info.resolution,p.x,p.y,q.x,q.y,q.z,q.w,zlib.crc32(bytes((v+1) for v in msg.data)))
        if signature!=self.map_signature:
            if self.map_signature is not None:
                self.map_change_hold=True
                self.last_error='Map changed: restart with the matching site profile and initial localization'
                if self.yield_probe:
                    for handle in self.yield_probe.get('handles',[]):handle.cancel_goal_async()
                self.yield_probe=None;self.recovery=None
            for name in self.auto_regions:
                self.zones.zones.pop(name,None)
                if name in self.preentry_zones:self.preentry_zones.remove(name)
            self.auto_regions.clear();self.discovered_route_contexts.clear()
            self.map_signature=signature;self.arrival_priority.reset(str(signature));self.arrival_ready=False
            self.route_paths.clear();self.route_wall.clear();self.route_context.clear()
        self.yield_grid=msg

    def corridor_priority_for(self,names,poses):
        orders={r:self.core.tasks.get(self.status[r].get('request_id')) for r in ROBOT_IDS}
        sequences={r:t.received_seq if t else None for r,t in orders.items()}
        for name in names:
            priority=self.arrival_priority.choose(name,ROBOT_IDS,sequences,poses,self.zones.zones[name])
            if priority:return priority
        return None

    def record(self,event,payload):
        with self.database_lock:
            self.db.execute('INSERT INTO events(stamp,event,payload) VALUES (?,?,?)',(time.time(),event,json.dumps(payload,ensure_ascii=False)))
            self.db.commit()

    def publish_msg(self,publisher,data):
        m=String();m.data=json.dumps(data,ensure_ascii=False);publisher.publish(m)

    def save_task(self,task):
        with self.database_lock:
            self.db.execute('UPDATE requests SET task=? WHERE request_id=?',(json.dumps(vars(task)),task.request_id));self.db.commit()
    def on_dynamic_event(self,robot,msg):
        evt=jparse(msg.data);self.dynamic_events[robot]=evt;self.record('DYNAMIC_EVENT',evt)
    def on_status(self,robot,msg):
        obj=jparse(msg.data)
        if obj.get('robot_id')!=robot:return
        self.status[robot]=obj;self.status_wall[robot]=time.monotonic()
        rid=obj.get('request_id');task=self.core.tasks.get(rid)
        if task and task.state=='RECOVERY_REQUIRED' and task.robot==robot and obj.get('state') not in ('IDLE','UNKNOWN'):
            self.core.active[robot]=rid
        before=task.state if task else None
        self.core.update(robot,obj.get('state',''),rid)
        if rid:
            with self.database_lock:
                for action in obj.get('accepted_signals',[]):self.db.execute('UPDATE signals SET applied=1 WHERE request_id=? AND action=? AND applied=0',(rid,action))
                self.db.commit()
        if task and task.state!=before:self.save_task(task)
        rid=obj.get('request_id')
        if obj.get('state') in ('COMPLETED','CANCELED','ERROR') and rid and (rid,obj['state']) not in self.terminal_seen:
            self.terminal_seen.add((rid,obj['state']))
            self.record('MISSION_TERMINAL',{'robot':robot,'request_id':rid,'state':obj['state'], 'reason':obj.get('reason')})


    def on_navigation(self,robot,msg):
        evt=jparse(msg.data);self.navigation_events[robot]=evt;self.record('NAVIGATION_EVENT',evt)
        rid=evt.get('request_id')
        if evt.get('event')=='MISSION_REJECTED' and rid and self.core.active.get(robot)==rid:
            self.core.tasks[rid].state='FAILED';self.save_task(self.core.tasks[rid])
            self.core.active.pop(robot,None)
            self.last_error='MISSION_REJECTED: '+str(evt.get('reason'))
            self.record('MISSION_REJECTED',evt)

    def on_odom(self,robot,msg):
        self.odom_wall[robot]=time.monotonic()

    def on_amcl(self,robot,msg):
        # AMCL is in map frame; robot-specific odom frames cannot be compared directly.
        if msg.header.frame_id!='map':return
        p=msg.pose.pose.position
        self.poses[robot]=(p.x,p.y)
        self.pose_wall[robot]=time.monotonic()

    def on_safety(self,robot,msg):
        obj=jparse(msg.data)
        self.safety[robot]=obj
        self.scan_wall[robot]=time.monotonic() if obj.get('sensor_fresh') and obj.get('detector_fresh') and obj.get('dynamic_fresh') else 0.0

    def enqueue_request(self,data,response=None):
        # HTTP and ROS share one FIFO receipt order. Rejected/duplicate requests
        # do not acquire a new accepted Task receipt sequence.
        self.http_queue.put((data,response,time.time()))

    def on_request(self,msg):
        try:self.enqueue_request(jparse(msg.data))
        except Exception as exc:self.get_logger().error('Unexpected request error: '+str(exc))

    def process(self,data,received_at=None):
        rid=data.get('request_id')
        action=data.get('action','SUBMIT')
        try:
            if action=='SUBMIT':
                robot=data.get('robot_id')
                if not robot:
                    if rid in self.core.tasks:robot=self.core.tasks[rid].robot
                    else:
                        eligible=[r for r in ROBOT_IDS if self.status[r].get('state') not in ('ERROR','UNKNOWN')]
                        if not eligible:raise ValueError('No available robot')
                        robot=min(eligible,key=lambda r:sum(t.robot==r and t.state in ('QUEUED','RUNNING','ASSIGNED') for t in self.core.tasks.values()))
                mode=data.get('mode','KITCHEN_PICKUP');table=data.get('table') if mode not in ('GO_HOME','GO_KITCHEN') else None
                if mode not in ('GO_HOME','GO_KITCHEN') and table not in self.table_names:
                    raise ValueError('Unknown table')
                side=data.get('side','auto')
                if table is not None and side!='auto' and side not in self.destinations[table].get('approaches',{}):
                    raise ValueError('Unknown destination approach '+str(side))
                fingerprint=json.dumps({'robot_id':data.get('robot_id') or None,'table':table,'mode':mode,'side':data.get('side','auto')},sort_keys=True)
                with self.database_lock:
                    old=self.db.execute('SELECT fingerprint FROM requests WHERE request_id=?',(rid,)).fetchone()
                    if old and old[0]!=fingerprint:raise ValueError('request_id content conflict')
                    if not old and mode=='GO_HOME':
                        if time.monotonic()-self.pose_wall.get(robot,0.)>=1. or time.monotonic()-self.status_wall.get(robot,0.)>=2.:
                            raise ValueError('NOT_LOCALIZED')
                        reason=home_refusal(robot,{k:vars(t) for k,t in self.core.tasks.items()},self.status,self.poses.get(robot),self.home_poses[ROBOT_IDS.index(robot)],self.limits['arrival']['xy_tolerance_m'],self.core.queue)
                        if reason:raise ValueError(reason)
                    task,new=self.core.submit(rid,table,robot,mode,data.get('side','auto'))
                    if new:
                        if received_at is not None:task.received_at=received_at
                        self.db.execute('INSERT INTO requests VALUES (?,?,?)',(rid,fingerprint,json.dumps(vars(task))));self.db.commit()
                if new:self.record('TASK_QUEUED',vars(task))
                reply={'ok':True,'action':action,'request_id':rid,'state':task.state, 'new':new,'robot_id':task.robot}
            elif action=='RECOVER':
                task=self.core.tasks.get(rid)
                if not task:raise ValueError('Unknown request_id')
                robot=task.robot;state=self.status[robot]
                if task.state=='RECOVERY_REQUIRED' and state.get('state')=='IDLE' and state.get('request_id') is None:
                    if not state.get('goal_execution_verified_idle'):raise ValueError('Goal idle not verified')
                    task.state='QUEUED';self.core.queue.append(rid);self.save_task(task)
                else:
                    if state.get('request_id')!=rid or state.get('state') not in ('ERROR','RECOVERY_WAIT'):raise ValueError('Recovery does not match current failed mission')
                    task.state='RECOVERY_REQUIRED';self.core.active[robot]=rid;self.save_task(task)
                    self.publish_msg(self.commands[robot],{'command':'RECOVER','request_id':rid,'robot_id':robot})
                reply={'ok':True,'action':action,'request_id':rid,'robot_id':robot}
            elif action=='RESET':
                robot=data.get('robot_id')
                if robot not in ROBOT_IDS:raise ValueError('Unknown robot')
                if self.status[robot].get('state') not in ('ERROR','CANCELED','COMPLETED') or robot in self.core.active:
                    raise ValueError('Robot has not reached a terminal state')
                self.publish_msg(self.commands[robot],{'command':'RESET','robot_id':robot})
                self.record('ROBOT_RESET_REQUEST',{'robot':robot})
                reply={'ok':True,'action':'RESET','robot_id':robot,'request_id':rid}
            elif action in ('LOAD','COMPLETE','COLLECTED','CANCEL'):
                task=self.core.tasks.get(rid)
                if task is None:raise ValueError('Unknown request_id')
                robot=task.robot;supervisor=self.status[robot];state=supervisor.get('state')
                if 'table' in data and data['table']!=task.table:raise ValueError('Confirmation table mismatch')
                fingerprint=json.dumps({'source':data.get('source'),'robot_id':data.get('robot_id') or robot},sort_keys=True)
                with self.database_lock:old=self.db.execute('SELECT fingerprint,reply FROM signals WHERE request_id=? AND action=?',(rid,action)).fetchone()
                if old:
                    if old[0]!=fingerprint:raise ValueError('Confirmation content conflict')
                    reply=json.loads(old[1]);self.publish_msg(self.reply_pub,reply);return reply
                if action in ('LOAD','COMPLETE','COLLECTED') and supervisor.get('request_id')==rid and action in supervisor.get('accepted_signals',[]):
                    if data.get('source')!=('kitchen' if action=='LOAD' else 'table'):raise ValueError('Wrong confirmation origin')
                    reply={'ok':True,'action':action,'request_id':rid,'robot':robot}
                    with self.database_lock:
                        self.db.execute('INSERT INTO signals(request_id,action,fingerprint,reply,applied) VALUES (?,?,?,?,1)',(rid,action,fingerprint,json.dumps(reply)));self.db.commit()
                    self.publish_msg(self.reply_pub,reply);return reply
                if action=='CANCEL':
                    if task.state=='QUEUED':
                        if rid in self.core.queue:self.core.queue.remove(rid)
                        task.state='CANCELED';self.save_task(task)
                    else:self.publish_msg(self.commands[robot],{'command':'CANCEL','request_id':rid})
                else:
                    if supervisor.get('request_id')!=rid or not supervisor.get('confirmation_pose_valid') or not may_confirm(action,state,task.mode):
                        raise ValueError('Robot not in corresponding confirmation state: '+str(state))
                    expected='kitchen' if action=='LOAD' else 'table'
                    if data.get('source')!=expected:raise ValueError('Wrong confirmation origin')
                    self.publish_msg(self.complete[robot],{'command':action,'request_id':rid})
                self.record('TASK_COMMAND',{'request_id':rid,'action':action,'robot':robot})
                reply={'ok':True,'action':action,'request_id':rid,'robot':robot}
                with self.database_lock:
                    self.db.execute('INSERT INTO signals(request_id,action,fingerprint,reply) VALUES (?,?,?,?)',(rid,action,fingerprint,json.dumps(reply)));self.db.commit()
            else:raise ValueError('Unknown action')
            self.last_error=None
        except (ValueError,KeyError) as exc:
            self.last_error=str(exc)
            reply={'ok':False,'action':action,'request_id':rid,'error':self.last_error}
            self.get_logger().warning('Request rejected: '+self.last_error)
        self.publish_msg(self.reply_pub,reply)
        self.record('FLEET_REPLY',reply)
        return reply

    def kitchen_busy(self):
        # Include newly ASSIGNED jobs before a mission-status heartbeat arrives.
        for r,rid in self.core.active.items():
            task=self.core.tasks.get(rid)
            status = self.status[r]
            # A temporary Nav2 recovery does not release the pickup destination.
            # Otherwise the next robot is dispatched into the same kitchen while
            # the first is still recovering its approach there.
            recovering_pickup = status.get('state') == 'RECOVERY_WAIT' and status.get('target') == 'kitchen'
            if task and task.mode in ('KITCHEN_PICKUP','GO_KITCHEN') and (task.state=='ASSIGNED' or status.get('state') in ('GOING_PICKUP','GOING_KITCHEN','WAIT_LOAD') or recovering_pickup):
                return True
        return False

    def _finish_yield_probe(self):
        probe=self.yield_probe
        if not probe:return
        self.yield_probe=None
        if (any(r not in self.blocked_since or not self.traffic.get(r) or
                distance(self.poses.get(r,probe['robot_poses'][r]),probe['robot_poses'][r])
                    >self.limits['fleet']['robot_separation_m']/2 for r in ROBOT_IDS)):
            self.record('YIELD_PROBE_STALE',{'robot_poses':self.poses})
            self.yield_attempted.clear()
            return
        choice=choose_yield(priority_yield_options(probe['results'],probe.get('priority')),self.last_yielder)
        if not choice:
            self.last_error='No Nav2-verified traffic yield route; both robots remain safely held'
            self.record('YIELD_NO_SAFE_ROUTE',{'rejected':probe['rejected'],'priority':probe.get('priority')})
            if probe.get('proactive') and probe.get('priority'):
                probe['priority_route_blocked']=True;probe['retry_at']=time.monotonic()+5.
                self.yield_probe=probe
            return
        loser=choice['robot'];winner=next(r for r in ROBOT_IDS if r!=loser)
        self.last_yielder=loser
        self.recovery={'loser':loser,'winner':winner,'started':time.monotonic(),
                       'blocked_goal':self.status[loser].get('target_pose'),
                       'priority':probe.get('priority'),'proactive':probe.get('proactive',False),'reserved_zones':probe.get('reserved_zones',[]),
                       'winner_start':probe['robot_poses'][winner],
                       'blocker_start':probe['robot_poses'][loser],
                       'yield_pose':choice['pose'],'route_length_m':choice['length_m'],
                       'peer_clearance_m':choice['peer_clearance_m'],
                       'route_sha256':choice['route_sha256']}
        self.publish_msg(self.commands[loser],{'command':'YIELD','robot_id':loser,
                         'request_id':self.status[loser].get('request_id'),
                         'pose':choice['pose']})
        self.record('YIELD_RETREAT_REQUEST',self.recovery)

    def _yield_plan_done(self,generation,robot,candidate,here,peer,reply=None,reason=None,planner_id='ForwardPath'):
        probe=self.yield_probe
        if not probe or probe['generation']!=generation:return
        accepted=False
        if reply is not None:
            try:
                result=reply.result();path=result.result.path
                if result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0:
                    points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                    proof=assess_path(points,here,candidate,peer,
                        self.limits['fleet']['robot_separation_m'],self.limits['fleet']['robot_separation_m']/2,
                        self.limits['arrival']['xy_tolerance_m'],
                        contact_clearance=2*self.limits['dynamic_obstacles']['robot_radius_m'])
                    if proof and probe.get('proactive') and any(in_rectangle(point,self.zones.zones[name]) for name in probe['reserved_zones'] for point in points):
                        proof=None;reason='YIELD_ROUTE_ENTERS_RESERVED_CORRIDOR'
                    if proof:
                        accepted=True
                        probe['results'].append({'robot':robot,'pose':candidate[:3],
                            'rear_turn':candidate[3],'planner_id':planner_id,
                            'turn_rad':abs(math.atan2(math.sin(candidate[2]-here[2]),math.cos(candidate[2]-here[2]))),
                            'route_sha256':hashlib.sha256(json.dumps(points).encode()).hexdigest(),**proof})
                    else:reason='PATH_TOO_CLOSE_TO_PEER_OR_STALE'
                else:reason='NAV2_NO_VALID_PATH'
            except Exception as exc:reason='NAV2_PATH_ERROR: '+str(exc)
        if reason:probe['rejected'].append({'robot':robot,'pose':candidate[:3],'planner_id':planner_id,'reason':reason})
        probe['pending']-=1;probe['busy'].discard(robot)
        if accepted:probe['queues'][robot].clear()
        elif planner_id=='ForwardPath':probe['queues'][robot].insert(0,(candidate,'GridBased'))
        self._next_yield_candidate(generation,robot)
        if self.yield_probe is probe and probe['pending']==0 and not probe['dispatching']:self._finish_yield_probe()

    def _yield_plan_response(self,f,generation,robot,candidate,here,peer,planner_id='ForwardPath'):
        probe=self.yield_probe
        if not probe or probe['generation']!=generation:return
        try:handle=f.result()
        except Exception as exc:
            self._yield_plan_done(generation,robot,candidate,here,peer,reason=str(exc),planner_id=planner_id);return
        if not handle or not handle.accepted:
            self._yield_plan_done(generation,robot,candidate,here,peer,reason='NAV2_PLAN_REJECTED',planner_id=planner_id);return
        probe['handles'].append(handle)
        handle.get_result_async().add_done_callback(
            lambda result:self._yield_plan_done(generation,robot,candidate,here,peer,reply=result,planner_id=planner_id))

    def _next_yield_candidate(self,generation,robot):
        probe=self.yield_probe
        if not probe or probe['generation']!=generation or robot in probe['busy'] or not probe['queues'][robot]:return
        candidate,planner=probe['queues'][robot].pop(0)
        here,peer=probe['contexts'][robot]
        client=self.yield_planners[robot]
        if not client.server_is_ready():
            probe['queues'][robot].clear();probe['rejected'].append({'robot':robot,'reason':'NAV2_PLANNER_UNAVAILABLE'});return
        goal=ComputePathToPose.Goal();goal.planner_id=planner;goal.use_start=False
        goal.goal.header.frame_id='map';goal.goal.header.stamp=self.get_clock().now().to_msg()
        goal.goal.pose.position.x=candidate[0];goal.goal.pose.position.y=candidate[1]
        goal.goal.pose.orientation.z=math.sin(candidate[2]/2);goal.goal.pose.orientation.w=math.cos(candidate[2]/2)
        probe['busy'].add(robot);probe['pending']+=1
        client.send_goal_async(goal).add_done_callback(
            lambda f:self._yield_plan_response(f,generation,robot,candidate,here,peer,planner))

    def _start_yield_probe(self,now,poses,goals,proactive=False,reserved_zones=None,priority=None):
        self.yield_generation+=1;generation=self.yield_generation
        probe={'generation':generation,'started':now,'pending':0,'results':[],'proactive':proactive,'reserved_zones':list(reserved_zones or []),'priority':priority,'map_signature':getattr(self,'map_signature',None),'awaiting_pause':proactive,'request_ids':{r:self.status[r].get('request_id') for r in ROBOT_IDS} if proactive else {},
               'rejected':[],'handles':[],'robot_poses':dict(poses),
               'queues':{},'contexts':{},'busy':set(),'dispatching':True}
        self.yield_probe=probe
        separation=self.limits['fleet']['robot_separation_m'];margin=self.limits['fleet']['zone_approach_margin_m']
        for robot in ROBOT_IDS:
            peer=next(r for r in ROBOT_IDS if r!=robot);here=(*poses[robot],self.pose_yaw.get(robot,0.))
            candidates=candidate_poses(here,poses[peer],self.pose_history[robot],self.zones.zones,
                self.zone_config.get('no_go',{}),self.home_poses,goals.get(peer),separation,margin)
            radius=self.limits['dynamic_obstacles']['robot_radius_m']+self.limits['dynamic_obstacles']['prediction_margin_m']
            clear_candidates=[c for c in candidates if yield_goal_clear(self.yield_grid,c,radius)]
            if priority and robot==priority['winner']:clear_candidates=[]
            probe['rejected'].extend({'robot':robot,'pose':c[:3],'reason':'YIELD_GOAL_BODY_NOT_KNOWN_FREE'} for c in candidates if c not in clear_candidates)
            probe['queues'][robot]=[(c,'ForwardPath') for c in sorted(clear_candidates,key=lambda c:c[3])[:8]]
            probe['contexts'][robot]=(here,poses[peer])
        probe['deadline_s']=2*max(1,max(len(q) for q in probe['queues'].values()))*self.limits['dynamic_obstacles']['detour_plan_timeout_s']
        if not proactive:
            for robot in ROBOT_IDS:self._next_yield_candidate(generation,robot)
        probe['dispatching']=False
        self.record('YIELD_ROUTE_PROBE',{'candidates':sum(len(q) for q in probe['queues'].values())+probe['pending'],
                    'robot_poses':poses,'goals':goals,'sequential_per_robot':True,'proactive':proactive,'reserved_zones':probe['reserved_zones']})
        if probe['pending']==0 and not probe['awaiting_pause']:self._finish_yield_probe()

    def tick(self):
        while True:
            try:body,resq,received_at=self.http_queue.get_nowait()
            except queue.Empty:break
            reply=self.process(body,received_at=received_at)
            if resq is not None:resq.put(reply)
        now=time.monotonic()
        # Persisted confirmations are retried until Supervisor records application.
        # Its own durable command guard makes this safe across reconnect/restart.
        with self.database_lock:pending_signals=self.db.execute('SELECT request_id,action FROM signals WHERE applied=0').fetchall()
        for rid,action in pending_signals:
            if action not in ('LOAD','COMPLETE','COLLECTED'):continue
            task=self.core.tasks.get(rid)
            if not task:continue
            current=self.status[task.robot]
            if current.get('request_id')!=rid or not current.get('confirmation_pose_valid') or not may_confirm(action,current.get('state'),task.mode):continue
            key=(rid,action)
            if now-self.signal_sent_wall.get(key,0.)>=1.:
                self.signal_sent_wall[key]=now;self.publish_msg(self.complete[task.robot],{'command':action,'request_id':rid});self.record('CONFIRM_OUTBOX_RETRY',{'request_id':rid,'robot_id':task.robot,'action':action})
        for r in ROBOT_IDS:
            try:
                t=self.tf.lookup_transform('map',r+'/base_footprint',Time())
                age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if age<1.:
                    self.poses[r]=(t.transform.translation.x,t.transform.translation.y);self.pose_wall[r]=now
                    q=t.transform.rotation
                    self.pose_yaw[r]=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    history=self.pose_history[r]
                    if not history or distance(history[-1],self.poses[r])>=self.limits['navigation']['stuck_distance_m']:
                        history.append(self.poses[r])
            except Exception:pass
        if self.yield_probe and self.yield_probe.get('priority_route_blocked'):
            probe=self.yield_probe
            if any(self.status[r].get('request_id')!=rid or self.status[r].get('state') in ('CANCELED','ERROR','COMPLETED') for r,rid in probe['request_ids'].items()):
                self.yield_probe=None;self.yield_attempted.clear()
            elif now>=probe['retry_at']:
                self.yield_probe=None
                self._start_yield_probe(now,dict(self.poses),{r:self.status[r].get('target_pose') for r in ROBOT_IDS},
                    proactive=True,reserved_zones=probe['reserved_zones'],priority=probe['priority'])
        if self.yield_probe and self.yield_probe.get('awaiting_pause'):
            probe=self.yield_probe
            if preentry_pause_acknowledged(self.status,probe['request_ids']):
                probe['awaiting_pause']=False;probe['dispatching']=True
                self.record('PREENTRY_PAUSE_ACK',{'request_ids':probe['request_ids']})
                for robot in ROBOT_IDS:self._next_yield_candidate(probe['generation'],robot)
                probe['dispatching']=False
                if self.yield_probe is probe and probe['pending']==0:self._finish_yield_probe()
            elif now-probe['started']>5.:
                self.record('PREENTRY_PAUSE_TIMEOUT',{'request_ids':probe['request_ids']})
                self.yield_probe=None;self.yield_attempted.clear();self.preentry_retry_after=now+5.
        if self.yield_probe and not self.yield_probe.get('priority_route_blocked') and now-self.yield_probe['started']>self.yield_probe.get('deadline_s',self.limits['dynamic_obstacles']['detour_plan_timeout_s']):
            for handle in self.yield_probe['handles']:handle.cancel_goal_async()
            self.yield_probe['rejected'].append({'reason':'NAV2_PLAN_TIMEOUT'})
            self._finish_yield_probe()
        for r in ROBOT_IDS:
            supervisor_fresh=now-self.status_wall.get(r,-1e9)<2.
            pose_valid=supervisor_fresh and (now-self.pose_wall.get(r,-1e9) < 2.0 and now-self.odom_wall.get(r,-1e9)<1.0)
            scan_valid=(now-self.scan_wall.get(r,-1e9) < 1.5)
            self.traffic[r]=not (pose_valid and scan_valid)
        health_hold=dict(self.traffic)
        # Fleet-owned zone arbitration and collision separation.
        current_map_poses={r:p for r,p in self.poses.items() if now-self.pose_wall.get(r,-1e9)<2.0}
        arrival_poses={r:p for r,p in current_map_poses.items() if now-self.pose_wall.get(r,-1e9)<.5}
        if not self.arrival_ready:
            self.arrival_ready=(len(arrival_poses)==len(ROBOT_IDS) and not any(health_hold.values()))
        if self.arrival_ready and not self.map_change_hold:
            self.arrival_priority.observe(arrival_poses,self.zones.zones,now,
                self.limits['fleet']['zone_approach_margin_m']+self.limits['dynamic_obstacles']['robot_radius_m'])
        traffic_conflict=set(self.zones.update(self.poses,self.limits['fleet']['zone_approach_margin_m']))
        # Reciprocal destination occupancy can stop both planners before they move.
        # Treat it as a mutual traffic block only while both missions and poses are fresh.
        goals={r:self.status[r].get('target_pose') for r in ROBOT_IDS}
        active=all(self.status[r].get('state') in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','RECOVERY_WAIT')
                   and now-self.status_wall.get(r,-1e9)<2. and r in current_map_poses
                   and isinstance(goals[r],(list,tuple)) and len(goals[r])>=2 for r in ROBOT_IDS)
        intent_routes,intent_sources=reservation_intents(current_map_poses,goals,self.route_paths,self.route_context,
            {r:(self.status[r].get('request_id'),tuple(goals[r] or ())) for r in ROBOT_IDS},self.route_wall,now)
        early_zones=[];early_priority=None
        if (not self.map_change_hold and self.preentry_zones and now>=self.preentry_retry_after and active and self.recovery is None and self.yield_probe is None
                and not self.yield_attempted and not any(health_hold.values())
                and distance(current_map_poses['robot_01'],current_map_poses['robot_02'])<=self.limits['dynamic_obstacles']['recognition_m']):
            early_zones=opposed_corridor_paths(current_map_poses,goals,intent_routes,
                {name:self.zones.zones[name] for name in self.preentry_zones})
            early_priority=self.corridor_priority_for(early_zones,current_map_poses)
            if early_priority:
                early_priority['route_sources']=dict(intent_sources)
                winner=early_priority['winner'];name=early_priority['zone']
                report=self.dynamic.get(winner,{})
                if forward_entry_observed(current_map_poses[winner],self.pose_yaw.get(winner,0.),self.zones.zones[name],report,self.yield_grid,
                        self.limits['dynamic_obstacles']['recognition_m'],self.limits['dynamic_obstacles']['front_clear_half_angle_rad'],path=intent_routes.get(winner,[])):
                    early_zones=[name]
                else:early_zones=[]
            else:early_zones=[]

        if early_zones or reservation_setup_pending(self.yield_probe,self.recovery,self.status):traffic_conflict.update(ROBOT_IDS)
        mutual_goal_occupancy=(active and distance(goals['robot_01'],goals['robot_02'])>self.limits['fleet']['robot_separation_m']
            and distance(current_map_poses['robot_01'],goals['robot_02'])<self.limits['fleet']['robot_separation_m']
            and distance(current_map_poses['robot_02'],goals['robot_01'])<self.limits['fleet']['robot_separation_m'])
        if mutual_goal_occupancy or mutual_peer_block(self.status,self.dynamic,traffic_conflict):traffic_conflict.update(ROBOT_IDS)
        for r in traffic_conflict:self.traffic[r]=True
        # Close robots both stop until clear; a verified route decides who yields
        # only if this remains a mutual hold for the configured period.
        if all(r in current_map_poses for r in ROBOT_IDS):
            if distance(current_map_poses['robot_01'],current_map_poses['robot_02'])<self.limits['fleet']['robot_separation_m']:
                traffic_conflict.update(ROBOT_IDS)
                for robot in ROBOT_IDS:self.traffic[robot]=True
        if self.recovery:
            loser=self.recovery['loser'];winner=self.recovery['winner']
            st=self.status[loser].get('state')
            safe=(now-self.pose_wall.get(loser,-1e9)<2. and now-self.scan_wall.get(loser,-1e9)<1.5)
            if st=='YIELDING':
                self.traffic[loser]=not safe;self.traffic[winner]=True
                if self.recovery.get('proactive') and reserved_winner_clear(current_map_poses,winner,loser,self.zones.zones,self.recovery['reserved_zones'],self.limits['fleet']['robot_separation_m'],self.limits['dynamic_obstacles']['robot_radius_m']):
                    self.traffic[winner]=health_hold[winner]
            elif st=='YIELD_WAIT':
                self.traffic[loser]=True
                if self.recovery.get('proactive') and reserved_winner_clear(current_map_poses,winner,loser,self.zones.zones,self.recovery['reserved_zones'],self.limits['fleet']['robot_separation_m'],self.limits['dynamic_obstacles']['robot_radius_m']):
                    self.traffic[winner]=health_hold[winner]
                occupied=any(in_bounds(current_map_poses.get(winner),b,.45) for b in self.zones.zones.values())
                blocked_goal=self.recovery.get('blocked_goal')
                goal_clear=(not blocked_goal or
                            (winner in current_map_poses and
                             distance(current_map_poses[winner],blocked_goal[:2])>=self.limits['fleet']['robot_separation_m']))
                winner_status=self.status[winner]
                winner_arrived=(winner_status.get('state') in ('WAIT_LOAD','WAIT_HANDOVER','WAIT_COLLECTION','COMPLETED')
                    and winner_status.get('goal_execution_verified_idle') is True
                    and winner_status.get('confirmation_pose_valid') is True)
                passage_clear=winner_arrived or passed_blocking_position(current_map_poses.get(winner),
                    self.recovery.get('winner_start'),self.recovery.get('blocker_start'),
                    self.limits['fleet']['robot_separation_m'])
                if not occupied and goal_clear and safe and passage_clear:
                    self.publish_msg(self.commands[loser],{'command':'RESUME','robot_id':loser})
                    self.record('YIELD_RECOVERY_COMPLETE',self.recovery);self.recovery=None;self.last_error=None
            elif st in ('ERROR','CANCELED','COMPLETED') or now-self.recovery['started']>self.limits['navigation']['goal_timeout_s']:
                self.record('YIELD_RECOVERY_FAILED',self.recovery);self.recovery=None
        for r in ROBOT_IDS:
            if r in traffic_conflict and self.traffic[r] and self.status[r].get('state') in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','RECOVERY_WAIT'):
                self.blocked_since.setdefault(r,now)
                blocked_for=now-self.blocked_since[r]
                if r not in self.block_alerted and blocked_for>=self.limits['fleet']['blocked_alert_s']:
                    self.block_alerted.add(r)
                    detail={'robot':r,'seconds':round(blocked_for,2),
                            'reason':'FLEET_TRAFFIC_HOLD'}
                    self.record('FLEET_TRAFFIC_BLOCKED',detail)
                    self.last_error='Fleet traffic hold exceeded configured limit: '+r
                peer=next(bot for bot in ROBOT_IDS if bot!=r)
                peer_blocked=self.blocked_since.get(peer)
                mutual_hold=(self.traffic.get(peer,False) and peer_blocked is not None
                             and now-peer_blocked>=self.limits['fleet']['blocked_alert_s'])
                if (blocked_for>=self.limits['fleet']['blocked_alert_s'] and mutual_hold
                        and self.recovery is None and self.yield_probe is None
                        and not self.yield_attempted
                        and all(bot in current_map_poses for bot in ROBOT_IDS)):
                    self.yield_attempted.update(ROBOT_IDS)
                    self._start_yield_probe(now,current_map_poses,goals,priority=self.corridor_priority_for(self.preentry_zones,current_map_poses))
            else:
                self.blocked_since.pop(r,None);self.block_alerted.discard(r)
                self.yield_attempted.discard(r)
            if self.map_change_hold:self.traffic[r]=True
            self.holds[r].publish(Bool(data=bool(self.traffic[r])))
        if early_zones and self.recovery is None and self.yield_probe is None and not self.yield_attempted:
            self.yield_attempted.update(ROBOT_IDS)
            self.record('PREENTRY_RESERVATION_REQUEST',{'zones':early_zones,'robot_poses':current_map_poses,'priority':early_priority})
            self._start_yield_probe(now,current_map_poses,goals,proactive=True,reserved_zones=early_zones,priority=early_priority)
        ready={r for r in ROBOT_IDS if self.status[r].get('state') in ('IDLE','COMPLETED','CANCELED') and not self.traffic[r] and self.status[r].get('map_valid') is True and self.status[r].get('nav_ready') is True and self.commands[r].get_subscription_count()>0}
        def approach_ready(task):
            if task.table is None:return True
            chosen=available_approach(self.destinations,task.table,task.assigned_side or task.side,
                current_map_poses.get(task.robot),[p for r,p in current_map_poses.items() if r!=task.robot],
                self.limits['fleet']['robot_separation_m'])
            if chosen is None:return False
            task.assigned_side=chosen;return True
        task=self.core.dispatch_next(ready,kitchen_free=not self.kitchen_busy(),acceptable=approach_ready)
        if task:
            self.save_task(task)
            self.publish_msg(self.commands[task.robot],{'command':'START','request_id':task.request_id,
                'robot_id':task.robot,'table':task.table,'mode':task.mode,'side':task.assigned_side or task.side})
            self.record('TASK_ASSIGNED',vars(task))
        # An idle return is a separate, persisted Fleet request, never part of
        # delivery completion. Any queued customer work takes precedence.
        if self.auto_home_when_idle and not self.core.queue:
            for index,r in enumerate(ROBOT_IDS):
                st=self.status[r]
                if should_return_home(state=st.get('state'),mode=st.get('mode'),
                        active=r in self.core.active,queued=bool(self.core.queue),ready=r in ready,
                        pose=current_map_poses.get(r),home=self.home_poses[index],
                        tolerance=self.limits['arrival']['xy_tolerance_m']):
                    self.enqueue_request({'action':'SUBMIT','request_id':'idle-home-'+uuid.uuid4().hex,
                                  'robot_id':r,'mode':'GO_HOME'})
                    self.record('IDLE_HOME_QUEUED',{'robot_id':r})
        snapshot=self.core.snapshot()
        self.status_seq+=1
        snapshot.update({'protocol_version':1,'site_id':self.site_id,'profile_revision':self.profile_revision,
                         'session_id':self.session_id,'status_seq':self.status_seq,
                         'pose_metadata':{r:{'x':xy[0],'y':xy[1],'yaw':self.pose_yaw.get(r), 'frame_id':'map',
                             'pose_age_s':max(0.,time.monotonic()-self.pose_wall.get(r,0.)),
                             'status_age_s':max(0.,time.monotonic()-self.status_wall.get(r,0.))}
                             for r,xy in self.poses.items()}})
        snapshot.update({'corridor_arrivals':self.arrival_priority.entries,'map_change_hold':self.map_change_hold,'destinations':sorted(self.table_names),'supervisors':self.status,'robot_poses':self.poses,'traffic_hold':self.traffic,
                         'last_error':self.last_error,'http_port':self.http_port,'safety':self.safety,'zone_owner':dict(self.zones.owners),'recovery':self.recovery,'dynamic':self.dynamic,'dynamic_events':self.dynamic_events,'navigation_events':self.navigation_events})
        self.publish_msg(self.pub,snapshot)
        with self.snapshot_lock:self.latest_status=snapshot

    def start_http(self):
        port=self.http_port; parent=self
        web=Path(__file__).resolve().parent.parent/'web'/'dashboard.html'
        # In symlink or installed builds, locate the web artifact in the package share.
        try:
            from ament_index_python.packages import get_package_share_directory
            web=Path(get_package_share_directory('cafe_autonomy'))/'web'/'dashboard.html'
        except Exception:pass
        class Handler(BaseHTTPRequestHandler):
            def respond(self,obj,code=200):
                data=json.dumps(obj,ensure_ascii=False,default=str).encode()
                self.send_response(code);self.send_header('Content-Type','application/json; charset=utf-8')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            def do_GET(self):
                if self.path in ('/','/dashboard'):
                    if not web.exists():return self.respond({'error':'dashboard missing'},404)
                    data=web.read_bytes();self.send_response(200)
                    self.send_header('Content-Type','text/html; charset=utf-8')
                    self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
                elif self.path=='/api/status':
                    with parent.snapshot_lock:data=dict(parent.latest_status)
                    self.respond(data)
                elif self.path=='/api/history':
                    with parent.database_lock:
                        rows=parent.db.execute('SELECT stamp,event,payload FROM events ORDER BY id DESC LIMIT 100').fetchall()
                    self.respond([{'time':r[0],'event':r[1],'payload':jparse(r[2])} for r in rows])
                elif self.path=='/health':self.respond({'ok':True})
                else:self.respond({'error':'not found'},404)
            def do_POST(self):
                if self.path!='/api/request':return self.respond({'error':'not found'},404)
                try:
                    n=int(self.headers.get('Content-Length','0'))
                    if n<=0 or n>16384:raise ValueError('Invalid request size')
                    payload=json.loads(self.rfile.read(n))
                    q=queue.Queue(maxsize=1);parent.enqueue_request(payload,q)
                    self.respond(q.get(timeout=3))
                except Exception as ex:self.respond({'ok':False,'error':str(ex)},400)
            def log_message(self,*args):pass
        try:
            server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
            threading.Thread(target=server.serve_forever,daemon=True).start()
        except OSError as exc:
            self.get_logger().error('HTTP dashboard disabled: '+str(exc))

def main():
    rclpy.init();node=Fleet()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
