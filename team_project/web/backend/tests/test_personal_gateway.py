"""HTTP gateway contract tests. These are simulated Fleet tests, not Gazebo evidence."""
import copy
import sys
import threading
from pathlib import Path
import pytest
from fastapi.testclient import TestClient

from app.config import Settings
from app.personal import Gateway, Problem, create_personal_app

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros_ws/src/cafe_autonomy'))
from cafe_autonomy.policy import FleetCore, may_confirm
from cafe_autonomy.home_policy import home_refusal


class SimulatedFleet:
    def __init__(self):
        self.core = FleetCore(); self.identity = {}; self.seq = 0; self.freeze = False
        self.fail_response = False; self.reject = False; self.calls = []; self.signals = {}
        self.supervisors = {r: {'state': 'IDLE'} for r in ('robot_01', 'robot_02')}
        self.poses = {'robot_01': [1., 0.], 'robot_02': [5.6, 0.]}
        self.goals = []; self.transitions = []

    def status(self):
        if not self.freeze: self.seq += 1
        return {**self.identity, 'protocol_version': 1, 'session_id': 'mock-session', 'status_seq': self.seq,
                **self.core.snapshot(), 'supervisors': copy.deepcopy(self.supervisors),
                'robot_poses': self.poses.copy(),
                'pose_metadata': {r: dict(x=p[0],y=p[1],yaw=0.,frame_id='map',pose_age_s=0.,status_age_s=0.)
                                  for r,p in self.poses.items()}}

    def send(self, wire):
        self.calls.append(copy.deepcopy(wire)); rid = wire['request_id']; action = wire['action']
        if self.reject:
            return dict(ok=False, action=action, request_id=rid, error='BUSINESS_REJECTION')
        if action == 'SUBMIT':
            robot = wire.get('robot_id', 'robot_01'); mode = wire['mode']
            if rid not in self.core.tasks and mode == 'GO_HOME':
                reason = home_refusal(robot, {k:vars(t) for k,t in self.core.tasks.items()},
                                      self.supervisors, self.poses[robot], (0.,0.) if robot=='robot_01' else (5.6,0.), .1, self.core.queue)
                if reason: return dict(ok=False, action=action, request_id=rid, error=reason)
            task,new = self.core.submit(rid,wire.get('table'),robot,mode,wire.get('side','auto'))
            reply = dict(ok=True,action=action,request_id=rid,new=new,robot_id=robot)
        else:
            key = (rid,action)
            if key in self.signals: return self.signals[key]
            task = self.core.tasks[rid]; st = self.supervisors[task.robot]
            if action == 'CANCEL':
                reply = dict(ok=True,action=action,request_id=rid)
                self.signals[key] = reply
                return reply  # acknowledgement is deliberately not physical cancellation
            if not may_confirm(action,st['state'],task.mode) or wire['source'] != ('kitchen' if action=='LOAD' else 'table'):
                return dict(ok=False,action=action,request_id=rid,error='WRONG_STATE')
            self.transitions.append(key)
            if action == 'LOAD': st['state']='GOING_TABLE'; self.goals.append((rid,'table'))
            else: st['state']='COMPLETED'; self.core.update(task.robot,'COMPLETED',rid)
            reply = dict(ok=True,action=action,request_id=rid)
            self.signals[key] = reply
        if self.fail_response:
            self.fail_response=False
            raise OSError('response lost after accepting the request')
        return reply

    def begin(self,rid,phase):
        task=self.core.tasks[rid]
        if rid in self.core.queue: self.core.queue.remove(rid)
        self.core.active[task.robot]=rid;task.state='RUNNING'
        self.supervisors[task.robot]=dict(state=phase,request_id=rid,confirmation_pose_valid=True)
        self.goals.append((rid,'initial'))


@pytest.fixture
def pair(tmp_path):
    cfg=Settings(adapter='personal',repo=str(ROOT),personal_config='config/integration/system.yaml',
                 personal_db=str(tmp_path/'gateway.sqlite3'))
    sim=SimulatedFleet();gateway=Gateway(cfg,sim)
    sim.identity={k:gateway.profile[k] for k in ('site_id','profile_revision')}
    gateway.pump()
    yield gateway,sim,cfg
    if not gateway.closed:gateway.close()


def order(mode='DELIVER_DIRECT',client='client-one'):
    return dict(type='SERVE',mode=mode,destination_id='table_03',client_request_id=client)


def test_same_order_concurrently_is_one_logical_execution(pair):
    g,s,_=pair;answers=[]
    threads=[threading.Thread(target=lambda:answers.append(g.create(order()))) for _ in range(2)]
    for t in threads:t.start()
    for t in threads:t.join()
    assert len(answers)==2 and sum(new for _,new in answers)==1
    g.pump();g.pump()
    rid=answers[0][0]['request_id'];s.begin(rid,'GOING_DIRECT')
    assert len(s.core.tasks)==1 and s.goals==[(rid,'initial')]
    with pytest.raises(Problem,match=''):g.create(order('TABLE_COLLECT'))


def test_response_loss_restart_same_id_and_fifo(pair):
    g,s,cfg=pair
    first,_=g.create(order());second,_=g.create(order(client='second'))
    s.fail_response=True;g.pump()
    assert len(s.core.tasks)==1
    g.close();g=Gateway(cfg,s)
    try:
        g.pump();g.pump();g.pump()
        assert [t.request_id for t in s.core.tasks.values()]==[first['request_id'],second['request_id']]
        assert g.create(order())[0]['request_id']==first['request_id']
        assert len(s.core.tasks)==2 and s.core.tasks[first['request_id']].received_seq==1
    finally:g.close()


@pytest.mark.parametrize('mode,action,phase,source',[
 ('KITCHEN_PICKUP','LOAD','WAIT_LOAD','kitchen'),
 ('DELIVER_DIRECT','COMPLETE','WAIT_HANDOVER','table'),
 ('TABLE_COLLECT','COLLECTED','WAIT_COLLECTION','table')])
def test_confirmation_twice_before_and_after_transition(pair,mode,action,phase,source):
    g,s,_=pair;t,_=g.create(order(mode));g.pump();s.begin(t['request_id'],phase);g.pump()
    body=dict(action=action,source=source)
    g.signal(t['task_id'],body);g.signal(t['task_id'],body);g.pump();g.pump()
    g.signal(t['task_id'],body);g.pump()
    assert s.transitions==[(t['request_id'],action)]
    assert sum(w['action']==action for w in s.calls)==1
    with pytest.raises(Problem):g.signal(t['task_id'],dict(action=action,source='wrong'))


def test_wrong_confirmation_and_home_during_delivery(pair):
    g,s,_=pair;t,_=g.create(order());g.pump();s.begin(t['request_id'],'GOING_DIRECT');g.pump()
    for _ in range(2):
        with pytest.raises(Problem):g.signal(t['task_id'],dict(action='COMPLETE',source='table'))
    for client in ('home-one','home-one','home-two'):
        with pytest.raises(Problem) as exc:g.home('robot_01',client)
        assert exc.value.code=='ROBOT_BUSY'
    assert len(s.core.tasks)==1 and len(s.goals)==1


def test_home_different_ids_and_later_normal_return(pair):
    g,s,_=pair
    g.home('robot_01','home-one')
    with pytest.raises(Problem) as exc:g.home('robot_01','home-two')
    assert exc.value.code=='ALREADY_RETURNING'
    g.home('robot_01','home-one');g.pump();g.pump()
    with pytest.raises(Problem):g.home('robot_01','home-three')
    assert len(s.core.tasks)==1
    rid=next(iter(s.core.tasks));s.begin(rid,'RETURNING_HOME')
    s.supervisors['robot_01']['state']='COMPLETED';s.core.update('robot_01','COMPLETED',rid)
    s.poses['robot_01']=[0.,0.];g.pump()
    with pytest.raises(Problem) as exc:g.home('robot_01','at-home')
    assert exc.value.code=='ALREADY_HOME'
    # A later mission has moved the robot away: old HOME history must not permanently lock it.
    s.poses['robot_01']=[1.,0.];g.pump();g.home('robot_01','later-home');g.pump()
    assert len(s.core.tasks)==2


def test_auto_and_manual_home_share_fleet_admission(pair):
    g,s,_=pair
    g.home('robot_01','manual')
    s.send(dict(action='SUBMIT',request_id='idle-home-test',robot_id='robot_01',mode='GO_HOME'))
    g.pump();g.pump()
    assert len(s.core.tasks)==1
    controls=g.view()['controls'];assert controls[0]['submission']=='REJECTED'


def test_stale_profile_and_rejection_are_not_success(pair):
    g,s,_=pair
    s.identity['profile_revision']='different';g.pump()
    with pytest.raises(Problem):g.create(order())
    s.identity['profile_revision']=g.profile['profile_revision'];g.pump()
    t,_=g.create(order());s.reject=True;g.pump()
    assert g.view()['tasks'][0]['submission']=='REJECTED'
    assert g.view()['tasks'][0]['status']=='FAILED' and not s.core.tasks
    s.freeze=True;g.fresh_at-=3.;g.pump()
    assert not g.ready() and g.view()['tasks'][0]['status']=='FAILED'


def test_cancel_acknowledgement_does_not_finish_task(pair):
    g,s,_=pair;t,_=g.create(order());g.pump();s.begin(t['request_id'],'GOING_DIRECT');g.pump()
    g.cancel(t['task_id']);g.pump();g.pump()
    assert g.get_task(t['task_id'])['status']=='RUNNING'
    assert g.get_task(t['task_id'])['cancel_requested']


def test_personal_http_map_websocket_and_unsupported_writes(tmp_path):
    cfg=Settings(adapter='personal',repo=str(ROOT),personal_config='config/integration/system.yaml',
                 personal_db=str(tmp_path/'gateway.sqlite3'))
    sim=SimulatedFleet();app=create_personal_app(cfg,sim);g=app.state.gateway
    sim.identity={k:g.profile[k] for k in ('site_id','profile_revision')};g.pump()
    with TestClient(app) as client:
        assert client.get('/api/capabilities').json()['execution_owner']=='personal_fleet'
        meta=client.get('/api/stores/demo_store/map').json()
        assert meta['origin']['x']==pytest.approx(-.779)
        assert client.get(meta['image_url']).headers['content-type']=='image/png'
        semantics=client.get('/api/stores/demo_store/semantics').json()
        assert any(w['entity_id']=='table_03' for w in semantics['waypoints'])
        r=client.post('/api/tasks',json=order());assert r.status_code==202
        assert client.post('/api/tasks',json=order()).status_code==200
        assert client.post('/api/mapping/start',json={}).status_code==409
        assert client.post('/api/robots/robot_01/reset',json={}).status_code==409
        with client.websocket_connect('/ws/state') as ws:
            event=ws.receive_json();assert event['type']=='robot_state'


def test_world_map_reference_points_and_rotated_grid():
    import math
    for world,expected in [((-3.2,-2.25),(0.,0.)),((2.4,-2.25),(5.6,0.))]:
        assert (world[0]+3.2,world[1]+2.25)==pytest.approx(expected)
    # Independently check the documented inverse rotation and image vertical flip.
    theta=math.pi/2;origin=(2.,3.);local=(.5,1.)
    p=(origin[0]-local[1],origin[1]+local[0])
    q=(math.cos(theta)*(p[0]-origin[0])+math.sin(theta)*(p[1]-origin[1]),
       -math.sin(theta)*(p[0]-origin[0])+math.cos(theta)*(p[1]-origin[1]))
    assert q==pytest.approx(local)
    assert (q[0]/.05,100-q[1]/.05)==pytest.approx((10.,80.))


@pytest.mark.parametrize('destination', ['meet_left', 'meet_right', 'human_lane_goal'])
def test_plain_coordinate_destinations_match_personal_fleet(tmp_path, destination):
    cfg=Settings(adapter='personal',repo=str(ROOT),
                 personal_config='config/integration/sites/validation_priority_first.yaml',
                 personal_db=str(tmp_path/'plain.sqlite3'))
    sim=SimulatedFleet();g=Gateway(cfg,sim)
    sim.identity={k:g.profile[k] for k in ('site_id','profile_revision')}
    try:
        g.pump();body=order();body['destination_id']=destination
        task,new=g.create(body);g.pump()
        assert new and sim.core.tasks[task['request_id']].table==destination
        assert 'kitchen' not in g.tables and 'robot_01_HOME' not in g.tables
    finally:g.close()
