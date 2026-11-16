from types import SimpleNamespace,MethodType
from rclpy.time import Time
from cafe_autonomy.policy import ZoneArbiter
from cafe_autonomy.yield_policy import assess_path
from cafe_autonomy.fleet import Fleet
import cafe_autonomy.fleet as module

def test_first_approaching_robot_keeps_grant_until_clear():
    arb=ZoneArbiter(zones={'lane':(0.,1.,0.,1.)})
    arb.update({'robot_01':(-2.,.5),'robot_02':(-.3,.5)})
    hold=arb.update({'robot_01':(-.2,.5),'robot_02':(-.3,.5)})
    assert arb.owners['lane']=='robot_02' and 'robot_01' in hold
    arb.update({'robot_01':(.2,.5),'robot_02':(-.3,.5)})
    assert arb.owners['lane']=='robot_01'  # actual occupant always wins

def test_close_robot_can_only_escape_monotonically_away():
    assert assess_path([(0.,0.),(-.5,0.),(-1.5,0.)],(0.,0.),(-1.5,0.),(.85,0.),.9,.1,.1,.66)['escape_from_close_start']
    assert assess_path([(0.,0.),(.05,0.),(-1.5,0.)],(0.,0.),(-1.5,0.),(.85,0.),.9,.1,.1,.66) is None
    assert assess_path([(0.,0.),(-1.5,0.)],(0.,0.),(-1.5,0.),(.5,0.),.9,.1,.1,.66) is None

def test_candidate_queries_do_not_preempt_each_other(monkeypatch):
    class Future:
        def add_done_callback(self,cb):self.cb=cb
        def result(self):return None
    class Client:
        def __init__(self):self.pending=[];self.sent=[]
        def server_is_ready(self):return True
        def send_goal_async(self,goal):
            assert not self.pending
            f=Future();self.pending.append(f);self.sent.append(goal.planner_id);return f
        def reject(self):
            f=self.pending.pop(0);f.cb(f)
    monkeypatch.setattr(module,'candidate_poses',lambda *a:[(-2.,-2.,0.,False),(-3.,-2.,0.,False)])
    monkeypatch.setattr(module,'yield_goal_clear',lambda *a:True)
    clients={r:Client() for r in ('robot_01','robot_02')}
    m=SimpleNamespace(yield_generation=0,yield_grid=None,pose_yaw={},pose_history={r:[] for r in clients},
        zones=SimpleNamespace(zones={}),zone_config={},home_poses=[],yield_planners=clients,
        limits={'fleet':{'robot_separation_m':.9,'zone_approach_margin_m':.45},'dynamic_obstacles':{'detour_plan_timeout_s':5.,'robot_radius_m':.33,'prediction_margin_m':.15}},
        get_clock=lambda:SimpleNamespace(now=lambda:Time()),record=lambda *a:None,finished=0)
    def finish():m.finished+=1;m.yield_probe=None
    m._finish_yield_probe=finish
    for name in ('_next_yield_candidate','_yield_plan_response','_yield_plan_done'):
        setattr(m,name,MethodType(getattr(Fleet,name),m))
    Fleet._start_yield_probe(m,0.,{'robot_01':(0.,0.),'robot_02':(2.,0.)},{})
    assert all(len(c.pending)==1 for c in clients.values())
    clients['robot_01'].reject()
    assert clients['robot_01'].sent==['ForwardPath','GridBased']
    assert len(clients['robot_02'].sent)==1
    for _ in range(10):
        for c in clients.values():
            if c.pending:c.reject()
    assert m.finished==1
