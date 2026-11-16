from types import SimpleNamespace as NS
from cafe_autonomy.mission import Mission

def fixture():
    events=[];calls=[]
    m=NS(traffic_hold=True,dynamic_hold=False,goal_handle=NS(cancel_goal_async=lambda:calls.append('cancel')),
         goal_pending=False,fine=False,cancel_requested=False,pause_pending=False,yield_pending=False,
         precision_pending=False,recovery_switch_pending=False,paused_nav=False,emit=lambda *a:events.append(a))
    return m,calls,events

def test_fleet_wait_cancels_once_and_retains_destination():
    m,c,e=fixture();m.target=(1.,2.,0.);m.request_id='same'
    Mission._apply_hold(m);Mission._apply_hold(m)
    assert c==['cancel'] and m.pause_pending and m.hold
    assert m.target==(1.,2.,0.) and m.request_id=='same'

def test_dynamic_observation_keeps_action_and_pending_switch_is_not_overridden():
    m,c,e=fixture();m.traffic_hold=False;m.dynamic_hold=True
    Mission._apply_hold(m);assert not c
    m.traffic_hold=True;m.yield_pending=True
    Mission._apply_hold(m);assert not c

def test_hold_after_goal_acceptance_is_not_lost():
    m,c,e=fixture();m.goal_pending=True
    Mission._apply_hold(m);assert not c
    m.goal_pending=False;Mission._apply_hold(m);assert c==['cancel']
