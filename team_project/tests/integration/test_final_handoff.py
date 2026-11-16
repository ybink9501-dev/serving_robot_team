from types import SimpleNamespace
import math
import pytest
from cafe_autonomy.mission import Mission

class EndTick(Exception):pass

def run(pending=None):
    calls=[]
    def stop():raise EndTick()
    m=SimpleNamespace(dynamic_tick=lambda:None,recovery_tick=lambda:None,
        robot_pose=lambda:(.32,0.,math.pi/2),state='RETURNING_HOME',hold=False,
        pose_error=lambda:(.32,-math.pi/2,(.32,0.,math.pi/2)),target=(0.,0.,0.),
        xy_tolerance=.1,fine_engage_max=.4,rotation_status={'continuous_rad':3.0},
        limits={'navigation':{'continuous_rotation_rad':math.pi}},
        goal_handle=SimpleNamespace(cancel_goal_async=lambda:calls.append('cancel')),
        precision_pending=False,recovery_switch_pending=False,pause_pending=False,yield_pending=False,
        get_clock=stop)
    if pending:setattr(m,pending,True)
    with pytest.raises(EndTick):Mission.tick(m)
    return m,calls

def test_home_near_goal_hands_off_instead_of_switching_to_another_loop():
    m,calls=run()
    assert m.precision_pending and not m.recovery_switch_pending
    assert calls==['cancel']

@pytest.mark.parametrize('pending',['precision_pending','recovery_switch_pending','pause_pending','yield_pending'])
def test_handoff_does_not_race_existing_transitions(pending):
    _,calls=run(pending)
    assert calls==[]
