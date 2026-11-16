"""Handover completes the mission without issuing an unsolicited HOME goal."""
import json
from types import SimpleNamespace
from std_msgs.msg import String
from cafe_autonomy.mission import Mission


def mission(mode):
    calls = []
    m = SimpleNamespace(request_id='order', confirmed_signals={}, mode=mode,
                        state='WAIT_HANDOVER', robot='robot_01', xy_tolerance=.1,
                        yaw_tolerance=.25, table='table_03',
                        pose_error=lambda: (.02,.02,(1.,1.,0.)),
                        emit=lambda *args, **kwargs: None)
    def start(name, state):
        calls.append((name,state))
        m.state=state
    m.start_goal=start
    return m, calls


def confirm(m):
    Mission.confirm(m, String(data=json.dumps({'command':'COMPLETE','request_id':'order'})))


def test_full_serving_completes_without_automatic_home():
    m, calls = mission('KITCHEN_PICKUP')
    confirm(m)
    assert m.state == 'COMPLETED'
    assert calls == []
    confirm(m)
    assert calls == []


def test_direct_table_command_completes_at_table():
    m, calls = mission('DELIVER_DIRECT')
    confirm(m)
    assert m.state == 'COMPLETED'
    assert calls == []
