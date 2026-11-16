from types import SimpleNamespace as NS
import cafe_autonomy.safety as module
from cafe_autonomy.safety import Safety

def fixture(monkeypatch):
    clocks={'wall':10.,'ros':.999}
    monkeypatch.setattr(module.time,'monotonic',lambda:clocks['wall'])
    m=NS(robot='robot_02',safe_cfg={'scan_timeout_s':.5},scan_stamp=0.,pending_scan=None,scan_input_age=None,future_scan_count=0,accepted_scan_count=0,
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clocks['ros']*1e9))))
    msg=NS(header=NS(frame_id='robot_02/laser_link',stamp=NS(sec=1,nanosec=0)),ranges=[1.],range_min=.1,range_max=12.)
    return m,msg,clocks

def test_future_scan_waits_for_clock_without_weakening_freshness(monkeypatch):
    m,msg,c=fixture(monkeypatch);Safety.on_scan(m,msg)
    assert m.scan_stamp==0. and m.future_scan_count==1
    c.update(ros=1.01,wall=10.02);Safety.validate_pending_scan(m)
    assert m.scan_stamp==10. and m.accepted_scan_count==1
    c['wall']=10.3;Safety.validate_pending_scan(m);assert m.scan_stamp==10.

def test_late_clock_and_invalid_scan_never_refresh_gate(monkeypatch):
    m,msg,c=fixture(monkeypatch);Safety.on_scan(m,msg)
    c.update(ros=1.1,wall=10.6);Safety.validate_pending_scan(m);assert m.scan_stamp==0.
    c.update(ros=1.7,wall=11.);Safety.on_scan(m,msg);assert m.scan_stamp==0.
    msg.header.frame_id='other';c['ros']=1.1;Safety.on_scan(m,msg);assert m.scan_stamp==0.
