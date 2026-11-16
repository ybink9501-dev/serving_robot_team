import time
from types import SimpleNamespace
import pytest
from serving_robot_mapping.mapping_bootstrap import bootstrap_plan,bootstrap_scan_check


def test_scan_sweep_is_bounded_and_returns_heading():
    plan=bootstrap_plan(.18)
    assert abs(sum(plan))<1e-12
    assert max(map(abs,plan))<=.5
    assert sum(map(abs,plan))<1.


@pytest.mark.parametrize('angle',[0.,.05,1.,float('nan')])
def test_unbounded_or_unproductive_scan_not_allowed(angle):
    with pytest.raises(ValueError):bootstrap_plan(angle)


def test_bootstrap_requires_live_raw_clearance():
    now=time.monotonic();scan=SimpleNamespace(ranges=[.7]*720,range_min=.1,range_max=12.)
    state={'scan':scan,'scan_wall':now}
    assert bootstrap_scan_check(state,now)[0]
    scan.ranges[0]=.49
    assert not bootstrap_scan_check(state,now)[0]
    scan.ranges[0]=.7
    assert not bootstrap_scan_check(state,now+.31)[0]
