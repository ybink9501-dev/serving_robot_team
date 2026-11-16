from pathlib import Path
from cafe_autonomy.waypoints import read_waypoints,available_approach
from cafe_autonomy.policy import FleetCore

def test_other_robot_home_near_table04_south_selects_north_instead():
    points=read_waypoints(Path(__file__).resolve().parents[2]/'config/integration/waypoints.yaml')
    assert available_approach(points,'table_04','auto',(0.,0.),[(5.6,0.)],.9)=='top'
    assert available_approach(points,'table_04','bottom',(0.,0.),[(5.6,0.)],.9) is None

def test_unavailable_explicit_approach_stays_queued_without_active_goal():
    core=FleetCore();core.submit('order','table_04','robot_01',side='bottom')
    assert core.dispatch_next({'robot_01'},acceptable=lambda _:False) is None
    assert core.queue==['order'] and not core.active

def test_dispatch_preserves_assignment_independently_of_original_request_side():
    core=FleetCore();task,_=core.submit('order','table_04','robot_01')
    task.assigned_side='top'
    again,new=core.submit('order','table_04','robot_01')
    assert not new and again.assigned_side=='top' and again.side=='auto'
