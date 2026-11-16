import math
import numpy as np
from serving_robot_mapping.mapping_perimeter import perimeter_routes,PerimeterPlan
from serving_robot_mapping.mapping_frontier_clearance import FrontierClearanceGuard


def scene(yaw=0.):
    a=np.zeros((120,160),dtype=int);a[[0,-1],:]=100;a[:,[0,-1]]=100
    a[50:65,70:85]=100
    grid=(a.ravel().tolist(),160,120,.05,0.,0.,yaw)
    return grid,(math.cos(yaw)-math.sin(yaw),math.sin(yaw)+math.cos(yaw))


def test_room_wall_first_then_fixed_obstacle_with_actual_clearance():
    grid,robot=scene();routes,metrics=perimeter_routes(grid,robot)
    guard=FrontierClearanceGuard();guard.update_map(*grid)
    assert routes[0]['kind']=='OUTER_WALL'
    assert any(r['kind']=='FIXED_OBSTACLE' for r in routes)
    for route in routes:
        assert route['points'][0]==route['points'][-1]
        assert all(guard.goal_clear(*p)[0] for p in route['points'])
    assert metrics['perimeter_radius_m']>.52


def test_rotated_map_preserves_wall_routes_and_no_map_pixels_are_changed():
    grid,robot=scene(.4);before=grid[0][:]
    routes,_=perimeter_routes(grid,robot)
    guard=FrontierClearanceGuard();guard.update_map(*grid)
    assert routes and all(guard.goal_clear(*p)[0] for r in routes for p in r['points'])
    assert grid[0]==before


def test_fixed_obstacle_near_wall_is_detoured_in_reachable_envelope():
    grid,robot=scene();a=np.array(grid[0]).reshape(120,160)
    a[90:117,4:45]=100  # Counter leaves a gap narrower than this robot envelope.
    grid=(a.ravel().tolist(),*grid[1:]);routes,_=perimeter_routes(grid,robot)
    assert routes[0]['kind']=='OUTER_WALL'
    guard=FrontierClearanceGuard();guard.update_map(*grid)
    assert all(guard.goal_clear(*p)[0] for p in routes[0]['points'])
    assert any(p[1]<4.3 and p[0]<2.5 for p in routes[0]['points'])


def test_blocked_candidate_is_retained_and_next_is_tried_without_false_completion():
    p=PerimeterPlan([dict(kind='OUTER_WALL',points=[(1.,1.),(2.,1.),(2.,2.),(1.,2.),(1.,1.)])])
    clear=lambda x,y:(True,'CLEAR')
    assert p.candidate(clear,{(1.,1.)},1)==[(2.,1.)]
    assert p.ledger[0]['status']=='DEFERRED'
    assert p.arrived(2.,1.,1)
    assert p.candidate(clear,set(),1)==[(2.,2.)]
    assert p.arrived(2.,2.,1)
    assert p.candidate(clear,set(),1)==[(1.,2.)]
    assert p.arrived(1.,2.,1)
    assert p.candidate(clear,set(),1)==[(2.,1.)]  # Return to the actually reached anchor.
    assert p.arrived(2.,1.,1)
    assert p.closed_loops==[0] and p.snapshot()['map_complete'] is False


def test_unreachable_wall_return_is_not_a_completed_loop_and_is_rechecked():
    p=PerimeterPlan([dict(kind='OUTER_WALL',points=[(1.,1.),(2.,1.),(1.,1.)])])
    clear=lambda x,y:(True,'CLEAR')
    for xy in [(1.,1.),(2.,1.)]:
        assert p.candidate(clear,set(),1)==[xy];assert p.arrived(*xy,1)
    assert p.candidate(clear,{(1.,1.)},1)==[]
    assert p.outer_return_failed and not p.closed_loops
    p.retry_deferred(2)
    assert p.candidate(clear,set(),2)==[(1.,1.)]


def test_out_and_back_is_not_a_closed_wall_circuit():
    p=PerimeterPlan([dict(kind='OUTER_WALL',points=[(1.,1.),(2.,1.),(1.,1.)])])
    for _ in range(3):
        xy=p.candidate(lambda x,y:(True,'CLEAR'),set(),1)[0]
        p.arrived(*xy,1)
    assert not p.closed_loops and p.outer_return_failed


def test_unknown_is_not_traversed_or_treated_as_observed_wall():
    grid,robot=scene();a=np.array(grid[0]).reshape(120,160);a[:,100:]=-1
    grid=(a.ravel().tolist(),*grid[1:]);routes,_=perimeter_routes(grid,robot)
    assert routes and all(p[0]<5. for r in routes for p in r['points'])
    assert -1 in grid[0]
