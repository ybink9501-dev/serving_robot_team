from types import SimpleNamespace
from unittest.mock import Mock,patch
import math
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from serving_robot_mapping.mapping_perimeter import PerimeterPlan
from serving_robot_mapping.mapping_frontier_clearance import FrontierClearanceGuard
from serving_robot_mapping.mapping_state_machine import MappingState,MappingStateMachine
from serving_robot_mapping.exploration_manager import ExplorationManager
from serving_robot_mapping.exploration_goal_manager import VerifiedFrontierGoal
import serving_robot_mapping.frontier_navigation_client as nav


def test_expanded_map_starts_new_outer_circuit_without_dropping_old_evidence():
    a=np.zeros((120,160),dtype=int);a[[0,-1],:]=100;a[:,[0,-1]]=100
    guard=FrontierClearanceGuard();guard.update_map(a.ravel().tolist(),160,120,.05,0,0,0)
    plan=PerimeterPlan([dict(kind='OUTER_WALL',points=[(1.,1.),(2.,1.),(2.,2.),(1.,1.)])])
    for _ in range(4):
        xy=plan.candidate(lambda x,y:(True,'CLEAR'),set(),1)[0];plan.arrived(*xy,1)
    state=MappingStateMachine();state.transition_to(MappingState.EXPLORING)
    tf=TransformStamped();tf.transform.translation.x=1.;tf.transform.translation.y=1.
    m=SimpleNamespace(wall_first_enabled=True,state_machine=state,frontier_clearance_guard=guard,
        perimeter_plan=plan,freshness_monitor=SimpleNamespace(map_revision=2),
        _perimeter_reference_area=1.5,_perimeter_history=[],
        tf_buffer=SimpleNamespace(lookup_transform=Mock(return_value=tf)),
        planner_client=SimpleNamespace(update_candidates=Mock()),write_perimeter_evidence=Mock(),get_logger=lambda:Mock())
    marker=Marker();marker.action=Marker.ADD;marker.type=Marker.POINTS;marker.header.frame_id='map'
    assert ExplorationManager.update_perimeter_candidates(m,marker)
    assert len(m._perimeter_history)==1 and m._perimeter_history[0]['closed_loops']==[0]
    assert m._perimeter_reference_area>1.5 and m.perimeter_plan is not plan
    forwarded=m.planner_client.update_candidates.call_args.args[0]
    assert len(forwarded.points)==1


def test_mapping_goal_uses_validated_tangent_without_changing_xy_path():
    node=Mock();node.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=10)
    with patch.object(nav,'ActionClient') as factory:
        factory.return_value.server_is_ready.return_value=True
        client=nav.FrontierNavigationClient(node,enabled=True,safety_check=lambda:True)
        goal=VerifiedFrontierGoal(x=2.,y=3.,path_length_m=1.,path_points=((1.,3.),(2.,3.)),yaw=math.pi/2)
        assert client.start(goal)
        pose=factory.return_value.send_goal_async.call_args.args[0].pose.pose
        assert (pose.position.x,pose.position.y)==(2.,3.)
        assert abs(pose.orientation.z-math.sin(math.pi/4))<1e-12
        assert abs(pose.orientation.w-math.cos(math.pi/4))<1e-12
