from types import SimpleNamespace
from unittest.mock import Mock
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from serving_robot_mapping.exploration_manager import ExplorationManager
from serving_robot_mapping.mapping_state_machine import MappingState


def test_map_and_marker_before_frontier_count_waits_without_crashing_or_completing():
    state=SimpleNamespace(state=MappingState.EXPLORING,transition_to=Mock())
    node=SimpleNamespace(navigation_enabled=True,wall_first_enabled=False,
        frontier_clearance_guard=SimpleNamespace(grid=(tuple([0]*400),20,20,.1,0.,0.,0.),
            observation_candidates=lambda *a,**k:[],goal_clear=lambda *a:(False,'NOT_READY')),
        last_frontier_count=None,state_machine=state,planner_client=Mock(),
        tf_buffer=SimpleNamespace(lookup_transform=Mock(side_effect=RuntimeError('TF not received yet'))),
        get_logger=lambda:Mock())
    marker=Marker();marker.action=Marker.ADD;marker.type=Marker.SPHERE_LIST
    marker.header.frame_id='map';marker.points=[Point(x=1.,y=1.)]
    ExplorationManager.mapping_update_candidates(node,marker)
    state.transition_to.assert_not_called()
    forwarded=node.planner_client.update_candidates.call_args.args[0]
    assert not forwarded.points
