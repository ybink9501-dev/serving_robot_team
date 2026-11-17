import copy,math
from types import SimpleNamespace
from unittest.mock import Mock,patch
from concurrent.futures import Future
import numpy as np
import pytest
from serving_robot_mapping.mapping_edge_observation import edge_ray,select_edge_observations,matches_probe
from serving_robot_mapping.mapping_observation_sweep import ObservationSweep
from serving_robot_mapping.mapping_state_machine import MappingState,MappingStateMachine
from serving_robot_mapping.exploration_goal_manager import ExplorationGoalManager
from serving_robot_mapping.mapping_edge_observation import can_observe_in_place
from serving_robot_mapping.mapping_observation_memory import restore_observation_memory
import json
import hashlib


def test_only_thin_occupied_edge_is_an_observation_proposal_not_a_map_edit():
    w=h=12;a=[0]*(w*h);a[6*w+8]=-1;a[6*w+6]=100;before=a.copy()
    assert edge_ray(a,w,h,2.5,6.5,8,6,8.5,6.5,.05)==pytest.approx(.05)
    a[6*w+5]=100
    assert edge_ray(a,w,h,2.5,6.5,8,6,8.5,6.5,.05) is None
    a[6*w+5]=-1
    assert edge_ray(a,w,h,2.5,6.5,8,6,8.5,6.5,.05) is None
    assert before[6*w+6]==100


def test_probe_respects_goal_guard_and_bounded_observation_memory():
    a=np.zeros((30,30),dtype=int);a[[0,-1],:]=100;a[:,[0,-1]]=100;a[15,15]=-1;a[15,14]=100
    grid=(tuple(a.ravel().tolist()),30,30,.1,0.,0.,0.)
    def guard(x,y):return (math.hypot(x-.75,y-1.55)<.001,'TEST')
    points,_=select_edge_observations(grid,(.75,.75),guard,12.)
    assert points and points[0]['proposal_only']
    assert tuple(points[0]['xy'])==(.75,1.55)
    assert not select_edge_observations(grid,(.75,.75),lambda *a:(False,'BLOCKED'),12.)[0]
    assert not select_edge_observations(grid,(.75,.75),guard,12.,[(.75,1.55,3)])[0]
    assert grid[0][15*30+14]==100


def test_observing_state_cannot_become_map_complete_from_zero_frontiers():
    sm=MappingStateMachine();sm.transition_to(MappingState.OBSERVING);manager=ExplorationGoalManager(sm)
    for t in range(100):assert manager.observe_frontiers(0,float(t))==MappingState.OBSERVING


def test_probe_match_is_explicit_and_does_not_release_other_visited_goals():
    node=SimpleNamespace(_edge_probe_goals=[{'xy':[1.,2.]}])
    assert matches_probe(node,1.,2.)
    assert not matches_probe(node,1.1,2.)


def test_new_vantage_ten_centimeters_away_requires_actual_navigation():
    assert not can_observe_in_place((1.,2.),(1.1,2.),[])
    assert can_observe_in_place((1.,2.),(1.01,2.),[])
    assert not can_observe_in_place((1.,2.),(1.01,2.),[(1.,2.,3)])


def test_only_confirmed_successful_sweeps_restore_after_map_resume(tmp_path):
    events=[{'status':'SUCCEEDED'}]*3
    records=[dict(success=True,xy=[1.,2.],events=events),dict(success=True,xy=[1.01,2.],events=events),dict(success=False,xy=[5.,6.],events=[])]
    (tmp_path/'mapping_observations.jsonl').write_text('\n'.join(json.dumps(r) for r in records))
    assert restore_observation_memory(tmp_path/'report.json')==[(1.,2.,2)]


def test_resume_chain_keeps_previous_observation_budget_and_rejects_changed_source(tmp_path):
    first=tmp_path/'first';second=tmp_path/'second';first.mkdir();second.mkdir()
    report=first/'report.json';report.write_text('{}')
    events=[{'status':'SUCCEEDED'}]*3
    record=dict(success=True,xy=[1.,2.],events=events)
    (first/'mapping_observations.jsonl').write_text(json.dumps(record))
    (second/'mapping_observations.jsonl').write_text(json.dumps(record))
    (second/'mapping_observation_resume.json').write_text(json.dumps(dict(source_report=str(report),source_sha256=hashlib.sha256(report.read_bytes()).hexdigest())))
    assert restore_observation_memory(second/'report.json')==[(1.,2.,2)]
    report.write_text('{"changed":true}')
    with pytest.raises(ValueError,match='changed'):restore_observation_memory(second/'report.json')


def sweep_fixture():
    node=Mock();node.mapping_navigation_safety_check.return_value=True
    node.navigation_start_scan=SimpleNamespace(ranges=[12.]*720,range_min=.1,range_max=12.)
    node.navigation_start_scan_wall=10.
    done=Mock()
    with patch('rclpy.action.ActionClient') as factory:
        sweep=ObservationSweep(node,done)
    sweep.client.server_is_ready.return_value=True
    sweep.state=dict(linear=0.,angular=0.,wall=10.)
    return sweep,node,done


def test_spin_waits_for_physical_settle_and_never_sends_when_safety_is_false():
    sweep,node,done=sweep_fixture()
    with patch('serving_robot_mapping.mapping_observation_sweep.time.monotonic',return_value=10.):sweep.start();sweep.tick()
    sweep.client.send_goal_async.assert_not_called()
    node.mapping_navigation_safety_check.return_value=False
    with patch('serving_robot_mapping.mapping_observation_sweep.time.monotonic',return_value=10.6):sweep.tick()
    sweep.client.send_goal_async.assert_not_called()


def test_running_spin_is_canceled_and_unconfirmed_cancel_is_not_hidden():
    sweep,node,done=sweep_fixture()
    with patch('serving_robot_mapping.mapping_observation_sweep.time.monotonic',return_value=10.):sweep.start()
    sweep.phase='SPIN';sweep.deadline=30.
    sweep.result=Future();sweep.handle=Mock();sweep.cancel_ack=None
    node.mapping_navigation_safety_check.return_value=False
    with patch('serving_robot_mapping.mapping_observation_sweep.time.monotonic',return_value=10.):sweep.tick()
    assert sweep.phase=='CANCEL';sweep.handle.cancel_goal_async.assert_called_once()
    assert sweep.reason=='OBSERVATION_SAFETY_NOT_CLEAR: LIVE_NAVIGATION_SAFETY'
    sweep.cancel_ack=Future()
    with patch('serving_robot_mapping.mapping_observation_sweep.time.monotonic',return_value=19.):sweep.tick()
    assert sweep.busy and sweep.phase=='UNCERTAIN'
    assert done.call_args.args[1]=='OBSERVATION_CANCEL_UNCONFIRMED'


@pytest.mark.parametrize('reason', [
    'OBSERVATION_SAFETY_NOT_CLEAR: LIVE_NAVIGATION_SAFETY',
    'OBSERVATION_READY_TIMEOUT',
])
def test_safe_observation_gate_failure_returns_to_candidate_selection(reason):
    from serving_robot_mapping.exploration_manager import ExplorationManager
    from serving_robot_mapping.mapping_state_machine import MappingState
    transform=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0,y=2.0)))
    manager=SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=Mock(return_value=transform)),
        _observation_pose=(1.0,2.0),_inspection_memory=(),
        _observation_proposal=None,
        _observation_unknown_before=0,
        frontier_clearance_guard=SimpleNamespace(grid=((0,-1),)),
        mapping_evidence_directory='',_edge_probe_goals=[],
        _visibility_fallback_cache=(),
        _visibility_fallback_last_scanned_context=None,
        _visibility_fallback_retry_after=0.0,
        state_machine=SimpleNamespace(transition_to=Mock()),
        request_navigation_replan=Mock(),get_logger=Mock(return_value=Mock()),
    )

    ExplorationManager.observation_finished(
        manager, False, reason, [],
    )

    manager.state_machine.transition_to.assert_called_once_with(
        MappingState.EXPLORING
    )
    manager.request_navigation_replan.assert_called_once_with()
    assert manager._inspection_memory==[(1.0,2.0,1)]


def test_arrival_offset_cannot_reset_requested_viewpoint_budget(tmp_path):
    from serving_robot_mapping.mapping_observation_memory import record_observation
    from serving_robot_mapping.mapping_edge_observation import attempts_at
    # Actual Gazebo failure: arrival inside one 10cm cluster, requested
    # position just outside it. Old merging forever reported zero attempts.
    actual=(5.055,4.506);proposal={'xy':[5.059,4.553]}
    memory=[(4.969,4.502,1)]
    for _ in range(3):memory=record_observation(memory,actual,proposal)
    assert attempts_at(*actual,memory)>=3
    assert attempts_at(*proposal['xy'],memory)>=3
    record=dict(success=True,xy=list(actual),proposal=proposal,events=[{'status':'SUCCEEDED'}]*3)
    (tmp_path/'mapping_observations.jsonl').write_text('\n'.join([json.dumps(record)]*3))
    restored=restore_observation_memory(tmp_path/'report.json')
    assert attempts_at(*proposal['xy'],restored)>=3


def test_observation_requires_actual_path_endpoint_not_five_centimeter_substitute():
    from serving_robot_mapping.mapping_edge_observation import observation_endpoint_matches
    goal=(5.265415558405114,4.552711317606024)
    actual_end=(5.265415562875463,4.502711321331314)
    assert not observation_endpoint_matches(goal,[(5.19,4.53),actual_end])
    assert observation_endpoint_matches(goal,[(5.19,4.53),(goal[0],goal[1]-.009)])
    assert not observation_endpoint_matches(goal,[])


def test_larger_angular_view_beats_staying_near_far_observer():
    a=np.zeros((30,30),dtype=int);a[[0,-1],:]=100;a[:,[0,-1]]=100;a[15,15]=-1
    grid=(tuple(a.ravel().tolist()),30,30,.1,0.,0.,0.)
    far=(.75,.75);near=(.75,1.55)
    def guard(x,y):return (min(math.dist((x,y),far),math.dist((x,y),near))<.001,'TEST')
    points,_=select_edge_observations(grid,far,guard,12.)
    assert tuple(points[0]['xy'])==near
