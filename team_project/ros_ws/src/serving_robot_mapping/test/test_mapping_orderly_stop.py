from types import SimpleNamespace
from unittest.mock import Mock,patch
from pathlib import Path
import runpy
from serving_robot_mapping.exploration_manager import _r16_203_settle_navigation,ExplorationManager
from serving_robot_mapping.mapping_state_machine import MappingState,MappingStateMachine
from serving_robot_mapping.exploration_goal_manager import ExplorationGoalManager,VerifiedFrontierGoal
import serving_robot_mapping.frontier_navigation_client as nav


def test_shutdown_requires_actual_terminal_result_and_cancel_ack():
    for ack,expected in [('ACCEPTED','CANCEL_CONFIRMED_TERMINAL'),('PENDING','CLIENT_IDLE_TERMINAL_STATUS_NOT_INDEPENDENTLY_VERIFIED')]:
        client=SimpleNamespace(busy=True,last_terminal_status=5,last_cancel_ack=ack)
        client.cancel=lambda reason:setattr(client,'busy',False)
        node=SimpleNamespace(navigation_client=client,stop_robot=Mock())
        assert _r16_203_settle_navigation(node,lambda *a,**k:None,lambda:True,lambda:0.)==expected


def test_confirmed_session_stop_does_not_blacklist_a_valid_route_or_complete_map():
    state=MappingStateMachine();state.transition_to(MappingState.EXPLORING)
    manager=ExplorationGoalManager(state_machine=state)
    goal=VerifiedFrontierGoal(1.,1.,1.);manager.begin_navigation(goal)
    node=SimpleNamespace(state_machine=state,goal_manager=manager,get_logger=lambda:Mock())
    ExplorationManager.navigation_finished(node,goal,'CANCELED','Mapping Explorer shutting down; terminal status=5; cancel response=ACCEPTED')
    assert state.state==MappingState.IDLE and manager.active_goal is None
    assert manager.failed_goals==[]


def test_client_preserves_structured_terminal_evidence_before_clearing_goal():
    with patch.object(nav,'ActionClient'):
        client=nav.FrontierNavigationClient(Mock(),enabled=True,safety_check=lambda:True)
    client.phase='MOVING';client.token=1;client.goal=VerifiedFrontierGoal(1.,1.,1.)
    client.cancel_reason='Mapping Explorer shutting down';client.cancel_ack='ACCEPTED'
    future=Mock();future.result.return_value.status=5
    client._result(1,future)
    assert not client.busy and client.last_terminal_status==5 and client.last_cancel_ack=='ACCEPTED'


def test_time_limited_run_remains_unfinished_but_reports_verified_cancellation(tmp_path):
    source=Path(__file__).resolve().parents[1]/'tools/run_lidar_mapping_session.py'
    functions=runpy.run_path(str(source));log=tmp_path/'explorer.log'
    log.write_text('R16-203 NAVIGATION_SHUTDOWN_STATUS= CANCEL_CONFIRMED_TERMINAL\n')
    outcome,evidence=functions['reconcile_explorer_shutdown']('TIME_LIMIT_REACHED',log)
    assert outcome=='TIME_LIMIT_REACHED' and evidence['nav2_action_result_independently_verified']
    assert not evidence['nav2_cancel_unconfirmed_observed']


def test_explicit_wall_return_failure_finishes_session_without_claiming_complete():
    source=Path(__file__).resolve().parents[1]/'tools/run_lidar_mapping_session.py'
    terminal=runpy.run_path(str(source))['terminal_event']
    for code in ('OUTER_WALL_RETURN_UNREACHABLE','OUTER_WALL_SEED_UNREACHABLE'):
        assert terminal('EXPLORATION_BLOCKED: '+code+'; MAP_COMPLETE not asserted')==code
    assert terminal('EXPLORATION_BLOCKED: VISITED_VIEWPOINTS_ONLY; no Nav2 goal dispatched')=='COVERAGE_LIMITED'
