"""Bounded initial scan sweep through the existing Nav2 behavior server."""
import math
import time
from .mapping_frontier_clearance import FrontierClearanceGuard


def bootstrap_plan(angle):
    angle=float(angle)
    if not math.isfinite(angle) or not .14<=angle<=.25:
        raise ValueError('Bootstrap requires a small .14..25 rad half sweep')
    return (angle,-2*angle,angle)


def bootstrap_scan_check(state, now):
    scan=state.get('scan')
    if scan is None or now-state.get('scan_wall',-100.)>.3:
        return False,'BOOTSTRAP_SCAN_STALE'
    return FrontierClearanceGuard().start_clear(scan.ranges,scan.range_min,scan.range_max,now-state['scan_wall'])


def run_bootstrap(node, state, should_stop, angle):
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import Spin
    from action_msgs.msg import GoalStatus
    plan=bootstrap_plan(angle)
    client=ActionClient(node,Spin,'/mapping_scan_spin')
    if not client.wait_for_server(timeout_sec=8.):
        client.destroy();raise RuntimeError('Mapping-only scan behavior unavailable')
    events=[]
    try:
        for yaw in plan:
            # Server discovery is blocking: explicitly receive a fresh scan
            # before the first command, keeping the same freshness threshold.
            scan_deadline=time.monotonic()+3.
            safe,reason=bootstrap_scan_check(state,time.monotonic())
            while not safe and reason=='BOOTSTRAP_SCAN_STALE' and time.monotonic()<scan_deadline and not should_stop():
                rclpy.spin_once(node,timeout_sec=.05)
                safe,reason=bootstrap_scan_check(state,time.monotonic())
            if should_stop() or not safe:raise RuntimeError('Bootstrap blocked: '+reason)
            before=float(state.get('yaw',0.))
            goal=Spin.Goal();goal.target_yaw=yaw;goal.time_allowance.sec=15
            # Jazzy Spin always uses behavior-server collision checks; no bypass flag.
            pending=client.send_goal_async(goal);deadline=time.monotonic()+8.
            while not pending.done() and time.monotonic()<deadline and not should_stop():
                rclpy.spin_once(node,timeout_sec=.05)
            if not pending.done():raise RuntimeError('Bootstrap goal acknowledgement unconfirmed; stop owned runtime')
            handle=pending.result()
            if not handle.accepted:raise RuntimeError('Bootstrap spin rejected')
            result=handle.get_result_async();deadline=time.monotonic()+18.;failure=None
            while not result.done():
                rclpy.spin_once(node,timeout_sec=.05)
                safe,reason=bootstrap_scan_check(state,time.monotonic())
                if should_stop() or not safe or time.monotonic()>deadline:
                    failure=reason if not safe else 'BOOTSTRAP_STOP_OR_TIMEOUT'
                    handle.cancel_goal_async()
                    cancel_deadline=time.monotonic()+8.
                    while not result.done() and time.monotonic()<cancel_deadline:rclpy.spin_once(node,timeout_sec=.05)
                    if not result.done():raise RuntimeError('Bootstrap cancel unconfirmed; stop owned runtime')
                    break
            if failure:raise RuntimeError('Bootstrap cancelled: '+failure)
            reply=result.result()
            if reply.status!=GoalStatus.STATUS_SUCCEEDED or reply.result.error_code!=0:
                raise RuntimeError('Bootstrap failed: '+str(reply.result.error_code))
            settle=time.monotonic()+.3
            while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.05)
            actual=math.atan2(math.sin(state['yaw']-before),math.cos(state['yaw']-before))
            if abs(actual)<.10:raise RuntimeError('Bootstrap did not exceed SLAM heading gate')
            events.append({'requested_yaw_rad':yaw,'actual_yaw_rad':actual,'result':'SUCCEEDED'})
        settle=time.monotonic()+3.
        while time.monotonic()<settle and not should_stop():rclpy.spin_once(node,timeout_sec=.05)
        if should_stop():raise RuntimeError('Mapping owner stopped after bootstrap')
        return events
    finally:
        client.destroy()
