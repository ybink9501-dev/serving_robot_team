#!/usr/bin/env python3
"""Read-only first-drive gate. Without --live, tests snapshots without ROS.
With --live, inspects the already running ROS graph. It NEVER sends any goal.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from checks import check,expand,read_yaml,HERE


def check_live(c,report):
    if os.environ.get('ROS_DOMAIN_ID')!='99':
        report['errors'].append({'name':'ROS environment','error':'ROS_DOMAIN_ID must be 99'});return
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as e:
        report['errors'].append({'name':'ROS modules','error':str(e)});return
    rclpy.init();node=Node('robot01_pretest_v6_readonly');statuses=[];odoms=[];goal_states=[]
    try:
        node.create_subscription(String,'/robot_01/serving/status',lambda m:statuses.append(m.data),10)
        node.create_subscription(Odometry,'/robot_01/odom',lambda m:odoms.append(m),10)
        status_qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(GoalStatusArray,'/robot_01/navigate_to_pose/_action/status',
                                 lambda m:goal_states.append(m),status_qos)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and (not statuses or not odoms):
            rclpy.spin_once(node,timeout_sec=.15)
        if not statuses:
            report['errors'].append({'name':'V3 heartbeat','error':'No serving/status within 8s'})
        else:
            status=json.loads(statuses[-1]);allowed=('IDLE','COMPLETED','CANCELED')
            if status.get('robot_id')!='robot_01' or status.get('state') not in allowed or status.get('map_valid') is not True:
                report['errors'].append({'name':'V3 heartbeat','error':f'Robot must be idle and map_valid: {status}'})
            else:report['checks'].append({'name':'Live V3 idle and map_valid','result':'PASS','details':status})
        if not odoms:
            report['errors'].append({'name':'Odom','error':'No odometry within 8s'})
        else:
            twist=odoms[-1].twist.twist
            v=float(twist.linear.x);w=float(twist.angular.z)
            if abs(v)>.01 or abs(w)>.02:
                report['errors'].append({'name':'Odom','error':f'Robot is moving: vx={v}, wz={w}'})
            else: report['checks'].append({'name':'Robot stationary','result':'PASS','details':{'vx':v,'wz':w}})
        if not goal_states:
            report['errors'].append({'name':'Nav2 active goals','error':'No action status received; cannot establish that no navigation goal is active'})
        else:
            active=[s.status for s in goal_states[-1].status_list if s.status in (1,2,3)]
            if active:report['errors'].append({'name':'Nav2 active goals','error':f'Active or canceling goals: {active}'})
            else:report['checks'].append({'name':'Nav2 has no active goal','result':'PASS'})
        subs=node.get_subscriptions_info_by_topic('/robot_01/destination_command')
        destsubs=[x for x in subs if 'destination_server' in x.node_name]
        if len(destsubs)!=1:
            report['errors'].append({'name':'Destination server','error':f'Expected one destination subscriber, saw {len(destsubs)}'})
        else: report['checks'].append({'name':'Single destination subscriber','result':'PASS'})
        publisher_info=node.get_publishers_info_by_topic('/robot_01/cmd_vel')
        names=[p.node_name for p in publisher_info]
        if len(names)!=1 or names[0]!='collision_monitor':
            report['errors'].append({'name':'Final velocity owner','error':f'Expected collision_monitor only: {names}'})
        else:report['checks'].append({'name':'Final cmd_vel has one safety owner','result':'PASS','details':names})
        # This rclpy.Node does not provide get_action_names_and_types().
        # Check readiness through Nav2 action clients; never send a goal.
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        for action, action_type in (
            ('/robot_01/navigate_to_pose', NavigateToPose),
            ('/robot_01/compute_path_to_pose', ComputePathToPose),
        ):
            client = ActionClient(node, action_type, action)
            try:
                if client.wait_for_server(timeout_sec=2.0):
                    report['checks'].append({'name': action, 'result': 'PASS'})
                else:
                    report['errors'].append({'name': 'Nav2 action',
                                             'error': f'Action server unavailable: {action}'})
            finally:
                client.destroy()
        # Destination must actually use V3. A valid V3 status alone does not guarantee that.
        try:
            proc=subprocess.run(['ros2','param','get','/destination_server','waypoints_file'],
                 capture_output=True,text=True,timeout=12,check=True)
            actual=proc.stdout.partition('String value is:')[2].strip()
            expected=str(expand(c['serving_v3_dir'])/'generated/active_waypoints.yaml')
            if actual!=expected:
                report['errors'].append({'name':'Active destination waypoint','error':f'Actual {actual!r}; expected {expected!r}'})
            else:report['checks'].append({'name':'Destination server uses V3 waypoints','result':'PASS','details':actual})
        except Exception as e:
            report['errors'].append({'name':'Active destination waypoint','error':str(e)})
        src=node.get_publishers_info_by_topic('/robot_01/person_safety_event_lab')
        dst=node.get_subscriptions_info_by_topic('/robot_01/person_safety_event_lab')
        if not src or not dst:
            report['warnings'].append('Person event direct link not fully visible: check before human-avoidance trial (not a blocker for empty café)')
        else:report['checks'].append({'name':'Person event publisher/subscriber present','result':'PASS'})
        speedsubs=node.get_subscriptions_info_by_topic('/robot_01/speed_limit')
        if not speedsubs:
            report['warnings'].append('Nav2 speed_limit topic subscriber not visible; check before runtime profile changes')
        else:report['checks'].append({'name':'Nav2 speed_limit subscriber present','result':'PASS'})
    except Exception as e:
        report['errors'].append({'name':'Live preflight runtime','error':str(e)})
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--live',action='store_true')
    parser.add_argument('--config',default=str(HERE/'config.yaml'))
    parser.add_argument('--out',help='Optional local JSON evidence file; no shared project files modified')
    opts=parser.parse_args()
    report=check(opts.config)
    if opts.live and report['ready']:
        check_live(read_yaml(expand(opts.config)),report)
        report['ready']=not report['errors']
    report['phase']='LIVE_PRETEST_NO_GOAL' if opts.live else 'STATIC_OFFLINE'
    result=json.dumps(report,ensure_ascii=False,indent=2)
    print(result)
    if opts.out:Path(opts.out).expanduser().write_text(result+'\n',encoding='utf8')
    return 0 if report['ready'] else 2
if __name__=='__main__':raise SystemExit(main())
