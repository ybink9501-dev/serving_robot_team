#!/usr/bin/env python3
"""Robot 01 stationary triage. READ-ONLY: never sends ROS goals, velocity, cancel or resets."""
import json
import math
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


def get_gz_pose():
    try:
        p = subprocess.run(['gz','model','-m','robot_01','-p'],capture_output=True,text=True,timeout=7)
        return {'returncode':p.returncode,'output':(p.stdout or p.stderr)[-5000:].strip()}
    except (FileNotFoundError,subprocess.TimeoutExpired) as e:
        return {'error':str(e)}


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument('--seconds',type=int,default=12)
    ap.add_argument('--out',default='')
    args=ap.parse_args()
    if not (5<=args.seconds<=60):ap.error('seconds must be 5..60')
    if os.getenv('ROS_DOMAIN_ID')!='99' or os.getenv('GZ_PARTITION')!='team_robot_lab_gazebo_99':
        raise SystemExit('STOP: expected LAB ROS_DOMAIN_ID=99, GZ_PARTITION=team_robot_lab_gazebo_99')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatusArray

    gazebo_before=get_gz_pose()
    rclpy.init()
    n=Node('robot01_readonly_motion_diagnostic_v12')
    sensor=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    times=[];odom=[];velocities={'nav':[],'safe':[]};statuses=[];nav_actions=[]
    def on_clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if not times or t!=times[-1]:times.append(t)
    def on_odom(m):
        pos=m.pose.pose.position
        odom.append((time.monotonic(),float(pos.x),float(pos.y),float(m.twist.twist.linear.x),float(m.twist.twist.angular.z)))
    def on_vel(which,m):
        velocities[which].append((time.monotonic(),float(m.linear.x),float(m.angular.z)))
    def on_status(m):
        try:v=json.loads(m.data)
        except ValueError:v={'raw':m.data}
        statuses.append(v)
    def on_action(m):
        nav_actions.append([(x.goal_info.goal_id.uuid.hex() if isinstance(x.goal_info.goal_id.uuid, bytes) else list(x.goal_info.goal_id.uuid)[:3], x.status) for x in m.status_list])
    n.create_subscription(Clock,'/clock',on_clock,sensor)
    n.create_subscription(Odometry,'/robot_01/odom',on_odom,sensor)
    n.create_subscription(Twist,'/robot_01/cmd_vel_nav',lambda m:on_vel('nav',m),10)
    n.create_subscription(Twist,'/robot_01/cmd_vel',lambda m:on_vel('safe',m),10)
    n.create_subscription(String,'/robot_01/serving/status',on_status,10)
    n.create_subscription(GoalStatusArray,'/robot_01/navigate_to_pose/_action/status',on_action,10)
    deadline=time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=0.25)
        topic_graph={}
        for t in ['/clock','/robot_01/odom','/robot_01/cmd_vel_nav','/robot_01/cmd_vel','/robot_01/serving/status','/robot_01/navigate_to_pose/_action/status']:
            topic_graph[t]={'publishers':[str(i.node_namespace).rstrip('/')+'/'+i.node_name for i in n.get_publishers_info_by_topic(t)]}
    finally:
        n.destroy_node();rclpy.shutdown()
    gazebo_after=get_gz_pose()
    path=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(odom,odom[1:]))
    net=math.hypot(odom[-1][1]-odom[0][1],odom[-1][2]-odom[0][2]) if len(odom)>1 else None
    peaks={k:{'count':len(v),'peak_linear':round(max((abs(q[1]) for q in v),default=0),4),'peak_angular':round(max((abs(q[2]) for q in v),default=0),4)} for k,v in velocities.items()}
    st=statuses[-1] if statuses else None
    clock_delta=(times[-1]-times[0]) if len(times)>1 else None
    if clock_delta is not None and clock_delta<0.2:
        verdict='SIM_CLOCK_NOT_PROGRESSING_CHECK_GAZEBO_PAUSE'
    elif st and st.get('state') in ('IDLE','CANCELED','ERROR','COMPLETED') and peaks['nav']['peak_linear']<0.001 and peaks['nav']['peak_angular']<0.001:
        verdict='NO_ACTIVE_DRIVE_COMMAND_EXPECTED_STATIC_VIEW'
    elif st and st.get('state') in ('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP'):
        if len(odom)<2:verdict='ACTIVE_MISSION_NO_ODOM_CHECK_SENSOR_BRIDGE'
        elif (net is not None and net<0.03) and peaks['nav']['peak_linear']>0.03:verdict='ACTIVE_NAV_COMMAND_BUT_NO_PROGRESS_CHECK_COLLISION_SAFETY_PHYSICS'
        elif (net is not None and net<0.03) and peaks['nav']['peak_linear']<0.001:verdict='ACTIVE_MISSION_NO_NAV_VELOCITY_CHECK_NAV2'
        else:verdict='ACTIVE_MISSION_MOVEMENT_OBSERVED'
    elif not st:
        verdict='NO_SERVING_STATUS_CHECK_V3_SUPERVISOR'
    else:
        verdict='INSUFFICIENT_TO_CONFIRM_STUCK'
    report={
        'timestamp_utc':datetime.now(timezone.utc).isoformat(),'duration_s':args.seconds,
        'verdict':verdict,'gazebo_pose_before':gazebo_before,'gazebo_pose_after':gazebo_after,
        'clock_messages':len(times),'clock_progress_s':round(clock_delta,3) if clock_delta is not None else None,
        'odom_messages':len(odom),'odom_path_m':round(path,4),'odom_net_m':round(net,4) if net is not None else None,
        'last_odom':{'x':odom[-1][1],'y':odom[-1][2]} if odom else None,
        'serving_status':st,'serving_status_count':len(statuses),
        'cmd_vel':peaks,'nav_action_status_recent':nav_actions[-1] if nav_actions else None,
        'topic_publishers':topic_graph,
        'note':'Read-only. Zero velocity while CANCELED is expected; path planning is not driving. Verify actual Gazebo clearance and Safety before re-issuing START.'}
    out=Path(args.out).expanduser() if args.out else Path('/tmp')/('robot01_still_v12_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
    if out.exists():raise SystemExit('STOP existing report '+str(out))
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print('REPORT_FILE:',out)

if __name__=='__main__':main()
