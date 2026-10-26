#!/usr/bin/env python3
"""Separate ROS node: map-frame arrival telemetry + cancel on V6 no-progress spin.
Does not change controller settings, goal checker, destination server, or cmd_vel.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import yaml

from policy import SpinWindow

HOME=Path.home()
V6=HOME/'team_robot_lab/robot01_pretest_v6/config.yaml'
WPS=HOME/'team_robot_lab/serving_v3/generated/active_waypoints.yaml'
ACTIVE=('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP')


def yaw_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def angle_error(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--enforce',action='store_true',help='Cancel serving on verified excessive no-progress turning')
    ap.add_argument('--out',default='/tmp/robot01_spin_arrival_v7.jsonl')
    args=ap.parse_args()
    if not V6.is_file() or not WPS.is_file():raise SystemExit('Missing V6 config or V3 waypoints; no ROS node started')
    c=yaml.safe_load(V6.read_text(encoding='utf-8'))
    m=c['monitor']
    wp=yaml.safe_load(WPS.read_text(encoding='utf-8'))
    targets={item['entity_id']:item['pose'] for item in wp['waypoints']}
    policy=SpinWindow(float(m['no_progress_window_s']),float(m['minimum_progress_m']),float(m['excessive_rotation_without_progress_rad']))
    out=Path(args.out)
    if out.exists():raise SystemExit(f'Output file already exists; choose --out: {out}')

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node=Node('robot01_spin_arrival_monitor_v7')
    buffer=Buffer();listener=TransformListener(buffer,node)
    fh=out.open('x',encoding='utf-8')
    state='IDLE';target_id=None;triggered=False;last_odom=None;last_report=0.;last_guard_log=0.
    pub=node.create_publisher(String,'/robot_01/serving/request',10)

    def emit(event,**data):
        payload={'timestamp_utc':datetime.now(timezone.utc).isoformat(),
                 'event':event,**data}
        fh.write(json.dumps(payload,ensure_ascii=False)+'\n');fh.flush()
        if event in ('STARTED','EXCESSIVE_SPIN','CANCEL_PUBLISHED','CANCEL_UNDELIVERABLE','STATE_TRANSITION'):
            print(json.dumps(payload,ensure_ascii=False),flush=True)

    def on_state(msg):
        nonlocal state,target_id,triggered
        try:o=json.loads(msg.data)
        except (TypeError,ValueError):return
        if o.get('robot_id')!='robot_01':return
        next_state=o.get('state',state)
        next_id=o.get('destination_id',target_id)
        if next_state!=state or next_id!=target_id:
            emit('STATE_TRANSITION',previous_state=state,state=next_state,destination_id=next_id,reason=o.get('reason'))
            if next_state not in ACTIVE or state not in ACTIVE or next_id!=target_id:
                policy.clear()
            if next_state not in ACTIVE:triggered=False
        state=next_state;target_id=next_id

    def on_odom(msg):
        nonlocal triggered,last_odom
        t=msg.twist.twist
        q=msg.pose.pose.orientation
        x=float(msg.pose.pose.position.x);y=float(msg.pose.pose.position.y)
        last_odom={'vx_mps':float(t.linear.x),'wz_radps':float(t.angular.z),
                   'odom_x':x,'odom_y':y,'odom_yaw':yaw_quat(q)}
        if state not in ACTIVE or triggered:
            policy.clear();return
        result=policy.observe(time.monotonic(),x,y,last_odom['odom_yaw'])
        if result['triggered']:
            triggered=True
            emit('EXCESSIVE_SPIN',state=state,destination_id=target_id,
                 rotation_deg=round(math.degrees(result['rotation_rad']),2),
                 window_s=result['window_s'],net_displacement_m=round(result['net_displacement_m'],4),
                 threshold_deg=round(math.degrees(policy.angular_limit_rad),2),
                 note='Odometry frame used ONLY for relative motion, not target distance')
            if args.enforce:
                if pub.get_subscription_count()>0:
                    pub.publish(String(data=json.dumps({'command':'CANCEL'})))
                    emit('CANCEL_PUBLISHED',reason='NO_PROGRESS_EXCESSIVE_ROTATION',
                         note='Async cancellation: watch the robot and pause Gazebo if it does not stop')
                else:
                    emit('CANCEL_UNDELIVERABLE',reason='No V3 supervisor subscriber')

    def telemetry():
        nonlocal last_report
        if state not in ACTIVE or not target_id or target_id not in targets:return
        if time.monotonic()-last_report<1.:return
        last_report=time.monotonic()
        try:
            tf=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        except Exception as exc:
            emit('MAP_TF_UNAVAILABLE',reason=str(exc)[:250]);return
        x=tf.transform.translation.x;y=tf.transform.translation.y
        ya=yaw_quat(tf.transform.rotation)
        goal=targets[target_id]
        emit('MAP_GOAL_ERROR',state=state,destination_id=target_id,
            map_xy_error_m=round(math.hypot(x-goal['x'],y-goal['y']),5),
            map_yaw_error_deg=round(math.degrees(angle_error(ya,goal['yaw'])),3),
            map_pose={'x':round(x,5),'y':round(y,5),'yaw':round(ya,5)},
            goal_pose=goal,odom_speed=last_odom)

    def on_nav(msg):
        try:o=json.loads(msg.data)
        except (TypeError,ValueError):return
        if o.get('robot_id')=='robot_01':emit('NAV_EVENT',payload=o)

    node.create_subscription(String,'/robot_01/serving/status',on_state,10)
    node.create_subscription(Odometry,'/robot_01/odom',on_odom,10)
    node.create_subscription(String,'/robot_01/navigation_event',on_nav,10)
    node.create_timer(.2,telemetry)
    emit('STARTED',mode=('ENFORCE_CANCEL' if args.enforce else 'OBSERVER'),
         threshold_window_s=policy.seconds,threshold_deg=math.degrees(policy.angular_limit_rad),
         min_progress_m=policy.min_progress_m,waypoint_count=len(targets),
         note='Does not issue GO goals or publish cmd_vel; requires an existing running V3 supervisor')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        emit('STOPPED')
        fh.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
