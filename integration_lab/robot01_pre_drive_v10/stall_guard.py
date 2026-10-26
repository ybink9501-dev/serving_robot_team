#!/usr/bin/env python3
"""Stage 3 - independent Robot 01 no-progress guard; never publishes cmd_vel or goals.
Use V7 in parallel for excessive spinning. This guard catches long stationary stalls.
"""
import argparse
import json
import os
import math
from pathlib import Path
import time
from datetime import datetime,timezone
import yaml
from policies import StallWindow

ROOT=Path(__file__).resolve().parent
ACTIVE={'GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP'}

def main():
    a=argparse.ArgumentParser()
    a.add_argument('--enforce',action='store_true',help='Send V3 cancellation ONLY after a verified stall')
    a.add_argument('--out',default='/tmp/robot01_stall_guard_v10.jsonl')
    args=a.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='99':raise SystemExit('ROS_DOMAIN_ID=99 required')
    c=yaml.safe_load((ROOT/'config.yaml').read_text())
    limits=c['stall'];policy=StallWindow(float(limits['window_s']),float(limits['minimum_travel_m']),
                                       float(limits['maximum_path_travel_m']))
    out=Path(args.out).expanduser()
    if out.exists():raise SystemExit('Output exists: choose unique --out')
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init()
    node=Node('robot01_stall_guard_v10')
    pub=node.create_publisher(String,'/robot_01/serving/request',10)
    state='IDLE';destination=None;map_valid=False;triggered=False
    odom_last=None;last_state_time=time.monotonic()
    cmds={}
    fh=out.open('x',encoding='utf-8')
    def emit(kind,**info):
        d={'timestamp_utc':datetime.now(timezone.utc).isoformat(),'event':kind,**info}
        fh.write(json.dumps(d,ensure_ascii=False)+'\n');fh.flush()
        if kind not in ('HEARTBEAT',):print(json.dumps(d,ensure_ascii=False),flush=True)
    def status_cb(msg):
        nonlocal state,destination,triggered,last_state_time,map_valid
        try:d=json.loads(msg.data)
        except ValueError:return
        if d.get('robot_id')!='robot_01':return
        nxt=d.get('state','');newdest=d.get('destination_id')
        map_valid=d.get('map_valid') is True
        if nxt!=state or newdest!=destination:
            emit('STATE_TRANSITION',previous=state,state=nxt,destination_id=newdest,reason=d.get('reason'))
            if nxt!=state or newdest!=destination:
                policy.clear();triggered=False;last_state_time=time.monotonic()
        state=nxt;destination=newdest
        if state not in ACTIVE:policy.clear()
    def on_command(name,msg):
        cmds[name]=(time.monotonic(),float(msg.linear.x),float(msg.angular.z))
    def command_kind(now):
        max_age=float(limits['command_age_s'])
        nav=cmds.get('nav');outcmd=cmds.get('final')
        if not nav or now-nav[0]>max_age:return 'NAV_COMMAND_MISSING_OR_STALE'
        if not outcmd or now-outcmd[0]>max_age:return 'FINAL_COMMAND_MISSING_OR_STALE'
        if abs(nav[1])>.03 and abs(outcmd[1])<.01:return 'SAFETY_CHAIN_BLOCKING_TRANSLATION'
        if abs(nav[1])>.03:return 'MOTION_REQUESTED_BUT_NOT_PROGRESSING'
        if abs(nav[2])>.05:return 'ROTATION_ONLY_WITHOUT_TRANSLATION'
        return 'CONTROLLER_NO_TRANSLATION_REQUEST'
    def odom_cb(msg):
        nonlocal triggered,odom_last
        now=time.monotonic()
        odom_last=(now, float(msg.twist.twist.linear.x),float(msg.twist.twist.angular.z))
        if state not in ACTIVE or not map_valid or triggered or not destination:
            policy.clear();return
        xy=msg.pose.pose.position
        result=policy.observe(now,float(xy.x),float(xy.y))
        if not result or not result['stalled']:return
        if now-last_state_time<float(limits['window_s']):return
        # An existing V3 subscriber MUST be present; no direct motor stop bypass.
        triggered=True
        cause=command_kind(now)
        emit('STALL_DETECTED',state=state,destination_id=destination,cause=cause,**result,
             note='V7 handles rotation separately; this guard only requests V3 cancel')
        if args.enforce and pub.get_subscription_count()>0:
            pub.publish(String(data=json.dumps({'command':'CANCEL'})))
            emit('CANCEL_PUBLISHED',reason='NO_TRANSLATION_FOR_STALL_WINDOW',
                 note='V3 cancel is asynchronous; pause Gazebo if robot does not stop')
        elif args.enforce:emit('CANCEL_UNDELIVERABLE',reason='V3 supervisor subscriber missing')
    def watch():
        if state not in ACTIVE or not map_valid:return
        if odom_last is None or time.monotonic()-odom_last[0]>float(limits['maximum_odom_age_s']):
            # Alert only. We cannot safely infer movement if odom is stale.
            if not getattr(watch,'was_stale',False):
                emit('ODOM_STALE_WARNING',state=state,reason='No fresh motion measurements; cannot assert stall')
                watch.was_stale=True
        else: watch.was_stale=False
    node.create_subscription(String,'/robot_01/serving/status',status_cb,10)
    node.create_subscription(Odometry,'/robot_01/odom',odom_cb,10)
    node.create_subscription(Twist,'/robot_01/cmd_vel_nav',lambda m:on_command('nav',m),10)
    node.create_subscription(Twist,'/robot_01/cmd_vel',lambda m:on_command('final',m),10)
    node.create_timer(1.0,watch)
    emit('STARTED',mode='CANCEL_VIA_V3' if args.enforce else 'OBSERVER',stall_window_s=policy.seconds,
         note='Never publishes cmd_vel or Nav2 goals; V7 spin guard remains independent')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        emit('STOPPED');fh.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
