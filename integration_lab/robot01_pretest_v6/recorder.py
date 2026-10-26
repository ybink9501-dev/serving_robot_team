#!/usr/bin/env python3
"""Passive evidence recorder with no goal or velocity publishers.
Run before one integrated serving test; Ctrl+C or --duration ends capture.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from checks import HERE,read_yaml


def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=420)
    p.add_argument('--out',help='JSONL path (default /tmp/robot01_v6_YYYYMMDD_HHMMSS.jsonl)')
    a=p.parse_args()
    if a.duration<10 or a.duration>7200:raise SystemExit('Duration must be 10..7200 seconds')
    out=Path(a.out or f'/tmp/robot01_v6_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jsonl')
    if out.exists():raise SystemExit('Will not overwrite log: '+str(out))
    config=read_yaml(HERE/'config.yaml')['monitor']
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init();node=Node('robot01_passive_recorder_v6');last_odom=0;history=deque()
    active_state='IDLE';yaw_prev=None;unwrapped=0.;warned_at=0.
    fh=out.open('x',encoding='utf8')
    def emit(name,content):
        row={'utc':datetime.now(timezone.utc).isoformat(),'t_monotonic_s':round(time.monotonic(),4),
             'channel':name,'data':content}
        fh.write(json.dumps(row,ensure_ascii=False)+'\n');fh.flush()
    def on_text(source):
        def cb(m):
            nonlocal active_state
            try:obj=json.loads(m.data)
            except Exception:obj={'raw':m.data}
            if source=='serving_status':active_state=obj.get('state',active_state)
            emit(source,obj)
        return cb
    def on_odom(m):
        nonlocal last_odom,yaw_prev,unwrapped,warned_at
        now=time.monotonic()
        x,y=float(m.pose.pose.position.x),float(m.pose.pose.position.y)
        q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if yaw_prev is not None:unwrapped+=math.atan2(math.sin(yaw-yaw_prev),math.cos(yaw-yaw_prev))
        yaw_prev=yaw
        history.append((now,x,y,unwrapped))
        window=float(config['no_progress_window_s'])
        while history and now-history[0][0]>window:history.popleft()
        if now-last_odom<float(config['odom_record_interval_s']):return
        last_odom=now
        t=m.twist.twist
        emit('odom',{'x':x,'y':y,'yaw_unwrapped':unwrapped,'vx':float(t.linear.x),
             'wz':float(t.angular.z),'serving_state':active_state})
        if (active_state in ('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP')
           and history and now-history[0][0]>=window*.9
           and now-warned_at>window):
            t0,x0,y0,a0=history[0]
            displacement=math.hypot(x-x0,y-y0)
            rotation=abs(unwrapped-a0)
            if (displacement<float(config['minimum_progress_m']) and
                rotation>float(config['excessive_rotation_without_progress_rad'])):
                emit('WARNING_EXCESSIVE_ROTATION',{'rotation_rad':rotation,
                   'displacement_m':displacement,'window_s':now-t0,
                   'note':'Observer only; use existing cancel to stop a bad mission'})
                warned_at=now
    node.create_subscription(Odometry,'/robot_01/odom',on_odom,10)
    for topic,name in [('/robot_01/robot_state','robot_state'),
             ('/robot_01/navigation_event','nav_event'),
             ('/robot_01/serving/status','serving_status'),
             ('/robot_01/person_safety_event_lab','person_event')]:
        node.create_subscription(String,topic,on_text(name),10)
    # Deliberately subscribe to output; does not publish any velocities.
    for channel in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel_semantic','cmd_vel'):
        node.create_subscription(Twist,'/robot_01/'+channel,
            lambda m, name=channel:emit(name,{'vx':float(m.linear.x),
                'wz':float(m.angular.z)}),10)
    print('PASSIVE RECORDER:',out,flush=True)
    emit('RECORDER_STARTED',{'duration_s':a.duration,'no_motion_command':True})
    end=time.monotonic()+a.duration
    try:
        while time.monotonic()<end and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:pass
    finally:
        emit('RECORDER_STOPPED',{})
        fh.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('CAPTURED:',out,flush=True)
if __name__=='__main__':main()
