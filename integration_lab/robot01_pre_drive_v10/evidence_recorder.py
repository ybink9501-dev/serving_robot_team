#!/usr/bin/env python3
"""Stage 4 - finite, read-only Robot 01 test evidence capture; no ROS commands."""
import argparse,json,math,time
from pathlib import Path
from datetime import datetime, timezone


def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=int,default=600)
    p.add_argument('--out',default='/tmp/robot01_evidence_v10.jsonl');args=p.parse_args()
    if not 10<=args.duration<=7200:raise SystemExit('duration must be 10..7200 seconds')
    out=Path(args.out).expanduser()
    if out.exists():raise SystemExit(f'Output exists: {out}')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    rclpy.init();node=Node('robot01_evidence_recorder_v10')
    node.set_parameters([Parameter('use_sim_time',value=True)])
    fh=out.open('x',encoding='utf-8')
    def emit(event,data):
        fh.write(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),
                             'channel':event,'data':data},ensure_ascii=False)+'\n');fh.flush()
    last_odom=0.;last_tf=0.
    def on_odom(m):
        nonlocal last_odom
        now=time.monotonic()
        if now-last_odom<.25:return
        last_odom=now
        q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        emit('odom',{'x':float(m.pose.pose.position.x),'y':float(m.pose.pose.position.y),
                     'yaw':yaw,'vx':float(m.twist.twist.linear.x),'wz':float(m.twist.twist.angular.z)})
    def parse_channel(topic,msg):
        try:v=json.loads(msg.data)
        except (ValueError,TypeError):v={'raw':msg.data}
        emit(topic,v)
    node.create_subscription(Odometry,'/robot_01/odom',on_odom,10)
    for channel,topic in [('serving_status','/robot_01/serving/status'),
                          ('robot_state','/robot_01/robot_state'),
                          ('nav_event','/robot_01/navigation_event')]:
        node.create_subscription(String,topic,lambda m,ch=channel:parse_channel(ch,m),10)
    for ch in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel_semantic','cmd_vel'):
        node.create_subscription(Twist,'/robot_01/'+ch,
             lambda m,ch=ch:emit(ch,{'vx':float(m.linear.x),'wz':float(m.angular.z)}),10)
    tf=Buffer();listener=TransformListener(tf,node)
    def timer():
        nonlocal last_tf
        now=time.monotonic()
        if now-last_tf<1.:return
        last_tf=now
        try:
            t=tf.lookup_transform('map','base_footprint',Time())
            q=t.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            emit('map_pose',{'x':t.transform.translation.x,'y':t.transform.translation.y,'yaw':yaw})
        except Exception as e:emit('map_tf_unavailable',{'detail':str(e)[:140]})
    node.create_timer(.5,timer)
    emit('RECORDER_STARTED',{'duration_s':args.duration,'no_goal_no_velocity_publisher':True})
    print('CAPTURING:',out,flush=True)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:pass
    finally:
        emit('RECORDER_STOPPED',{'duration_s':round(time.monotonic()-start,2)})
        fh.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('RECORDER_FINISHED:',out,flush=True)

if __name__=='__main__':main()
