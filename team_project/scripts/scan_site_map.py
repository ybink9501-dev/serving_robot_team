"""Mapping-only scan route with fresh LiDAR/contact stops; saves actual SLAM map.

Test operator substitute, not an operation-mode navigation controller.
Never writes synthetic occupancy or a Gazebo pose into SLAM.
"""
import argparse,json,math,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer,TransformListener

ap=argparse.ArgumentParser();ap.add_argument('--site',required=True);ap.add_argument('--output',required=True)
ap.add_argument('--timeout',type=float,default=600.);args=ap.parse_args()
root=Path(__file__).resolve().parents[1]
runtime=json.loads((root/'_local_only/runtime.json').read_text())
if runtime['mode']!='mapping':raise SystemExit('Refusing cmd_vel: managed runtime is not mapping')
if runtime.get('mapping_stack','personal')!='personal':
    raise SystemExit('Refusing direct cmd_vel: TEAM mapping owns the safety/control chain; use run_mapping_session.sh')
site=yaml.safe_load((root/args.site).read_text());route=site['mapping_route']
rclpy.init();n=Node('personal_mapping_scan_operator',parameter_overrides=[Parameter('use_sim_time',value=True)]);tf=Buffer();listener=TransformListener(tf,n)
pub=n.create_publisher(Twist,'/robot_01/cmd_vel',10);scan=None;scan_wall=0.;contacts={};contact_wall={};samples=[]
def scan_cb(msg):
    global scan,scan_wall
    scan=msg;scan_wall=time.monotonic()
def contact_cb(msg,index):
    contact_wall[index]=time.monotonic()
    contacts[index]=any(not any('ground' in c.name for c in (v.collision1,v.collision2)) for v in msg.contacts)
subs=[n.create_subscription(LaserScan,'/robot_01/scan',scan_cb,qos_profile_sensor_data)]
for i in range(3):subs.append(n.create_subscription(Contacts,'/robot_01/contacts/sensor_'+str(i),lambda m,i=i:contact_cb(m,i),qos_profile_sensor_data))
start=time.monotonic();step=0;blocked=None;success=False;last_sample=0.;error=None
def norm(a):return math.atan2(math.sin(a),math.cos(a))
try:
    while time.monotonic()-start<args.timeout:
        rclpy.spin_once(n,timeout_sec=.05);cmd=Twist()
        if any(contacts.values()):raise RuntimeError('Physical contact; mapping stopped')
        try:
            t=tf.lookup_transform('map','robot_01/base_footprint',Time())
            q=t.transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            x=t.transform.translation.x;y=t.transform.translation.y
            age=n.get_clock().now().nanoseconds*1e-9-(t.header.stamp.sec+t.header.stamp.nanosec*1e-9)
            ready=abs(age)<1. and scan is not None and time.monotonic()-scan_wall<.5 and len(contact_wall)==3 and all(time.monotonic()-v<1. for v in contact_wall.values())
        except Exception:ready=False
        if not ready:pub.publish(cmd);continue
        if step>=len(route):success=True;break
        tx,ty=route[step];d=math.hypot(tx-x,ty-y)
        if d<.16:step+=1;blocked=None;pub.publish(cmd);continue
        turn=norm(math.atan2(ty-y,tx-x)-a)
        echoes=[r for r in scan.ranges if math.isfinite(r) and r>=scan.range_min]
        nearest=min(echoes,default=scan.range_max)
        front=[r for i,r in enumerate(scan.ranges) if abs(norm(scan.angle_min+i*scan.angle_increment))<.35 and r>=scan.range_min]
        front_clear=min(front,default=0.)
        if abs(turn)>.15:
            if nearest>.38:cmd.angular.z=max(-.4,min(.4,turn))
        elif front_clear>.5:
            cmd.linear.x=min(.18,.6*d);cmd.angular.z=max(-.2,min(.2,turn))
        if cmd.linear.x==0. and cmd.angular.z==0.:
            if blocked is None:blocked=time.monotonic()
            if time.monotonic()-blocked>12:raise RuntimeError('Reviewed scan route blocked; no blind detour')
        else:blocked=None
        pub.publish(cmd)
        if time.monotonic()-last_sample>1:
            last_sample=time.monotonic();samples.append(dict(step=step,map_pose=[x,y,a],front_clear_m=front_clear,nearest_m=nearest,v=cmd.linear.x,w=cmd.angular.z))
            print(json.dumps(samples[-1]),flush=True)
    if not success:raise RuntimeError('Mapping scan route timeout')
    pub.publish(Twist())
    until=time.monotonic()+4
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
    target=root/args.output;target.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(target),'--ros-args',
        '-p','use_sim_time:=true','-p','save_map_timeout:=10.0','-r','map:=/map'],check=True,timeout=30)
except Exception as exc:error=str(exc);success=False;print(error,flush=True)
finally:
    for _ in range(3):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
    report=dict(success=success,error=error,source='SLAM Toolbox consuming actual /robot_01/scan',
                route_points_reached=step,site=args.site,output=args.output,samples=samples)
    target=root/(args.output+'.provenance.json');target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
