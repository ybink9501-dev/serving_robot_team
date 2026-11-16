"""Real Gazebo person and real sensors: SLOW, STOP, avoidance and resume.
No injected detector output or velocity bypass. Run in reviewed open test space.
"""
import argparse,json,math,re,subprocess,sys,time,urllib.request,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from serving_robot_person_simulator.pose_cli import PoseStream
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean
gz_node=GzNode()

ap=argparse.ArgumentParser();ap.add_argument('--goal',default='human_lane_goal');ap.add_argument('--robot',default='robot_01');a=ap.parse_args()
root=Path(__file__).resolve().parents[1];rclpy.init();n=Node('personal_human_flow_trial')
latest={};samples=[];events=[];truth={};contacts=False;fixture=None;minimum=math.inf
pose_stream=PoseStream('cafe','test_person_',[a.robot,'test_person'])
def api(path,body=None):
    req=urllib.request.Request('http://127.0.0.1:8765/api/'+path,data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    result=json.load(urllib.request.urlopen(req,timeout=5))
    if body and not result.get('ok'):raise RuntimeError(result)
    return result
def on_json(msg,key):
    global contacts
    latest[key]=json.loads(msg.data)
    if key=='dynamic':contacts=contacts or latest[key].get('contact_active',False)
    if key=='event' or key=='detour':events.append(latest[key])
    if key=='safety':samples.append({'wall':time.monotonic(),'fixture':fixture,'truth':dict(truth),'latest':dict(latest)})
def on_truth(msg):
    global minimum
    for t in msg.transforms:
        if t.child_frame_id==a.robot:
            q=t.transform.rotation
            truth.update(x=t.transform.translation.x,y=t.transform.translation.y,h=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            observed=pose_stream.snapshot().get('test_person')
            if fixture and observed and time.monotonic()-observed[1]<.5:
                minimum=min(minimum,math.hypot(truth['x']-observed[0][0],truth['y']-observed[0][1]))
subs=[]
for topic,key in [('safety_status','safety'),('dynamic_safety','dynamic'),('navigation_event','event'),('detour_state','detour')]:
    subs.append(n.create_subscription(String,'/'+a.robot+'/'+topic,lambda m,k=key:on_json(m,k),qos_profile_sensor_data))
subs.append(n.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',on_truth,10))
subs.append(n.create_subscription(Twist,'/'+a.robot+'/cmd_vel_nav_smooth',lambda m:latest.update(nav_v=m.linear.x,nav_w=m.angular.z),qos_profile_sensor_data))
subs.append(n.create_subscription(Twist,'/'+a.robot+'/cmd_vel',lambda m:latest.update(base_v=m.linear.x,base_w=m.angular.z),qos_profile_sensor_data))
subs.append(n.create_subscription(Odometry,'/'+a.robot+'/odom',lambda m:latest.update(odom_v=m.twist.twist.linear.x,odom_w=m.twist.twist.angular.z),qos_profile_sensor_data))
def until(pred,seconds,label):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if pred():print(label,flush=True);return
    raise RuntimeError(label+' timeout')
def actual_pose(model=None):
    raw=subprocess.run(['gz','model','-m',model or a.robot,'-p'],capture_output=True,text=True,check=True).stdout
    vectors=[]
    for values in re.findall(r'\[([^\]]+)\]',raw):
        try:v=list(map(float,values.split()))
        except ValueError:continue
        if len(v)==3:vectors.append(v)
    if len(vectors)<2:raise RuntimeError('No actual pose')
    return vectors[-2][0],vectors[-2][1],vectors[-1][2]
def place(forward,left,spawn=False):
    global fixture
    previous=fixture
    x,y,h=truth['x'],truth['y'],truth['h'];fixture=(x+forward*math.cos(h)-left*math.sin(h),y+forward*math.sin(h)+left*math.cos(h))
    command=[sys.executable,str(root/'scripts/person_scenario.py'),'spawn' if spawn else 'move',
        '--yaw',str(h+math.pi)]
    if spawn:command+=['--x',str(fixture[0]),'--y',str(fixture[1])]
    else:
        observed=pose_stream.snapshot().get('test_person')
        previous=observed[0] if observed else previous
        started=time.monotonic();duration=2.
        while True:
            fraction=min(1.,(time.monotonic()-started)/duration)
            pose=GzPose();pose.name='test_person'
            pose.position.x=previous[0]+fraction*(fixture[0]-previous[0])
            pose.position.y=previous[1]+fraction*(fixture[1]-previous[1])
            pose.orientation.z=math.sin((h+math.pi)/2);pose.orientation.w=math.cos((h+math.pi)/2)
            ok,response=gz_node.request('/world/cafe/set_pose',pose,GzPose,Boolean,1000)
            if not ok or not response.data:raise RuntimeError('Person motion rejected')
            rclpy.spin_once(n,timeout_sec=.05)
            if fraction==1.:break
        return x,y,h
    child=subprocess.Popen(command)
    while child.poll() is None:rclpy.spin_once(n,timeout_sec=.05)
    if child.returncode:raise RuntimeError('Physical fixture operation failed')
    return x,y,h
def clear():
    global fixture
    if fixture is None:return
    result=subprocess.run([sys.executable,str(root/'scripts/person_scenario.py'),'clear'],capture_output=True)
    if result.returncode:raise RuntimeError('Physical fixture cleanup failed')
    fixture=None
def slow(s):
    q=s['latest'].get('safety',{});d=q.get('person_control_distance_m')
    return q.get('detector_fresh') and q.get('dynamic_fresh') and d is not None and .8<d<=1.5 and q.get('state')=='SLOW' and s['latest'].get('nav_v',0)>.02 and 0<q.get('output_v',0)<s['latest']['nav_v']-.003
def stopped(s):
    q=s['latest'].get('safety',{});d=q.get('person_control_distance_m')
    return q.get('detector_fresh') and q.get('dynamic_fresh') and d is not None and d<=.8 and q.get('output_v',1)==0 and q.get('output_w',1)==0 and abs(s['latest'].get('base_v',1))<.001 and abs(s['latest'].get('base_w',1))<.001 and abs(s['latest'].get('odom_v',1))<.02
rid='human-flow-'+uuid.uuid4().hex[:12];report={'request_id':rid,'actual_sensor_only':True};success=False
try:
    until(lambda:truth and latest.get('safety',{}).get('detector_fresh'),60,'sensors ready')
    start=dict(truth)
    api('request',{'action':'SUBMIT','request_id':rid,'robot_id':a.robot,'mode':'DELIVER_DIRECT','table':a.goal})
    until(lambda:math.hypot(truth['x']-start['x'],truth['y']-start['y'])>.12 and latest.get('nav_v',0)>.02,45,'physical motion before person')
    index=len(samples);place(1.5,-.9,True)
    until(lambda:sum(slow(s) for s in samples[index:])>=3,25,'person SLOW observed')
    report['slow_samples']=sum(slow(s) for s in samples[index:])
    index=len(samples);place(1.0,0.)
    until(lambda:sum(stopped(s) for s in samples[index:])>=3,15,'person STOP and physical standstill observed')
    report['stop_samples']=sum(stopped(s) for s in samples[index:])
    x,y,h=place(1.3,0.);obstacle=fixture
    def passed():return (truth['x']-obstacle[0])*math.cos(h)+(truth['y']-obstacle[1])*math.sin(h)>.45
    until(passed,130,'physical avoidance past blocking person')
    report['detour_events']=[e for e in events if e.get('state')=='EXECUTING' or 'DETOUR' in e.get('event','')]
    clear();start=dict(truth)
    until(lambda:math.hypot(truth['x']-start['x'],truth['y']-start['y'])>.1 or api('status').get('supervisors',{}).get(a.robot,{}).get('state')=='WAIT_HANDOVER',25,'resume after removal')
    until(lambda:api('status').get('supervisors',{}).get(a.robot,{}).get('state')=='WAIT_HANDOVER',90,'goal after person encounter')
    api('request',{'action':'COMPLETE','request_id':rid,'source':'table'})
    success=not contacts and math.isfinite(minimum) and minimum>=.54
except Exception as exc:report['error']=str(exc);print(str(exc),flush=True)
finally:
    try:clear()
    except Exception as exc:report['cleanup_error']=str(exc);success=False
    try:
        if not success:api('request',{'action':'CANCEL','request_id':rid})
    except Exception:pass
    report.update(success=success,contact_detected=contacts,minimum_actual_center_distance_m=minimum if math.isfinite(minimum) else None,samples=samples,events=events)
    p=root/'_local_only'/('human_flow_'+time.strftime('%Y%m%d_%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','events')},indent=2),flush=True)
    pose_stream.close();n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
