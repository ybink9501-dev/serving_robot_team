"""Verify lost-person clearance from a new actual robot viewpoint."""
import json,math,time,urllib.request,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
root=Path(__file__).resolve().parents[1];rclpy.init();n=Node('personal_reobserve_trial')
latest={};history=[];truth=None;initial=None;contact=False
def dynamic(msg):
    global latest,contact
    latest=json.loads(msg.data);history.append(latest);contact=contact or latest.get('contact_active',False)
def pose(msg):
    global truth,initial
    for t in msg.transforms:
        if t.child_frame_id=='robot_02':
            truth=(t.transform.translation.x,t.transform.translation.y)
            if initial is None:initial=truth
subs=[n.create_subscription(String,'/robot_02/dynamic_safety',dynamic,qos_profile_sensor_data),n.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',pose,10)]
def api(path,body=None):
    req=urllib.request.Request('http://127.0.0.1:8765/api/'+path,data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    result=json.load(urllib.request.urlopen(req,timeout=5))
    if body and not result.get('ok'):raise RuntimeError(result)
    return result
end=time.monotonic()+10
while time.monotonic()<end and (not latest or truth is None):rclpy.spin_once(n,timeout_sec=.1)
before=[t for t in latest.get('tracks',[]) if t.get('uncertain')];rid='reobserve-'+uuid.uuid4().hex[:12];success=False;error=None
try:
    api('request',dict(action='SUBMIT',request_id=rid,robot_id='robot_02',mode='GO_KITCHEN'))
    end=time.monotonic()+180;last=0.
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if time.monotonic()-last<.5:continue
        last=time.monotonic();s=api('status')
        if s.get('requests',{}).get(rid,{}).get('state')=='SUCCEEDED':
            success=not contact and initial is not None and truth is not None and math.dist(initial,truth)>.5 and not any(t.get('class')=='person' and t.get('uncertain') for t in latest.get('tracks',[]));break
    if not success:error='Mission, viewpoint change or lost-person clearance not verified'
except Exception as exc:error=str(exc)
finally:
    if not success:
        try:api('request',dict(action='CANCEL',request_id=rid))
        except Exception:pass
    report=dict(success=success,error=error,request_id=rid,actual_viewpoint_displacement_m=math.dist(initial,truth) if initial and truth else None,
        before=before,after=[t for t in latest.get('tracks',[]) if t.get('uncertain')],contact_detected=contact,history=history)
    p=root/'_local_only'/('reobserve_'+time.strftime('%Y%m%d_%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
