"""Reproduce occluded lost-person tracks, then verify safe mission progress."""
import json,subprocess,sys,time,urllib.request,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
root=Path(__file__).resolve().parents[1];rclpy.init();n=Node('removed_person_visibility_trial')
reports={};history=[];seen={};contact=False
def cb(msg,r):
    global contact
    obj=json.loads(msg.data);reports[r]=obj;history.append(obj)
    contact=contact or obj.get('contact_active',False)
    for tr in obj.get('tracks',[]):
        if tr.get('class')=='person':seen.setdefault(r,set()).add(tr['id'])
subs=[n.create_subscription(String,'/'+r+'/dynamic_safety',lambda m,r=r:cb(m,r),qos_profile_sensor_data) for r in ('robot_01','robot_02')]
def api(path,body=None):
    req=urllib.request.Request('http://127.0.0.1:8765/api/'+path,data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    obj=json.load(urllib.request.urlopen(req,timeout=5))
    if body and not obj.get('ok'):raise RuntimeError(obj)
    return obj
def wait(pred,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if pred():return True
    return False
def fixture(action):
    return subprocess.run([sys.executable,str(root/'scripts/person_scenario.py'),action,'--x','0','--y','-1.2','--yaw','0'],check=action=='spawn',capture_output=True,text=True)
rid='removed-person-'+uuid.uuid4().hex[:12];success=False;error=None;lost={}
try:
    fixture('spawn')
    if not wait(lambda:len(seen)==2,20):raise RuntimeError('Both real cameras did not observe the fixture')
    wait(lambda:False,3.)
    fixture('clear');wait(lambda:False,2.)
    lost={r:[t for t in obj.get('tracks',[]) if t.get('uncertain')] for r,obj in reports.items()}
    api('request',dict(action='SUBMIT',request_id=rid,robot_id='robot_01',mode='DELIVER_DIRECT',table='table_04',side='top'))
    def arrived():
        s=api('status').get('supervisors',{}).get('robot_01',{})
        return s.get('request_id')==rid and s.get('state')=='WAIT_HANDOVER'
    if not wait(arrived,160):raise RuntimeError('Mission remained blocked after fixture removal')
    api('request',dict(action='COMPLETE',request_id=rid,source='table'))
    success=not contact
except Exception as exc:error=str(exc)
finally:
    fixture('clear')
    if not success:
        try:api('request',dict(action='CANCEL',request_id=rid))
        except Exception:pass
    remaining={r:[t for t in obj.get('tracks',[]) if t.get('uncertain')] for r,obj in reports.items()}
    result=dict(success=success,error=error,request_id=rid,actual_detector_only=True,seen_person_ids={r:sorted(v) for r,v in seen.items()},
        lost_after_removal=lost,remaining_at_completion=remaining,contact_detected=contact,history=history)
    p=root/'_local_only'/('removed_person_'+time.strftime('%Y%m%d_%H%M%S')+'.json');p.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='history'},indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
