"""Public manager commands -> real people -> robot perception/safety -> mission."""
import json,math,time,uuid,urllib.request,yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from serving_robot_person_simulator.pose_cli import PoseStream

root=Path(__file__).resolve().parents[1]
entry=json.loads((root/'_local_only/runtime.json').read_text())
config=yaml.safe_load(Path(entry['config']).read_text())
if entry.get('mode')!='operation' or config.get('site_id')!='validation_cafe':
    raise SystemExit('This fixed blocking-person scenario requires --site config/integration/sites/validation_cafe.yaml --people; corridor spawn poses are a different scenario')
rclpy.init();n=Node('personal_manager_chain_check')
latest={};history=[];samples=[];contact=False
poses=PoseStream('cafe','test_person_',['robot_01','robot_02'])
rid='manager-chain-'+uuid.uuid4().hex[:12]
def callback(msg,key):
    global contact
    value=json.loads(msg.data);latest[key]=value
    if key.startswith('dynamic_'):contact=contact or value.get('contact_active',False)
    if key=='manager':history.append({'wall':time.monotonic(),'status':value})
    if key=='safety_robot_01':samples.append({'wall':time.monotonic(),'latest':dict(latest)})
subs=[]
for topic,key in [('/person_simulator/status','manager')]+[(f'/{r}/{topic}',f'{key}_{r}') for r in ('robot_01','robot_02') for topic,key in [('perception_status','perception'),('dynamic_safety','dynamic'),('safety_status','safety'),('serving/status','mission')]]:
    subs.append(n.create_subscription(String,topic,lambda m,k=key:callback(m,k),qos_profile_sensor_data))
pub=n.create_publisher(String,'/person_simulator/command',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def until(predicate,seconds,label):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if predicate():print(label,flush=True);return
    raise RuntimeError(label+' timeout')
def send(action,**fields):
    pub.publish(String(data=json.dumps(dict(action=action,**fields))));spin(.3)
def api(body=None):
    req=urllib.request.Request('http://127.0.0.1:8765/api/'+('request' if body else 'status'),data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    result=json.load(urllib.request.urlopen(req,timeout=5))
    if body and not result.get('ok'):raise RuntimeError(result)
    return result
def people():return latest.get('manager',{}).get('people',[])
def reaction(sample):
    x=sample['latest'];q=x.get('safety_robot_01',{});m=x.get('mission_robot_01',{})
    d=q.get('person_control_distance_m')
    return (m.get('request_id')==rid and m.get('state')=='GOING_DIRECT'
            and q.get('detector_fresh') and q.get('dynamic_fresh') and not q.get('traffic_hold')
            and d is not None and d<=1.5 and q.get('state') in ('SLOW','STOP')
            and q.get('output_v',1)<=.15)
report={'request_id':rid,'person_source':'public person_simulator manager; bundled visual+collision; no detector injection'};success=False
try:
    until(lambda:latest.get('manager',{}).get('robot_pose_valid') and pub.get_subscription_count()>0 and all(latest.get('perception_'+r,{}).get('fresh') and latest.get('mission_'+r,{}).get('nav_ready') for r in ('robot_01','robot_02')),80,'manager and both robot pipelines ready')
    if people():raise RuntimeError('Test requires no preexisting people; refusing to remove user people')
    send('spawn',count=2,mode='legacy')
    until(lambda:len(people())==2 and all(p.get('position') for p in people()),20,'two actual manager people spawned')
    until(lambda:all((latest.get('perception_'+r,{}).get('count',0)+latest.get('perception_'+r,{}).get('candidate_count',0))>0 and any(t.get('class')=='person' and not t.get('uncertain') for t in latest.get('dynamic_'+r,{}).get('tracks',[])) for r in ('robot_01','robot_02')),25,'both robot camera and dynamic tracker observe manager people')
    report['both_robots_detect_manager_people']=True
    api(dict(action='SUBMIT',request_id=rid,robot_id='robot_01',mode='DELIVER_DIRECT',table='meet_right'))
    until(lambda:sum(reaction(s) for s in samples)>=3,35,'manager person affects active robot safety')
    report['safety_reaction_samples']=sum(reaction(s) for s in samples)
    before={p['id']:p['position'] for p in people()};send('start')
    until(lambda:len(people())==2 and all(p.get('position') and math.dist(before[p['id']][:2],p['position'][:2])>.12 for p in people()),22,'both manager people physically move')
    report['actual_displacements_m']={p['name']:math.dist(before[p['id']][:2],p['position'][:2]) for p in people()}
    send('stop');spin(2.)
    assert all(p['paused'] and math.hypot(*p['velocity_cmd'])<1e-6 for p in people()),'Person stop command not zero'
    stopped={p['id']:p['position'] for p in people()};spin(2.)
    report['stop_drift_m']={p['name']:math.dist(stopped[p['id']][:2],p['position'][:2]) for p in people()}
    assert max(report['stop_drift_m'].values())<.04,'People do not physically stop'
    send('cleanup')
    until(lambda:not people() and all(time.monotonic()-stamp>.7 for name,(p,stamp) in poses.snapshot().items() if name.startswith('test_person_')),12,'manager removal reflected in actual pose stream')
    report['actual_people_removed']=True
    until(lambda:latest.get('mission_robot_01',{}).get('request_id')==rid and latest['mission_robot_01'].get('state')=='WAIT_HANDOVER',130,'robot completes route after manager cleanup')
    api(dict(action='COMPLETE',request_id=rid,source='table'))
    until(lambda:api().get('requests',{}).get(rid,{}).get('state')=='SUCCEEDED',10,'same request completed')
    success=not contact
except Exception as exc:report['error']=str(exc);print(str(exc),flush=True)
finally:
    # This trial creates only the owned two people in an initially empty manager.
    if report.get('both_robots_detect_manager_people') or len(people())==2:send('cleanup')
    if not success:
        try:api(dict(action='CANCEL',request_id=rid))
        except Exception:pass
    report.update(success=success,contact_detected=contact,history=history,samples=samples)
    path=root/'_local_only'/('manager_chain_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('history','samples')},indent=2),flush=True)
    poses.close();n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
