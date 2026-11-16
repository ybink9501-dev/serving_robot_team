"""Observe real detector input and velocity outputs during a blocking-person mission.

Run person_scenario.py spawn --relative-to robot_02 --distance 1.3 first.
This tool never publishes a person detection or bypasses the safety chain.
"""
import json,time,uuid,urllib.request,subprocess,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String

root=Path(__file__).resolve().parents[1];out=root/'_local_only';out.mkdir(exist_ok=True)
def post(data):
    req=urllib.request.Request('http://127.0.0.1:8765/api/request',json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as r:return json.load(r)
rclpy.init();n=Node('human_acceptance');samples=[];latest={};subscriptions=[];events=[]
def cb(msg,key):
    if isinstance(msg,Twist):latest[key]={'v':msg.linear.x,'w':msg.angular.z}
    else:
        try:latest[key]=json.loads(msg.data)
        except ValueError:return
        if key in ('navigation_event','person_replan_event'):events.append(latest[key])
        if key=='safety_status':samples.append({'time':time.monotonic(),**{k:dict(v) for k,v in latest.items()}})
for topic,typ in [('safety_status',String),('navigation_event',String),('person_replan_event',String),('cmd_vel_nav_smooth',Twist),('cmd_vel_semantic',Twist),('cmd_vel',Twist)]:
    subscriptions.append(n.create_subscription(typ,'/robot_02/'+topic,lambda m,k=topic:cb(m,k),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<2:rclpy.spin_once(n,timeout_sec=.05)
rid='person-test-'+uuid.uuid4().hex[:12]
reply=post({'action':'SUBMIT','request_id':rid,'robot_id':'robot_02','table':'table_04','mode':'DELIVER_DIRECT'})
print(reply,flush=True)
start=time.monotonic();moved_close=False
while time.monotonic()-start<35:
    rclpy.spin_once(n,timeout_sec=.05)
    if not moved_close and time.monotonic()-start>=12:
        # Reposition the actual Gazebo fixture; never publish detector results.
        subprocess.run([sys.executable,str(root/'scripts/person_scenario.py'),'move',
                        '--relative-to','robot_02','--distance','0.7','--duration','0'],check=True)
        moved_close=True
slow=[s for s in samples if s['safety_status'].get('state')=='SLOW' and s.get('cmd_vel_nav_smooth',{}).get('v',0)>.01 and 0<s.get('cmd_vel_semantic',{}).get('v',0)<s['cmd_vel_nav_smooth']['v']]
stop=[s for s in samples if s['safety_status'].get('person_control_distance_m') is not None and s['safety_status']['person_control_distance_m']<=.8 and abs(s.get('cmd_vel',{}).get('v',1))<.001 and abs(s.get('cmd_vel_semantic',{}).get('v',1))<.001]
# This fixture blocks the requested route: dynamic OBSERVE/WAIT_BLOCKED is
# intentionally allowed to stop before the lower-priority semantic SLOW band.
early_stop=[s for s in samples if s['safety_status'].get('detector_fresh') is True
            and s['safety_status'].get('person_control_distance_m') is not None
            and .8<s['safety_status']['person_control_distance_m']<=1.5
            and s['safety_status'].get('state')=='STOP'
            and s.get('cmd_vel_nav_smooth',{}).get('v',0)>.01
            and abs(s.get('cmd_vel',{}).get('v',1))<.001
            and abs(s.get('cmd_vel',{}).get('w',1))<.001]
report={'request_id':rid,'actual_detector_only':True,'scenario':'stationary person blocking requested route',
        'slow_samples':len(slow),'slow_verified':bool(slow),'early_blocked_stop_samples':len(early_stop),
        'distance_metric':'front_person_m used by the safety controller; detector centre is recorded separately','fixture_moved_close':moved_close,'person_stop_samples':len(stop),
        'stationary_replan_events':[e for e in events if 'REPLAN' in e.get('event','')],'samples':samples,'events':events}
path=out/('human_acceptance_'+time.strftime('%Y%m%d_%H%M%S')+'.json');path.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('samples','events')},indent=2),flush=True)
print(post({'action':'CANCEL','request_id':rid}),flush=True)
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if moved_close and (slow or early_stop) and stop else 1)
