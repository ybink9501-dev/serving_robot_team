"""Two real robot missions meeting in a physically narrow corridor."""
import argparse,json,math,time,urllib.request,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

ap=argparse.ArgumentParser();ap.add_argument('--submit-first',choices=['robot_01','robot_02'],default='robot_01');ap.add_argument('--expect-winner',choices=['robot_01','robot_02']);ap.add_argument('--expect-reason');args=ap.parse_args()
root=Path(__file__).resolve().parents[1];rclpy.init();n=Node('personal_corridor_trial')
truth={};minimum=math.inf;corridor=set();history=[];recovery_seen=False;contact=False;entry_order=[];priority_observed=None
def on_truth(msg):
    global minimum
    for t in msg.transforms:
        if t.child_frame_id in ('robot_01','robot_02'):
            p=t.transform.translation;truth[t.child_frame_id]=(p.x,p.y)
            if abs(p.x)<1. and abs(p.y)<.45:
                corridor.add(t.child_frame_id)
                if t.child_frame_id not in entry_order:entry_order.append(t.child_frame_id)
    if len(truth)==2:minimum=min(minimum,math.dist(truth['robot_01'],truth['robot_02']))
sub=n.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',on_truth,10)
def api(path,body=None):
    req=urllib.request.Request('http://127.0.0.1:8765/api/'+path,data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    reply=json.load(urllib.request.urlopen(req,timeout=5))
    if body and not reply.get('ok'):raise RuntimeError(reply)
    return reply
ids={r:'corridor-'+uuid.uuid4().hex[:12] for r in ('robot_01','robot_02')};confirmed=set();success=False;error=None;mission_start=None;first_recovery=None;completion_times={}
try:
    end=time.monotonic()+80
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
        try:s=api('status')
        except Exception:continue
        if len(truth)==2 and all(s.get('supervisors',{}).get(r,{}).get('nav_ready') and s.get('safety',{}).get(r,{}).get('detector_fresh') for r in ids):break
    else:raise RuntimeError('Readiness timeout')
    mission_start=time.monotonic()
    missions=[('robot_01','meet_right'),('robot_02','meet_left')]
    missions.sort(key=lambda x:x[0]!=args.submit_first)
    for r,goal in missions:
        print(api('request',dict(action='SUBMIT',request_id=ids[r],robot_id=r,mode='DELIVER_DIRECT',table=goal)),flush=True)
    end=time.monotonic()+240;last=0.
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if time.monotonic()-last<.4:continue
        last=time.monotonic();s=api('status');history.append(s)
        if s.get('recovery') and first_recovery is None:
            first_recovery=time.monotonic()-mission_start
            priority_observed=dict(s['recovery'].get('priority') or {},actual_winner=s['recovery']['winner'])
        recovery_seen=recovery_seen or bool(s.get('recovery'))
        contact=contact or any(v.get('contact_active',False) for v in s.get('dynamic',{}).values())
        for r in ids:
            m=s.get('supervisors',{}).get(r,{})
            if m.get('request_id')==ids[r] and m.get('state')=='WAIT_HANDOVER' and r not in confirmed:
                api('request',dict(action='COMPLETE',request_id=ids[r],source='table'));confirmed.add(r)
        for r,rid in ids.items():
            if s.get('requests',{}).get(rid,{}).get('state')=='SUCCEEDED':completion_times.setdefault(r,time.monotonic()-mission_start)
        if all(s.get('requests',{}).get(rid,{}).get('state')=='SUCCEEDED' for rid in ids.values()):
            success=recovery_seen and corridor==set(ids) and minimum>=.9 and not contact
            if args.expect_winner:success=success and priority_observed is not None and priority_observed.get('actual_winner')==args.expect_winner and entry_order[0]==args.expect_winner
            if args.expect_reason:success=success and priority_observed is not None and priority_observed.get('reason')==args.expect_reason
            break
    if not success:error='Meeting did not satisfy yield + corridor passage + both completion + clearance'
except Exception as exc:error=str(exc)
finally:
    if not success:
        for rid in ids.values():
            try:api('request',dict(action='CANCEL',request_id=rid))
            except Exception:pass
    report=dict(success=success,error=error,priority_observed=priority_observed,entry_order=entry_order,submit_first=args.submit_first,elapsed_s=time.monotonic()-mission_start if mission_start else None,first_recovery_s=first_recovery,completion_times_s=completion_times,request_ids=ids,recovery_seen=recovery_seen,
        robots_physically_in_corridor=sorted(corridor),minimum_actual_robot_separation_m=minimum if math.isfinite(minimum) else None,
        contact_detected=contact,operator_confirmations_simulated=True,history=history)
    p=root/'_local_only'/('corridor_trial_'+time.strftime('%Y%m%d_%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if success else 1)
