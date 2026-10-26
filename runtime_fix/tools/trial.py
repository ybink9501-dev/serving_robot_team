#!/usr/bin/env python3
"""Live end-to-end trial. Reports physical Gazebo truth and never infers success from publish."""
import argparse,json,math,re,subprocess,time,uuid,sys
import yaml
from pathlib import Path


def gazebo_pose(robot):
    try:
        cmd=subprocess.run(['gz','model','-m',robot,'-p'],capture_output=True,text=True,timeout=8)
        z=re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*\[([\d.\-+eE]+)\s+([\d.\-+eE]+)\s+([\d.\-+eE]+)\]\s*\[([\d.\-+eE]+)\s+([\d.\-+eE]+)\s+([\d.\-+eE]+)\]',cmd.stdout)
        if not z:return None
        return [float(z.group(1)),float(z.group(2)),float(z.group(6))]
    except Exception:return None


def trial(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument('robot',choices=['robot_01','robot_02'])
    p.add_argument('table',choices=[f'table_{i:02}' for i in range(1,5)])
    p.add_argument('--timeout',type=float,default=240)
    p.add_argument('--mode',choices=['KITCHEN_PICKUP','DELIVER_DIRECT'],default='KITCHEN_PICKUP')
    p.add_argument('--mission-only',action='store_true',help='Mission-only diagnosis; does not certify physical 10cm/yaw criteria')
    a=p.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init();node=Node('live_trial_'+uuid.uuid4().hex[:8]);events=[];responses=[];nav_events=[]
    node.create_subscription(String,f'/{a.robot}/serving/status',lambda m:events.append(json.loads(m.data)),10)
    node.create_subscription(String,'/fleet/reply',lambda m:responses.append(json.loads(m.data)),10)
    node.create_subscription(String,f'/{a.robot}/navigation_event',lambda m:nav_events.append(json.loads(m.data)),10)
    pub=node.create_publisher(String,'/fleet/request',10)
    rid='trial-'+uuid.uuid4().hex[:12]
    def cmd(action,source=None):
        payload={'action':action,'request_id':rid}
        if source:payload['source']=source
        if action=='SUBMIT':payload.update(robot_id=a.robot,table=a.table,mode=a.mode,source='operator')
        start=len(responses)
        pub.publish(String(data=json.dumps(payload)))
        deadline=time.monotonic()+7
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.15)
            hit=[v for v in responses[start:] if v.get('request_id')==rid and v.get('action')==action]
            if hit:return hit[-1]
        return {'ok':False,'error':'No Fleet acknowledgment'}
    results={'robot':a.robot,'table':a.table,'mode':a.mode,'request_id':rid,'start_truth':gazebo_pose(a.robot),
             'events':[],'status':'FAIL','final_truth':None,'home_error_m':None,
             'physical_10cm_verified':False,'home_yaw_error_rad':None,'table_arrivals':[],
             'pickup_arrivals':[],'physical_table_verified':False}
    try:
        deadline=time.monotonic()+a.timeout
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
        if pub.get_subscription_count()==0:results['failure']='Fleet subscriber missing';return results
        response=cmd('SUBMIT')
        if not response.get('ok'):results['failure']=response;return results
        load_sent=False;complete_sent=False;last_state=None
        runtime=Path(__file__).resolve().parent.parent/'ros_ws/src/cafe_autonomy/config/runtime_limits.yaml'
        limits=yaml.safe_load(runtime.read_text(encoding='utf8'))
        xy_max=limits['arrival']['xy_tolerance_m'];yaw_max=limits['arrival']['yaw_tolerance_rad']
        def evidence(target_name):
            truth=gazebo_pose(a.robot)
            goals=[e for e in nav_events if e.get('request_id')==rid and e.get('event')=='NAV_GOAL_SENT' and e.get('target')==target_name]
            evidence={'truth':truth,'target':target_name,'goal':goals[-1] if goals else None,'xy_error_m':None,'yaw_error_rad':None,'pass':False}
            if truth and goals:
                g=goals[-1];xy=g.get('xy');ya=g.get('yaw')
                if isinstance(xy,list) and len(xy)==2 and isinstance(ya,(int,float)):
                    d=math.hypot(truth[0]-xy[0],truth[1]-xy[1])
                    da=abs(math.atan2(math.sin(truth[2]-ya),math.cos(truth[2]-ya)))
                    evidence.update(xy_error_m=round(d,6),yaw_error_rad=round(da,6),**{'pass':bool(d<=xy_max and da<=yaw_max)})
            return evidence
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.25)
            if not events:continue
            s=events[-1]
            if s.get('request_id')!=rid:continue
            state=s.get('state')
            if state!=last_state:
                results['events'].append({'state':state,'at_s':round(a.timeout-(deadline-time.monotonic()),2)})
                print('STATE',state,flush=True);last_state=state
            if state=='ERROR':results['failure']=s.get('reason','Mission ERROR');return results
            if state=='WAIT_LOAD' and not load_sent:
                results['pickup_arrivals'].append(evidence('kitchen'))
                result=cmd('LOAD','kitchen');load_sent=True
                if not result.get('ok'):results['failure']=result;return results
            if state=='WAIT_HANDOVER' and not complete_sent:
                results['table_arrivals'].append(evidence(a.table))
                results['physical_table_verified']=results['table_arrivals'][-1]['pass']
                result=cmd('COMPLETE','table');complete_sent=True
                if not result.get('ok'):results['failure']=result;return results
            if state=='COMPLETED':
                results['status']='MISSION_COMPLETED';results['final_truth']=gazebo_pose(a.robot)
                import yaml
                wp=Path(__file__).resolve().parent.parent/'ros_ws/src/cafe_autonomy/config/waypoints.yaml'
                dest=yaml.safe_load(wp.read_text())['destinations'][a.robot+'_HOME']
                if results['final_truth']:
                    x,y,_=results['final_truth']
                    err=math.hypot(x-dest['x'],y-dest['y'])
                    results['home_error_m']=round(err,6)
                    results['physical_10cm_verified']=err<=xy_max
                    yawerr=math.atan2(math.sin(results['final_truth'][2]-dest['yaw']),math.cos(results['final_truth'][2]-dest['yaw']))
                    results['home_yaw_error_rad']=round(abs(yawerr),6)
                    results['physical_10cm_verified']=bool(err<=xy_max and abs(yawerr)<=yaw_max)
                return results
        results['failure']='Timeout before COMPLETED';return results
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    r=trial()
    out=Path(__file__).resolve().parent.parent/'results';out.mkdir(exist_ok=True)
    f=out/f"trial_{r['robot']}_{int(time.time())}.json"
    f.write_text(json.dumps(r,ensure_ascii=False,indent=2))
    print('RESULT',json.dumps(r,ensure_ascii=False,indent=2),'\nSAVED',f)
    sys.exit(0 if r['status']=='MISSION_COMPLETED' and (sys.argv.count('--mission-only') or (r['physical_10cm_verified'] and r['physical_table_verified'] and all(v['pass'] for v in r['pickup_arrivals']))) else 1)
