"""One fresh-spawn two-robot serving trial; local-only evidence."""
import csv, json, math, subprocess, time, urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'_local_only/wheel_0317_trial/deadlock_top_result_01.json'
if OUT.exists(): raise SystemExit('Refusing to overwrite')
if (ROOT/'_local_only/runtime.json').exists(): raise SystemExit('Runtime active')
URL='http://127.0.0.1:8765/api/'
def api(path,payload=None):
    req=urllib.request.Request(URL+path, None if payload is None else json.dumps(payload).encode(), {'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as response: return json.load(response)
def status(): return api('status')['supervisors']
trials={'robot_01':{'table':'table_02'},'robot_02':{'table':'table_01'}}
launch_time=time.time()
launcher=subprocess.Popen(['bash','scripts/start_operation.sh','--headless'],cwd=ROOT)
try:
    ready=False
    for _ in range(180):
        if launcher.poll() is not None: break
        try:
            s=status()
            if all(s.get(r,{}).get('nav_ready') for r in trials): ready=True;break
        except Exception: pass
        time.sleep(.5)
    if not ready: raise RuntimeError('Both Nav2 instances not ready')
    for robot,trial in trials.items():
        rid='deadlocktop-'+robot+'-'+str(int(time.time()))
        trial['request_id']=rid
        trial['submit']=api('request',{'action':'SUBMIT','request_id':rid,'robot_id':robot,'table':trial['table'],'mode':'DELIVER_DIRECT','side':'top'})
    end=time.monotonic()+360
    while time.monotonic()<end:
        s=status()
        for robot,trial in trials.items():
            c=s.get(robot,{})
            state=c.get('state')
            if c.get('request_id')!=trial['request_id']: continue
            if state=='WAIT_HANDOVER' and not trial.get('handover'):
                trial['handover']={'target_pose':c.get('target_pose'),'pose_error':c.get('pose_error'),'wall_time':time.time()}
                trial['complete']=api('request',{'action':'COMPLETE','request_id':trial['request_id'],'source':'table'})
            if state in ('COMPLETED','ERROR','CANCELED'):
                trial['final_state']=state
                trial['final_pose_error']=c.get('pose_error')
                trial['reason']=c.get('reason')
        if all('final_state' in trial for trial in trials.values()):break
        time.sleep(.5)
    files=[p for p in (ROOT/'_local_only/metrics').glob('measure_*.csv') if p.stat().st_mtime>=launch_time]
    if files:
        metric=max(files,key=lambda p:p.stat().st_mtime)
        for robot,trial in trials.items():
            trial['metrics']=str(metric)
            rows=[r for r in csv.DictReader(metric.open()) if r['robot']==robot and r['gt_x'] and r['gt_y'] and r['mission_state']=='WAIT_HANDOVER']
            target=trial.get('handover',{}).get('target_pose')
            if rows and target:
                first=rows[0]
                trial['actual_error_cm']=100*math.hypot(float(first['gt_x'])-target[0],float(first['gt_y'])-target[1])
                trial['tf_gt_cm']=100*float(first['xy_error_tf_gt']) if first['xy_error_tf_gt'] else None
    OUT.write_text(json.dumps(trials,indent=2))
    print(json.dumps(trials),flush=True)
finally:
    subprocess.run(['bash','scripts/stop.sh'],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
    try:launcher.wait(timeout=10)
    except subprocess.TimeoutExpired: pass
