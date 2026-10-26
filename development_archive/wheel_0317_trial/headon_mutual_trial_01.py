"""Controlled face-to-face Gazebo trial; no source or configuration edits."""
import csv, json, math, os, subprocess, time, urllib.request
from pathlib import Path
ROOT=Path("/home/user/team_serving_robot_clean_20261022")
BASE=ROOT/"_local_only/wheel_0317_trial"
OUT=BASE/"headon_mutual_result_01.json"
TRACE=BASE/"headon_mutual_trace_01.jsonl"
if OUT.exists() or TRACE.exists():
    raise SystemExit("Refusing overwrite")
if (ROOT/"_local_only/runtime.json").exists():
    raise SystemExit("Runtime active")
URL="http://127.0.0.1:8765/api/"
def api(path,payload=None):
    req=urllib.request.Request(URL+path,None if payload is None else json.dumps(payload).encode(),{"Content-Type":"application/json"})
    with urllib.request.urlopen(req,timeout=5) as response:
        return json.load(response)
def status():
    return api("status")
def set_pose(robot,mx,my,yaw):
    wx,wy=mx-3.2,my-2.25
    req=f'name:"{robot}" position{{x:{wx} y:{wy} z:0.02}} orientation{{z:{math.sin(yaw/2)} w:{math.cos(yaw/2)}}}'
    env=os.environ.copy()
    env.setdefault("GZ_PARTITION","team_integration_101")
    p=subprocess.run(["gz","service","-s","/world/cafe/set_pose","--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean","--timeout","5000","--req",req],capture_output=True,text=True,env=env,timeout=10)
    if p.returncode or "data: true" not in p.stdout:
        raise RuntimeError("Gazebo set_pose failed: "+p.stdout+p.stderr)
def trace_one(trace,phase):
    x=status()
    trace.write(json.dumps({"phase":phase,"wall_time":time.time(),"poses":x.get("robot_poses"),"traffic":x.get("traffic_hold"),"zones":x.get("zone_owner"),"recovery":x.get("recovery"),"supervisors":{r:{"state":v.get("state"),"request_id":v.get("request_id"),"pose_error":v.get("pose_error"),"reason":v.get("reason"),"target_pose":v.get("target_pose")} for r,v in x.get("supervisors",{}).items()}})+"\n")
    trace.flush()
    return x
result={"started":time.time()}
launcher=subprocess.Popen(["bash","scripts/start_operation.sh","--headless"],cwd=ROOT)
try:
    ready=False
    for _ in range(180):
        if launcher.poll() is not None:
            break
        try:
            s=status()["supervisors"]
            if all(s.get(r,{}).get("nav_ready") for r in ("robot_01","robot_02")):
                ready=True
                break
        except Exception:
            pass
        time.sleep(.5)
    if not ready:
        raise RuntimeError("Both Nav2 instances not ready")
    trials={"robot_01":{"table":"table_02"},"robot_02":{"table":"table_01"}}
    for robot,trial in trials.items():
        rid="deadlockfacing-"+robot+"-"+str(int(time.time()))
        trial["request_id"]=rid
        trial["submit"]=api("request",{"action":"SUBMIT","request_id":rid,"robot_id":robot,"table":trial["table"],"mode":"DELIVER_DIRECT","side":"top"})
    with TRACE.open("x") as trace:
        active=False
        for _ in range(80):
            x=trace_one(trace,"approach")
            s=x.get("supervisors",{})
            if all(s.get(r,{}).get("state")=="GOING_DIRECT" and s[r].get("request_id")==trials[r]["request_id"] for r in trials):
                active=True
                break
            time.sleep(.5)
        if not active:
            raise RuntimeError("Both missions did not become active")
        set_pose("robot_01",2.32,3.85,0.)
        set_pose("robot_02",3.04,3.85,math.pi)
        time.sleep(2)
        x=trace_one(trace,"placed")
        poses=x.get("robot_poses",{})
        expected={"robot_01":(2.32,3.85),"robot_02":(3.04,3.85)}
        for r,target in expected.items():
            actual=poses.get(r)
            if not actual or math.dist(actual,target)>.25:
                raise RuntimeError("TF did not follow Gazebo placement: "+str((r,actual,target)))
        result["placed_at"]=time.time()
        result["placed_poses"]=poses
        end=time.monotonic()+360
        while time.monotonic()<end:
            x=trace_one(trace,"facing")
            s=x.get("supervisors",{})
            for robot,trial in trials.items():
                c=s.get(robot,{})
                state=c.get("state")
                if c.get("request_id")!=trial["request_id"]:
                    continue
                if state=="WAIT_HANDOVER" and not trial.get("handover"):
                    trial["handover"]={"target_pose":c.get("target_pose"),"pose_error":c.get("pose_error"),"wall_time":time.time()}
                    trial["complete"]=api("request",{"action":"COMPLETE","request_id":trial["request_id"],"source":"table"})
                if state in ("COMPLETED","ERROR","CANCELED"):
                    trial["final_state"]=state
                    trial["final_pose_error"]=c.get("pose_error")
                    trial["reason"]=c.get("reason")
            if all("final_state" in t for t in trials.values()):
                break
            time.sleep(.5)
    files=[p for p in (ROOT/"_local_only/metrics").glob("measure_*.csv") if p.stat().st_mtime>=result["started"]]
    if files:
        metric=max(files,key=lambda p:p.stat().st_mtime)
        result["metrics"]=str(metric)
        rows=list(csv.DictReader(metric.open()))
        for robot,trial in trials.items():
            target=trial.get("handover",{}).get("target_pose")
            stamp=trial.get("handover",{}).get("wall_time",0)
            candidates=[r for r in rows if r["robot"]==robot and r["gt_x"] and r["gt_y"] and r["mission_state"]=="WAIT_HANDOVER" and abs(float(r["wall_time"])-stamp)<3]
            if target and candidates:
                row=min(candidates,key=lambda r:abs(float(r["wall_time"])-stamp))
                trial["actual_error_cm"]=100*math.hypot(float(row["gt_x"])-target[0],float(row["gt_y"])-target[1])
    result["trials"]=trials
    OUT.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
except Exception as exc:
    result["error"]=str(exc)
    result["trials"]=locals().get("trials")
    OUT.write_text(json.dumps(result,indent=2))
    raise
finally:
    subprocess.run(["bash","scripts/stop.sh"],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
    try:
        launcher.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
