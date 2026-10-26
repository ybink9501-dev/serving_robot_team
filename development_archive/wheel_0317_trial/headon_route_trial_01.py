"""Two-stage top-lane Gazebo deadlock trial. Local-only evidence."""
import csv, json, math, subprocess, time, urllib.request
from pathlib import Path
ROOT=Path("/home/user/team_serving_robot_clean_20261022")
BASE=ROOT/"_local_only/wheel_0317_trial"
OUT=BASE/"headon_route_result_01.json"
TRACE=BASE/"headon_route_trace_01.jsonl"
if OUT.exists() or TRACE.exists():
    raise SystemExit("Refusing overwrite")
if (ROOT/"_local_only/runtime.json").exists():
    raise SystemExit("Runtime active")
URL="http://127.0.0.1:8765/api/"
def api(path,payload=None):
    req=urllib.request.Request(URL+path,None if payload is None else json.dumps(payload).encode(),{"Content-Type":"application/json"})
    with urllib.request.urlopen(req,timeout=5) as response:
        return json.load(response)
def snapshot():
    return api("status")
def wait_ready(launcher):
    for _ in range(180):
        if launcher.poll() is not None:
            break
        try:
            s=snapshot()["supervisors"]
            if all(s.get(r,{}).get("nav_ready") for r in ("robot_01","robot_02")):
                return
        except Exception:
            pass
        time.sleep(.5)
    raise RuntimeError("Both Nav2 instances not ready")
def phase(name,targets,limit,trace,side):
    phase_result={}
    for robot,table in targets.items():
        rid=name+"-"+robot+"-"+str(int(time.time()))
        phase_result[robot]={"table":table,"request_id":rid}
        phase_result[robot]["submit"]=api("request",{"action":"SUBMIT","request_id":rid,"robot_id":robot,"table":table,"mode":"DELIVER_DIRECT","side":side})
    end=time.monotonic()+limit
    while time.monotonic()<end:
        s=snapshot()
        trace.write(json.dumps({"phase":name,"wall_time":time.time(),"poses":s.get("robot_poses"),"traffic":s.get("traffic_hold"),"zones":s.get("zone_owner"),"recovery":s.get("recovery"),"supervisors":{r:{"state":v.get("state"),"request_id":v.get("request_id"),"pose_error":v.get("pose_error"),"reason":v.get("reason"),"target_pose":v.get("target_pose")} for r,v in s.get("supervisors",{}).items()}})+"\n")
        trace.flush()
        for robot,trial in phase_result.items():
            c=s.get("supervisors",{}).get(robot,{})
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
        if all("final_state" in t for t in phase_result.values()):
            break
        time.sleep(.5)
    return phase_result
result={"started":time.time()}
launcher=subprocess.Popen(["bash","scripts/start_operation.sh","--headless"],cwd=ROOT)
try:
    wait_ready(launcher)
    with TRACE.open("x") as trace:
        result["stage"]=phase("deadlockstage",{"robot_01":"table_01","robot_02":"table_02"},260,trace,"top")
        if all(t.get("final_state")=="COMPLETED" for t in result["stage"].values()):
            result["swap_started"]=time.time()
            result["swap"]=phase("deadlockswap",{"robot_01":"table_04","robot_02":"table_03"},300,trace,"bottom")
    files=[p for p in (ROOT/"_local_only/metrics").glob("measure_*.csv") if p.stat().st_mtime>=result["started"]]
    if files:
        metric=max(files,key=lambda p:p.stat().st_mtime)
        rows=list(csv.DictReader(metric.open()))
        result["metrics"]=str(metric)
        for phase_name in ("stage","swap"):
            for robot,trial in result.get(phase_name,{}).items():
                target=trial.get("handover",{}).get("target_pose")
                stamp=trial.get("handover",{}).get("wall_time",0)
                candidates=[r for r in rows if r["robot"]==robot and r["gt_x"] and r["gt_y"] and r["mission_state"]=="WAIT_HANDOVER" and abs(float(r["wall_time"])-stamp)<3]
                if target and candidates:
                    row=min(candidates,key=lambda r:abs(float(r["wall_time"])-stamp))
                    trial["actual_error_cm"]=100*math.hypot(float(row["gt_x"])-target[0],float(row["gt_y"])-target[1])
    OUT.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:
    subprocess.run(["bash","scripts/stop.sh"],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
    try:
        launcher.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
