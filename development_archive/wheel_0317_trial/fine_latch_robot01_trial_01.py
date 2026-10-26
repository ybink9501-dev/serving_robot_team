import csv,json,math,subprocess,time,urllib.request
from pathlib import Path
R=Path("/home/user/team_serving_robot_clean_20261022");B=R/"_local_only/wheel_0317_trial"
O=B/"fine_latch_robot01_result_01.json";T=B/"fine_latch_robot01_trace_01.jsonl"
if O.exists() or T.exists() or (R/"_local_only/runtime.json").exists():raise SystemExit("Refusing overwrite or active runtime")
def api(p,v=None):
 q=urllib.request.Request("http://127.0.0.1:8765/api/"+p,None if v is None else json.dumps(v).encode(),{"Content-Type":"application/json"})
 with urllib.request.urlopen(q,timeout=5) as f:return json.load(f)
result={"started":time.time()}
launch=subprocess.Popen(["bash","scripts/start_operation.sh","--headless","--robot","robot_01"],cwd=R)
try:
 for i in range(180):
  if launch.poll() is not None:raise RuntimeError("Runtime exited")
  try:
   if api("status")["supervisors"]["robot_01"].get("nav_ready"):break
  except Exception:pass
  time.sleep(.5)
 else:raise RuntimeError("Nav2 not ready")
 rid="finelatch-r01-"+str(int(time.time()));result["request_id"]=rid
 result["submit"]=api("request",{"action":"SUBMIT","request_id":rid,"robot_id":"robot_01","table":"table_03","mode":"KITCHEN_PICKUP","side":"bottom"})
 with T.open("x") as tr:
  end=time.monotonic()+330
  while time.monotonic()<end:
   s=api("status");v=s["supervisors"]["robot_01"]
   tr.write(json.dumps({"wall_time":time.time(),"state":v.get("state"),"request_id":v.get("request_id"),"pose_error":v.get("pose_error"),"target_pose":v.get("target_pose"),"reason":v.get("reason"),"safety":s.get("safety",{}).get("robot_01"),"dynamic":s.get("dynamic",{}).get("robot_01")},default=str)+"\n");tr.flush()
   if v.get("request_id")==rid:
    if v.get("state")=="WAIT_LOAD" and "load" not in result:result["load"]=api("request",{"action":"LOAD","request_id":rid,"source":"kitchen"})
    if v.get("state")=="WAIT_HANDOVER" and "handover" not in result:
     result["handover"]={"wall_time":time.time(),"target_pose":v.get("target_pose"),"pose_error":v.get("pose_error")}
     result["complete"]=api("request",{"action":"COMPLETE","request_id":rid,"source":"table"})
    if v.get("state") in ("COMPLETED","ERROR","CANCELED"):
     result["final_state"]=v["state"];result["reason"]=v.get("reason");break
   time.sleep(.5)
 files=[p for p in (R/"_local_only/metrics").glob("measure_*.csv") if p.stat().st_mtime>=result["started"]]
 if files:
  m=max(files,key=lambda p:p.stat().st_mtime);result["metrics"]=str(m)
  if "handover" in result:
   rows=[x for x in csv.DictReader(open(m)) if x["robot"]=="robot_01" and x["gt_x"] and x["mission_state"]=="WAIT_HANDOVER"]
   if rows:
    x=min(rows,key=lambda z:abs(float(z["wall_time"])-result["handover"]["wall_time"]));g=result["handover"]["target_pose"]
    result["actual_error_cm"]=100*math.hypot(float(x["gt_x"])-g[0],float(x["gt_y"])-g[1])
 O.write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 subprocess.run(["bash","scripts/stop.sh"],cwd=R,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
 try:launch.wait(timeout=10)
 except subprocess.TimeoutExpired:pass
