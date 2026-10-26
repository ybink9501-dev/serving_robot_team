import csv,json,math,subprocess,time,urllib.request,os
from pathlib import Path
R=Path("/home/user/team_serving_robot_clean_20261022")
O=R/"_local_only/wheel_0317_trial/contact_recovery_result_03.json"
T=R/"_local_only/wheel_0317_trial/contact_recovery_trace_03.jsonl"
if O.exists() or T.exists() or (R/"_local_only/runtime.json").exists(): raise SystemExit("Refusing overwrite or active runtime")
def api(p,v=None):
 q=urllib.request.Request("http://127.0.0.1:8765/api/"+p,None if v is None else json.dumps(v).encode(),{"Content-Type":"application/json"})
 with urllib.request.urlopen(q,timeout=5) as a:return json.load(a)
def obs(action,x=0,y=0):
 a=["python3","scripts/object_scenario.py",action,"--name","contact_recovery_probe_01"]
 if action=="spawn":a+=["--x",str(x),"--y",str(y),"--radius","0.2"]
 subprocess.run(a,cwd=R,check=True,timeout=20,env={**os.environ,"GZ_PARTITION":"team_integration_101"})
def metric(t):
 f=[p for p in (R/"_local_only/metrics").glob("measure_*.csv") if p.stat().st_mtime>=t]
 if not f:return None
 with max(f,key=lambda p:p.stat().st_mtime).open() as h:a=[x for x in csv.DictReader(h) if x["robot"]=="robot_01" and x["gt_x"]]
 return a[-1] if a else None
result={"started":time.time()};launcher=subprocess.Popen(["bash","scripts/start_operation.sh","--headless","--robot","robot_01"],cwd=R)
spawned=False;did_spawn=False
try:
 for i in range(180):
  if launcher.poll() is not None:raise RuntimeError("Operation exited")
  try:
   if api("status")["supervisors"]["robot_01"].get("nav_ready"):break
  except Exception:pass
  time.sleep(.5)
 else:raise RuntimeError("Nav2 not ready")
 rid="contactprobe-"+str(int(time.time()));result["request_id"]=rid
 result["submit"]=api("request",{"action":"SUBMIT","request_id":rid,"robot_id":"robot_01","table":"table_01","mode":"DELIVER_DIRECT","side":"bottom"})
 start=None
 with T.open("x") as tr:
  end=time.monotonic()+270
  while time.monotonic()<end:
   st=api("status");sup=st.get("supervisors",{}).get("robot_01",{});dyn=st.get("dynamic",{}).get("robot_01",{});row=metric(result["started"])
   tr.write(json.dumps({"wall_time":time.time(),"state":sup.get("state"),"dynamic":{k:dyn.get(k) for k in ("stage","contact_latched","contact_active","contact_observed_s","front_clear_m","clearance_confirmed")},"gt":[row["gt_x"],row["gt_y"]] if row else None,"cmd_safe_v":row["cmd_safe_v"] if row else None})+"\n");tr.flush()
   if row and start is None:start=row
   if row and start and not did_spawn and sup.get("request_id")==rid and sup.get("state")=="GOING_DIRECT" and math.hypot(float(row["gt_x"])-float(start["gt_x"]),float(row["gt_y"])-float(start["gt_y"]))>.35:
    result["spawn_at"]=time.time();result["spawn_gt"]=[float(row["gt_x"]),float(row["gt_y"])]
    obs("spawn",float(row["gt_x"])-3.2,float(row["gt_y"])-2.25);spawned=True;did_spawn=True
   if spawned and dyn.get("contact_latched") and "contact_seen_at" not in result:result["contact_seen_at"]=time.time()
   if spawned and time.time()-result["spawn_at"]>7:
    obs("clear");spawned=False;result["clear_at"]=time.time()
   if sup.get("request_id")==rid and sup.get("state")=="WAIT_HANDOVER":
    result["handover"]={"target_pose":sup.get("target_pose"),"pose_error":sup.get("pose_error"),"wall_time":time.time()}
    result["complete"]=api("request",{"action":"COMPLETE","request_id":rid,"source":"table"})
   if sup.get("request_id")==rid and sup.get("state") in ("COMPLETED","ERROR","CANCELED"):
    result["final_state"]=sup.get("state");result["reason"]=sup.get("reason");break
   time.sleep(.5)
 result["trace"]=str(T);O.write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 if spawned:
  try:obs("clear")
  except Exception:pass
 subprocess.run(["bash","scripts/stop.sh"],cwd=R,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
 try:launcher.wait(timeout=10)
 except subprocess.TimeoutExpired:pass
