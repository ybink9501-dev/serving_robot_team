"""Existing-goal top aisle face-to-face attempt; local-only evidence."""
import csv,json,math,subprocess,time,urllib.request
from pathlib import Path
R=Path("/home/user/team_serving_robot_clean_20261022")
B=R/"_local_only/wheel_0317_trial"
O=B/"headon_stagger_result_02.json";T=B/"headon_stagger_trace_02.jsonl"
if O.exists() or T.exists() or (R/"_local_only/runtime.json").exists():raise SystemExit("Refusing overwrite or active runtime")
def api(p,v=None):
 q=urllib.request.Request("http://127.0.0.1:8765/api/"+p,None if v is None else json.dumps(v).encode(),{"Content-Type":"application/json"})
 with urllib.request.urlopen(q,timeout=5) as f:return json.load(f)
def snap():return api("status")
def submit(robot,table,mode,side,tag):
 rid=tag+"-"+robot+"-"+str(int(time.time()))
 return {"request_id":rid,"table":table,"submit":api("request",{"action":"SUBMIT","request_id":rid,"robot_id":robot,"table":table,"mode":mode,"side":side})}
def record(tr,phase):
 x=snap()
 tr.write(json.dumps({"phase":phase,"wall_time":time.time(),"poses":x.get("robot_poses"),"traffic":x.get("traffic_hold"),"recovery":x.get("recovery"),"supervisors":{r:{"state":v.get("state"),"request_id":v.get("request_id"),"target_pose":v.get("target_pose"),"pose_error":v.get("pose_error"),"reason":v.get("reason")} for r,v in x.get("supervisors",{}).items()} })+"\n");tr.flush()
 return x
def update(x,t):
 for r,v in t.items():
  s=x["supervisors"].get(r,{})
  if s.get("request_id")!=v["request_id"]:continue
  if s.get("state")=="WAIT_LOAD" and "load" not in v:
   v["load"]=api("request",{"action":"LOAD","request_id":v["request_id"],"source":"kitchen"})
  if s.get("state")=="WAIT_HANDOVER" and "handover" not in v:
   v["handover"]={"wall_time":time.time(),"target_pose":s.get("target_pose")}
   v["complete"]=api("request",{"action":"COMPLETE","request_id":v["request_id"],"source":"table"})
  if s.get("state") in ("COMPLETED","ERROR","CANCELED"):
   v["final_state"]=s["state"];v["reason"]=s.get("reason")
result={"started":time.time()}
launch=subprocess.Popen(["bash","scripts/start_operation.sh","--headless"],cwd=R)
try:
 for i in range(180):
  if launch.poll() is not None:raise RuntimeError("Runtime exited")
  try:
   if all(snap()["supervisors"][r].get("nav_ready") for r in ("robot_01","robot_02")):break
  except Exception:pass
  time.sleep(.5)
 else:raise RuntimeError("Nav2 not ready")
 with T.open("x") as tr:
  stage={"robot_01":submit("robot_01","table_01","DELIVER_DIRECT","top","headonstage"),"robot_02":submit("robot_02","table_02","DELIVER_DIRECT","top","headonstage")}
  result["stage"]=stage;end=time.monotonic()+250
  while time.monotonic()<end:
   x=record(tr,"stage");update(x,stage)
   if all("final_state" in v for v in stage.values()):break
   time.sleep(.5)
  if not all(v.get("final_state")=="COMPLETED" for v in stage.values()):raise RuntimeError("Stage incomplete")
  active={"robot_02":submit("robot_02","table_03","KITCHEN_PICKUP","bottom","headonserve")}
  result["active"]=active
  end=time.monotonic()+90;trigger=False
  while time.monotonic()<end:
   x=record(tr,"approach");update(x,active)
   p=x.get("robot_poses",{}).get("robot_02")
   if p and p[0]<3.9 and p[1]>2.8 and x["supervisors"]["robot_02"].get("state")=="GOING_PICKUP":
    result["trigger_pose"]=p;result["trigger_time"]=time.time();trigger=True;break
   time.sleep(.5)
  if trigger:active["robot_01"]=submit("robot_01","table_02","DELIVER_DIRECT","top","headonmove")
  else:result["trigger_error"]="Robot 02 did not traverse top aisle westbound"
  end=time.monotonic()+330
  while time.monotonic()<end:
   x=record(tr,"encounter");update(x,active)
   if all("final_state" in v for v in active.values()):break
   time.sleep(.5)
 result["trace"]=str(T)
 f=[p for p in (R/"_local_only/metrics").glob("measure_*.csv") if p.stat().st_mtime>=result["started"]]
 if f:result["metrics"]=str(max(f,key=lambda p:p.stat().st_mtime))
 O.write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
except Exception as e:
 result["error"]=str(e);O.write_text(json.dumps(result,indent=2));raise
finally:
 subprocess.run(["bash","scripts/stop.sh"],cwd=R,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=45,check=False)
 try:launch.wait(timeout=10)
 except subprocess.TimeoutExpired:pass
