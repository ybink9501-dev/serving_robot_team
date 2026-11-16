"""Run real Fleet/Nav2 missions; only simulate the two operator confirmations.

KITCHEN_PICKUP and DELIVER_DIRECT complete after handover without automatic HOME.
GO_HOME can also be verified independently with trial_go_home.py.
"""
import argparse,json,time,urllib.request,uuid
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--mode',default='DELIVER_DIRECT',choices=['DELIVER_DIRECT','KITCHEN_PICKUP','TABLE_COLLECT']);ap.add_argument('--timeout',type=float,default=300);ap.add_argument('--port',type=int,default=8765);ap.add_argument('--cross',action='store_true');ap.add_argument('--north',action='store_true');ap.add_argument('--tables',nargs=2,help='Two destination names from the selected site');ap.add_argument('--wait-home',action='store_true');args=ap.parse_args()
root=Path(__file__).resolve().parents[1];out=root/'_local_only';out.mkdir(exist_ok=True)
def get(path):
    with urllib.request.urlopen(f'http://127.0.0.1:{args.port}'+path,timeout=4) as res:return json.load(res)
def post(data):
    req=urllib.request.Request(f'http://127.0.0.1:{args.port}/api/request',json.dumps(data).encode(),{'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=4) as res:reply=json.load(res)
    if not reply.get('ok'):raise RuntimeError(reply)
    return reply
ids={r:'trial-'+uuid.uuid4().hex[:12] for r in ('robot_01','robot_02')}
log=out/('serving_trial_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl');terminal={};confirmed=set();start=time.monotonic();previous={}
with log.open('w') as f:
    for r,table in (list(zip(('robot_01','robot_02'),args.tables)) if args.tables else [('robot_01','table_01'),('robot_02','table_02')] if args.north else [('robot_01','table_04'),('robot_02','table_03')] if args.cross else [('robot_01','table_03'),('robot_02','table_04')]):
        print(post({'action':'SUBMIT','request_id':ids[r],'robot_id':r,'table':table,'mode':args.mode}),flush=True)
    while time.monotonic()-start<args.timeout:
        status=get('/api/status');f.write(json.dumps({'elapsed_s':time.monotonic()-start,'fleet':status})+'\n');f.flush()
        for r,s in status.get('supervisors',{}).items():
            if previous.get(r)!=s.get('state'):print(r,s,flush=True);previous[r]=s.get('state')
            if s.get('request_id')!=ids[r]:continue
            st=s.get('state')
            if st in ('WAIT_LOAD','WAIT_HANDOVER','WAIT_COLLECTION') and (r,st) not in confirmed:
                cmd='LOAD' if st=='WAIT_LOAD' else 'COLLECTED' if st=='WAIT_COLLECTION' else 'COMPLETE'
                post({'action':cmd,'request_id':ids[r],'source':'kitchen' if cmd=='LOAD' else 'table'});confirmed.add((r,st))
            if st in ('COMPLETED','ERROR','CANCELED'):terminal[r]=s
        # Fleet persists terminal acknowledgements even if auto HOME changes
        # the supervisor request before the observer's next polling sample.
        for r,rid in ids.items():
            task=status.get('requests',{}).get(rid,{})
            if r not in terminal and task.get('state')=='SUCCEEDED':
                terminal[r]={'state':'COMPLETED','request_id':rid,'proof':'Fleet persisted supervisor completion'}
        if len(terminal)==2:break
        time.sleep(.5)
home_verified=not args.wait_home
if args.wait_home and len(terminal)==2:
    while time.monotonic()-start<args.timeout:
        status=get('/api/status');further={r:status.get('supervisors',{}).get(r,{}) for r in ids}
        if all(v.get('mode')=='GO_HOME' and v.get('state')=='COMPLETED' for v in further.values()):home_verified=True;break
        time.sleep(.5)
report={'home_verified':home_verified,'elapsed_s':time.monotonic()-start,'operator_confirmations_simulated':True,'request_ids':ids,'results':terminal,'success':home_verified and len(terminal)==2 and all(s['state']=='COMPLETED' for s in terminal.values()),'log':str(log)}
log.with_suffix('.summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['success'] else 1)
