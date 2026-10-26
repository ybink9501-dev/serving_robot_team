#!/usr/bin/env python3
"""Bounded, unattended, read-only ROS-free test and stress report.

No Gazebo/Nav2/ROS subscription/publisher; no writes outside the report folder.
Does not edit source or claim to discover an optimal controller automatically.
"""
import argparse
import importlib.util
import json
import math
import random
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

HOME = Path.home()
ROOT = HOME / 'team_robot_lab'
REPORT_ROOT = ROOT / 'robot01_diagnostics_v9' / 'reports'
SUITES = [
    ('v6_preflight', ROOT/'robot01_pretest_v6/tests', 'unittest'),
    ('v7_spin', ROOT/'robot01_spin_v7/tests', 'unittest'),
    ('v8_localization_geometry', ROOT/'robot01_localization_v8/tests', 'unittest'),
    ('v3_waypoint_snapshot', ROOT/'serving_v3/tests', 'pytest'),
]

def fuzz_spin(seed, cases=400):
    # Deliberately only exercises the Python ROS-free SpinWindow policy.
    p = ROOT/'robot01_spin_v7/policy.py'
    spec = importlib.util.spec_from_file_location('v7_policy_nightly',str(p))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    rng = random.Random(seed)
    failures=[]
    for i in range(cases):
        kind=i%3
        policy=mod.SpinWindow(12,.08,math.pi)
        rot = rng.uniform(3.65,5.3) if kind<2 else rng.uniform(.1,2.0)
        dx = 0 if kind==0 or kind==2 else rng.uniform(.2,.5)
        x0,y0 = rng.uniform(-20,20),rng.uniform(-20,20)
        value=None; fired=False
        for j in range(121):
            value=policy.observe(j*.1,x0+dx*j/120,y0,rot*j/120)
            fired = fired or value['triggered']
        expected = kind==0
        if fired != expected:
            failures.append({'case':i,'kind':kind,'rotation':rot,'dx':dx,
                             'expected_trigger':expected,'actual_trigger':fired})
    return {'seed':seed,'cases':cases,'failures':failures[:20],
            'failure_count':len(failures)}

def summarize_logs():
    paths=sorted(Path('/tmp').glob('robot01_spin_arrival_v7_*.jsonl'))[-3:]
    paths+=sorted(Path('/tmp').glob('robot01_v6_*.jsonl'))[-2:]
    results=[]
    for path in paths:
        counts=Counter(); states=Counter(); total=0
        try:
            with path.open(encoding='utf8') as f:
                for line in f:
                    total+=1
                    try: obj=json.loads(line)
                    except ValueError:continue
                    event=obj.get('event') or obj.get('channel') or 'unknown'
                    counts[event]+=1
                    if obj.get('event')=='STATE_TRANSITION':
                        states[obj.get('state','?')]+=1
        except OSError:
            continue
        results.append({'file':str(path),'lines':total,
                        'events':dict(counts),'transition_states':dict(states)})
    return results

def run_once(i, out):
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    cycle={'round':i+1,'timestamp_local':stamp,'suites':{},'fuzz':{},'logs':[]}
    for name,path,mode in SUITES:
        if not path.exists():
            cycle['suites'][name]={'status':'MISSING'}
            continue
        cmd=([sys.executable,'-m','unittest','discover','-s',str(path),'-v']
             if mode=='unittest' else [sys.executable,'-m','pytest','-q',str(path)])
        try:
            proc=subprocess.run(cmd, capture_output=True, text=True, timeout=180)
            output=(proc.stdout or '')+'\n'+(proc.stderr or '')
            (out/f'{stamp}_{name}.txt').write_text(output,encoding='utf8')
            cycle['suites'][name]={'status':'PASS' if proc.returncode==0 else 'FAIL',
                                   'return_code':proc.returncode,
                                   'log':str(out/f'{stamp}_{name}.txt')}
        except subprocess.TimeoutExpired:
            cycle['suites'][name]={'status':'TIMEOUT'}
        except Exception as exc:
            cycle['suites'][name]={'status':'ERROR','error':str(exc)}
    try:
        cycle['fuzz']=fuzz_spin(i*1277+20261018)
    except Exception as exc:
        cycle['fuzz']={'error':str(exc),'failure_count':1}
    cycle['logs']=summarize_logs()
    path=out/f'{stamp}_round{i+1:02d}.json'
    path.write_text(json.dumps(cycle,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    return cycle

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--rounds',type=int,default=1)
    a.add_argument('--interval-seconds',type=float,default=0)
    a.add_argument('--report-dir',default='')
    arg=a.parse_args()
    if not 1<=arg.rounds<=48:raise SystemExit('--rounds must be 1..48')
    if not 0<=arg.interval_seconds<=3600:raise SystemExit('--interval-seconds must be 0..3600')
    out=Path(arg.report_dir).expanduser() if arg.report_dir else REPORT_ROOT / datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=True)
    summary=[]
    print('OFFLINE ONLY. No ROS goals, cmd_vel, initialpose, restarts, or file changes.',flush=True)
    for idx in range(arg.rounds):
        cycle=run_once(idx,out)
        summary.append({'round':idx+1,'suites':{k:v['status'] for k,v in cycle['suites'].items()},
                        'fuzz_failures':cycle['fuzz'].get('failure_count',-1)})
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        print(json.dumps(summary[-1],ensure_ascii=False),flush=True)
        if idx+1<arg.rounds:time.sleep(arg.interval_seconds)
    print('REPORT_DIR:',out,flush=True)
if __name__=='__main__':main()
