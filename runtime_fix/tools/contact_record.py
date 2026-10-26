#!/usr/bin/env python3
"""Gazebo physical body-contact evidence. Monitors Gazebo contact sensors (not safety decisions)."""
import argparse,subprocess,time,datetime,json,sys,select
from pathlib import Path

def main():
    a=argparse.ArgumentParser();a.add_argument('--seconds',type=float,default=180)
    a.add_argument('--robot',choices=['robot_01','robot_02'],default='robot_01')
    ns=a.parse_args()
    out=Path(__file__).resolve().parent.parent/'results';out.mkdir(exist_ok=True)
    path=out/f'contacts_{ns.robot}_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl'
    topic=f'/{ns.robot}/contact_truth'
    info=subprocess.run(['gz','topic','-i','-t',topic],capture_output=True,text=True,timeout=6)
    if info.returncode or 'Publisher' not in info.stdout:
        raise SystemExit('Contact sensor topic has no publisher: '+topic)
    proc=subprocess.Popen(['gz','topic','-e','-t',topic],stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                          text=True,bufsize=1)
    count=0;deadline=time.monotonic()+ns.seconds
    try:
        with path.open('w') as f:
            while time.monotonic()<deadline:
                available,_,_=select.select([proc.stdout],[],[],min(0.2,max(0,deadline-time.monotonic())))
                if not available:
                    if proc.poll() is not None:break
                    continue
                line=proc.stdout.readline()
                if not line:break
                if 'collision1:' in line:
                    count+=1
                    record={'wall_time':time.time(),'robot':ns.robot,'contact_evidence':line.strip()}
                    f.write(json.dumps(record)+'\n');f.flush()
    finally:
        proc.terminate()
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:proc.kill()
    print(json.dumps({'topic':topic,'events':count,'file':str(path)},ensure_ascii=False))
    if count:sys.exit(1)

if __name__=='__main__':main()
