#!/usr/bin/env python3
"""Summarize existing V6 JSONL commands and events. No ROS required; read-only."""
import argparse,collections,datetime,json,math
from pathlib import Path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('log',nargs='?',help='Existing recorder JSONL path')
    args=ap.parse_args()
    if args.log:path=Path(args.log)
    else:
        found=sorted(Path('/tmp').glob('robot01_v6_*.jsonl'),key=lambda p:p.stat().st_mtime)
        if not found:raise SystemExit('No /tmp/robot01_v6_*.jsonl found')
        path=found[-1]
    states=[];nav=[];warnings=[];velocities=collections.defaultdict(list);odoms=[]
    previous=None;count=0
    for line in path.open(encoding='utf8'):
        try:r=json.loads(line);d=r.get('data',{});kind=r.get('channel');count+=1
        except (ValueError,TypeError):continue
        if kind=='odom':odoms.append(r)
        elif kind in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel_semantic','cmd_vel'):
            velocities[kind].append((r.get('utc'),d.get('vx'),d.get('wz')))
        elif kind=='nav_event':nav.append(r)
        elif kind=='serving_status':
            key=(d.get('state'),d.get('reason'),d.get('destination_id'))
            if key!=previous:states.append(r);previous=key
        elif kind=='WARNING_EXCESSIVE_ROTATION':warnings.append(r)
    print('FILE:',path,'ROWS:',count)
    print('\nSTATE TIMELINE (UTC):')
    for r in states:
        d=r['data'];print(r.get('utc','')[:19],d.get('state'),d.get('destination_id'),d.get('reason'))
    print('\nNAV EVENTS (UTC):')
    for r in nav:print(r.get('utc','')[:19],json.dumps(r['data'],ensure_ascii=False))
    print('\nSPIN WARNINGS:')
    for r in warnings:print(r.get('utc','')[:19],json.dumps(r['data'],ensure_ascii=False))
    print('\nVELOCITY COMMAND DISTRIBUTION (observed messages):')
    for name,samples in velocities.items():
        if not samples:continue
        print(name,'count=',len(samples),'peak_abs_linear=',max(abs(v or 0) for _,v,w in samples),
              'peak_abs_angular=',max(abs(w or 0) for _,v,w in samples),
              'last=',samples[-1])
    if odoms:
        a=odoms[0]['data'];b=odoms[-1]['data']
        print('\nODOM FRAME ONLY: first=',(a['x'],a['y']),'last=',(b['x'],b['y']),
              'last_twist=',(b['vx'],b['wz']))
        print('CAUTION: odom x/y are NOT map coordinates. Do not subtract from a map waypoint.')
    print('\nMissing from the old V6 log: map→base_footprint error and target yaw error.')
    print('V7 monitor records these MAP-frame errors during a future guarded run.')

if __name__=='__main__':main()
