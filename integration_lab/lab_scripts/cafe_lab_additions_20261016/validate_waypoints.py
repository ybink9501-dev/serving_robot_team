#!/usr/bin/env python3
"""Check explicitly reviewed waypoints against provisional PGM and fixture layout.
Does not alter any files or publish goals.
"""
import argparse
import json
import math
from pathlib import Path
import yaml


def load_pgm(path):
    with path.open('rb') as f:
        if f.readline().strip() != b'P5':
            raise ValueError('Only binary P5 PGM is supported')
        tokens=[]
        while len(tokens)<3:
            line=f.readline()
            if not line:
                raise ValueError('Incomplete PGM header')
            tokens.extend(line.split(b'#')[0].split())
        w,h,depth = map(int,tokens[:3])
        if depth != 255:
            raise ValueError('Expected 8-bit PGM')
        pixels=f.read()
        if len(pixels) != w*h:
            raise ValueError(f'Expected {w*h} pixels; got {len(pixels)}')
        return w,h,pixels


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--map-yaml', type=Path, required=True)
    p.add_argument('--waypoints', type=Path, required=True)
    p.add_argument('--layout-json', type=Path, required=True)
    p.add_argument('--clearance',type=float,default=0.35)
    args=p.parse_args()
    meta=yaml.safe_load(args.map_yaml.read_text())
    w,h,img=load_pgm(args.map_yaml.parent / meta['image'])
    origin=meta['origin']; r=float(meta['resolution'])
    layout=json.loads(args.layout_json.read_text())
    centers={m['name'].split('/')[-1]:m for m in layout['models']}
    obj=yaml.safe_load(args.waypoints.read_text())
    if not isinstance(obj.get('map_revision'),int):
        raise SystemExit('map_revision missing')
    problems=[]
    for waypoint in obj.get('waypoints',[]):
        eid=waypoint.get('entity_id','?'); pose=waypoint.get('pose',{})
        x=float(pose['x']); y=float(pose['y']); yaw=float(pose['yaw'])
        # Disks of radius clearance around robot center must be free on 2D map.
        clear=True
        for ixstep in range(-7,8):
            for iystep in range(-7,8):
                dx=ixstep*args.clearance/7; dy=iystep*args.clearance/7
                if dx*dx+dy*dy>args.clearance**2: continue
                ix=int(math.floor((x+dx-origin[0])/r))
                iy=int(math.floor((y+dy-origin[1])/r))
                if not(0<=ix<w and 0<=iy<h) or img[(h-1-iy)*w+ix] < 200:
                    clear=False
        model='cafe_counter' if eid=='kitchen' else 'table_set_'+eid[len('table_'):] if eid.startswith('table_') else None
        heading='UNKNOWN'
        if model in centers:
            m=centers[model]
            wanted=math.atan2(m['y']-y,m['x']-x)
            error=abs(math.atan2(math.sin(yaw-wanted),math.cos(yaw-wanted)))
            heading=f'{math.degrees(error):.1f} deg difference to fixture'
            if error>0.25: problems.append(eid+': not facing fixture (tolerance 0.25rad)')
        if not clear: problems.append(eid+': near occupied cell or outside map')
        print(f'{eid}: (x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}), footprint_clear={clear}, facing={heading}')
    if not obj.get('waypoints'):
        problems.append('No waypoints defined')
    print('STATUS:', 'REVIEW PASSED (NOT a Nav2 arrival guarantee)' if not problems else 'REVIEW FAILED')
    for e in problems: print(' -',e)
    if problems: raise SystemExit(1)


if __name__=='__main__':
    main()
