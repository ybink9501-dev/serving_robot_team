#!/usr/bin/env python3
"""Stage 1/2: independent offline pickup proof + existing V8 AMCL check / guarded recovery.
Never modifies shared/team code or guesses the final kitchen handoff location.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import yaml
from policies import inspect_pickup

ROOT=Path(__file__).resolve().parent

def expand(p):return Path(str(p)).expanduser().resolve()

def inspect(config,static_only=False):
    cfg=yaml.safe_load(config.read_text(encoding='utf-8'))
    if cfg['robot_id']!='robot_01' or cfg['ros_domain_id']!=99:raise ValueError('Invalid robot or ROS domain in config')
    v3=expand(cfg['v3_dir'])
    layout=yaml.safe_load((v3/'layout.yaml').read_text(encoding='utf-8'))
    manifest=json.loads((v3/'generated/manifest.json').read_text(encoding='utf-8'))
    wps=yaml.safe_load((v3/'generated/active_waypoints.yaml').read_text(encoding='utf-8'))
    if layout.get('robot_id')!='robot_01':raise ValueError('Wrong V3 robot ID')
    if layout['map_revision']!=wps['map_revision'] or layout['map_revision']!=manifest['map_revision']:
        raise ValueError('Map revision mismatch')
    if layout['frame_alignment']['mode']!='sdf_aligned_lab_only' or any(
       abs(float(layout['frame_alignment']['world_to_map'][k]))>1e-8 for k in ('x','y','yaw')):
        raise ValueError('SDF world != map: do not compare pickup directly')
    import hashlib
    for p,k in [(v3/'layout.yaml','layout_sha256'),(v3/'generated/active_waypoints.yaml','waypoints_sha256'),
                (expand(manifest['world']),'world_sha256'),(expand(manifest['map']),'map_yaml_sha256'),
                (expand(manifest['map_raster']),'map_raster_sha256')]:
        if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=manifest[k]:
            raise ValueError(f'Snapshot integrity mismatch: {p}')
    goal=next((w['pose'] for w in wps['waypoints'] if w['entity_id']=='counter_pickup'),None)
    if not goal:raise ValueError('V3 counter_pickup destination missing')
    # This only proves static clearance with the model collision box and robot radius.
    pick=inspect_pickup(expand(manifest['world']),layout['geometry']['kitchen_model'],goal,
                       float(layout['geometry']['robot_clearance_radius_m']),
                       float(cfg['pickup']['minimum_body_to_counter_clearance_m']),
                       float(cfg['pickup']['maximum_face_direction_error_deg']))
    if not pick['pass_static']:raise ValueError('Pickup static geometry unsuitable; DO NOT issue GO')
    return {'stage_1':'PASS_STATIC','map_revision':layout['map_revision'],
            'pickup_static':pick,'static_only':static_only,
            'remaining_approval':'Final team pickup/service location not approved; location remains a LAB candidate'}

def invoke_existing_v8(cfg,apply):
    v8=expand(cfg['v8_dir'])/'relocalize.py'
    if not v8.is_file():raise ValueError('V8 recovery script missing')
    src=v8.read_text(encoding='utf-8')
    if "'/robot_01/initialpose'" not in src:
        raise ValueError('Current V8 is missing the verified robot_01 initialpose patch')
    if os.environ.get('ROS_DOMAIN_ID')!='99' or os.environ.get('GZ_PARTITION')!='team_robot_lab_gazebo_99':
        raise ValueError('Expected isolated LAB ROS_DOMAIN_ID=99/GZ_PARTITION=team_robot_lab_gazebo_99')
    command=[sys.executable,str(v8)]+(['--apply'] if apply else [])
    r=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=55)
    observed={}
    for line in r.stdout.splitlines():
        if line.startswith('BEFORE:'):
            try:observed['before']=json.loads(line.split('BEFORE:',1)[1].strip())
            except ValueError:pass
        elif line.startswith('AFTER:'):
            try:observed['after']=json.loads(line.split('AFTER:',1)[1].strip())
            except ValueError:pass
    if apply:
        aligned=(('PASS: AMCL pose stable' in r.stdout and 'after' in observed) or
                 ('ALREADY_ALIGNED' in r.stdout and 'before' in observed and
                  observed['before']['position_error_m']<=.10 and observed['before']['yaw_error_deg']<=10))
    else:
        aligned=('before' in observed and observed['before']['position_error_m']<=.15 and
                 observed['before']['yaw_error_deg']<=12)
    return {'command':'V8 guarded apply' if apply else 'V8 read-only','exit_code':r.returncode,
            'aligned':aligned,'measurements':observed,
            'stdout':r.stdout.strip(),'stderr':r.stderr[-600:].strip()}

def main():
    p=argparse.ArgumentParser()
    mode=p.add_mutually_exclusive_group(required=False)
    mode.add_argument('--live-readonly',action='store_true')
    mode.add_argument('--recover',action='store_true',help='EXPLICIT initialpose publish via original V8 when safe')
    p.add_argument('--out',default='/tmp/robot01_setup_gate_v10.json')
    args=p.parse_args();report={'ready':False}
    try:
        cfg=yaml.safe_load((ROOT/'config.yaml').read_text(encoding='utf-8'))
        report.update(inspect(ROOT/'config.yaml',static_only=not(args.live_readonly or args.recover)))
        if args.live_readonly or args.recover:
            report['stage_2']=invoke_existing_v8(cfg,args.recover)
            if report['stage_2']['exit_code']!=0 or not report['stage_2']['aligned']:
                raise RuntimeError('AMCL not confirmed aligned: '+report['stage_2']['stdout'])
        report['ready']=True
    except Exception as e:report['error']=str(e)
    out=expand(args.out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ready'] else 2

if __name__=='__main__':raise SystemExit(main())
