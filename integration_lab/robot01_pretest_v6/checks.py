#!/usr/bin/env python3
"""Pure Python, READ-ONLY checks before Robot 01 driving. ROS is NOT needed."""
import hashlib
import json
import math
from pathlib import Path
import yaml

HERE = Path(__file__).resolve().parent

def expand(value):
    return Path(str(value)).expanduser().resolve()

def hash_file(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def read_yaml(p):
    return yaml.safe_load(p.read_text(encoding='utf-8'))

def pgm_load(path):
    with path.open('rb') as f:
        # Extract first four PGM tokens, ignore comments and variable whitespace.
        def tokens():
            for line in f:
                if b'#' in line:
                    line = line.partition(b'#')[0]
                for token in line.split():
                    yield token
                if b'\n' not in line:
                    continue
        # Fixed binary P5, practical current project header is token-per-line.
        # Read precisely through maxval without consuming bytes from the binary raster.
        acc=[]
        while len(acc)<4:
            line=f.readline()
            if not line: raise ValueError('PGM header incomplete')
            acc.extend(line.split(b'#',1)[0].split())
        if acc[0]!=b'P5' or len(acc)!=4: raise ValueError('Require P5 with one header-token sequence')
        w,h,maxval=map(int,acc[1:])
        raw=f.read()
        if w<=0 or h<=0 or maxval!=255 or len(raw)!=w*h:
            raise ValueError('PGM size/maxval/raster inconsistent')
        return w,h,raw

def occupancy_signature(width,height,raster):
    # ROS OccupancyGrid starts at bottom-left; PGM raster begins at top-left.
    # Semantically distinguish occupied and free, mirroring V3 prepare.py.
    rows=[bytes(1 if v<200 else 0 for v in raster[y*width:(y+1)*width]) for y in range(height)]
    return hashlib.sha256(b''.join(reversed(rows))).hexdigest()

def speed_config_ok(config,limits):
    speed=config['speed'];profiles=speed['profiles']
    max_v=float(speed['max_linear_mps']);max_w=float(speed['max_angular_radps'])
    assert all(math.isfinite(x) for x in (max_v,max_w)), 'Invalid nonfinite speed limits'
    assert 0<max_v<=float(limits['max_linear_mps']), 'Linear speed exceeds safety limit'
    assert 0<max_w<=float(limits['max_angular_rps']), 'Angular speed exceeds safety limit'
    assert {'low','normal','high'} <= profiles.keys(), 'Missing low/normal/high profile'
    low,normal,high=[float(profiles[k]) for k in ('low','normal','high')]
    assert 0<low<=normal<=high<=max_v, 'Speed profiles must be ordered and within safety ceiling'
    assert speed['default_profile'] in profiles, 'Default profile does not exist'
    assert speed['topic']=='/robot_01/speed_limit', 'Wrong Robot 01 Nav2 speed topic'
    return {'low':low,'normal':normal,'high':high}

def check(config_file=None):
    cpath=expand(config_file or HERE/'config.yaml')
    report={'scope':'Robot01 LAB preflight static','ready':False,'checks':[], 'warnings':[], 'errors':[]}
    def ok(name,data=None): report['checks'].append({'name':name,'result':'PASS','details':data})
    def err(name,ex): report['errors'].append({'name':name,'error':str(ex)})
    try:
        c=read_yaml(cpath)
        if c['schema_version']!=1 or c['robot_id']!='robot_01' or c['ros_domain_id']!=99:
            raise ValueError('V6 is Robot 01 ROS domain 99 only')
        v3=expand(c['serving_v3_dir']); layout=v3/'layout.yaml'; manifest=v3/'generated/manifest.json'
        waypoints=v3/'generated/active_waypoints.yaml'
        for p in (layout,manifest,waypoints):
            if not p.is_file():raise FileNotFoundError(f'V3 missing: {p}')
        lay=read_yaml(layout);snap=json.loads(manifest.read_text(encoding='utf-8'))
        wps=read_yaml(waypoints)
        if lay['robot_id']!='robot_01' or snap['robot_id']!='robot_01':raise ValueError('V3 robot_id mismatch')
        if not (lay['map_revision']==snap['map_revision']==wps['map_revision']):
            raise ValueError('V3 revision mismatch')
        ok('V3 identifiers and map revisions',{'map_revision':lay['map_revision']})
        for name,path,hashkey in (
            ('layout',layout,'layout_sha256'),('waypoints',waypoints,'waypoints_sha256'),
            ('world',expand(snap['world']),'world_sha256'),
            ('map yaml',expand(snap['map']),'map_yaml_sha256'),
            ('map raster',expand(snap['map_raster']),'map_raster_sha256')):
            if not path.is_file() or hash_file(path)!=snap[hashkey]:
                raise ValueError(f'Snapshot changed/missing: {name}. Run V3 prepare/deploy safely before driving')
        ok('V3 snapshot content integrity',{'files':5})
        if lay['frame_alignment']['mode'] != 'sdf_aligned_lab_only':
            raise ValueError('V6 supports only the current SDF-aligned team LAB map; verify SLAM alignment separately')
        mapy=read_yaml(expand(snap['map']));m=snap['map_info']
        raster_path=expand(snap['map_raster']); width,height,raw=pgm_load(raster_path)
        if (width,height)!=(m['width'],m['height']):raise ValueError('Raster dimensions differ from manifest')
        if abs(float(mapy['resolution'])-m['resolution'])>1e-8:raise ValueError('Resolution mismatch')
        if any(abs(float(a)-float(b))>1e-8 for a,b in zip(mapy['origin'][:3],m['origin'])):
            raise ValueError('Map origin mismatch')
        if occupancy_signature(width,height,raw)!=snap['occupancy_binary_sha256']:
            raise ValueError('Occupancy signature mismatch')
        ok('Map occupancy and geometry',{'width':width,'height':height,'resolution':m['resolution']})
        poses={x['entity_id']:x['pose'] for x in wps['waypoints']}
        if len(poses)!=len(wps['waypoints']):raise ValueError('Duplicate waypoint ID')
        if 'counter_pickup' not in poses or 'kitchen' not in poses:raise ValueError('Missing pickup/kitchen waypoint')
        clearance=float(lay['geometry']['robot_clearance_radius_m'])
        r=float(mapy['resolution']);ox=float(mapy['origin'][0]);oy=float(mapy['origin'][1]);
        # V3 prepare checks complete collision geometry; this independent check guards saved free-space center.
        for name,p in poses.items():
            x,y,yaw=[float(p[k]) for k in ('x','y','yaw')]
            if not all(math.isfinite(v) for v in (x,y,yaw)):
                raise ValueError(f'Nonfinite waypoint: {name}')
            ix=int(math.floor((x-ox)/r));iy=int(math.floor((y-oy)/r))
            if ix<0 or ix>=width or iy<0 or iy>=height:
                raise ValueError(f'Waypoint outside map: {name}')
            if raw[(height-1-iy)*width+ix]<200:
                raise ValueError(f'Waypoint blocked/unknown: {name}')
        for table in ('table_01','table_02','table_03','table_04'):
            candidates=snap['choices'].get(table)
            if not candidates:raise ValueError(f'No candidates for {table}')
            valid=[v for v in candidates if v['enabled'] and v['clearance_ok']]
            if not valid:raise ValueError(f'No safe enabled table approach: {table}')
            for item in valid:
                if item['id'] not in poses:raise ValueError(f'Active candidate has no waypoint: {item["id"]}')
            for item in candidates:
                if item['side'] in ('left','right') and item['enabled']:
                    report['warnings'].append(f'{table} chair-side approach enabled: inspect real furniture clearance')
        ok('All four tables have safe candidate IDs',{'total_destination_ids':len(poses)})
        s=lay['serving']
        if s['loading_mode']!='manual' or s['completion_mode']!='manual':
            report['warnings'].append('Serving auto-confirm enabled: first real test should use manual confirmations')
        if s['approach_selection']!='shortest_planned_path':
            raise ValueError('Unexpected V3 approach selection configuration')
        if not (float(s['active_goal_timeout_s'])>float(s['planner_timeout_s'])>0):
            raise ValueError('Serving timeouts inconsistent')
        ok('Serving confirmation and timeout gates')
        lim=read_yaml(expand(c['runtime_limits_yaml']))
        profiles=speed_config_ok(c,lim)
        ok('Speed profiles below existing safety limits',profiles)
        arrival_path=expand(c['arrival_yaml'])
        if not arrival_path.is_file():raise FileNotFoundError(f'Cannot find active arrival YAML: {arrival_path}')
        arr=read_yaml(arrival_path)['arrival']
        if not 0<float(arr['final_error_m'])<=float(arr['detection_radius_m']):
            raise ValueError('Arrival detection/final tolerances conflict')
        if not (float(arr['yaw_tolerance_rad'])>0 and float(arr['final_verify_timeout_s'])>0):
            raise ValueError('Arrival yaw/timeout invalid')
        if float(arr['final_error_m'])<.02:
            report['warnings'].append('Final error below 2cm: risk of valid Nav2 goal never entering SERVING; do not loosen silently')
        ok('Physical arrival gate defined',{'final_error_m':arr['final_error_m'],
            'detection_radius_m':arr['detection_radius_m'], 'yaw_tolerance_rad':arr['yaw_tolerance_rad']})
        report['ready']=True
    except Exception as e:
        err('blocking preflight condition',e)
    return report
