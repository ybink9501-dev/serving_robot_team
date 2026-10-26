#!/usr/bin/env python3
"""Generate a reviewed LAB semantic snapshot from the SAME team SDF and map.
Read-only to Gazebo/ROS and shared team files. Writes only serving_v3/generated.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import yaml

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(Path.home()/'team_robot_lab/lab_scripts/cafe_lab_additions_20261016'))
from build_cafe_map import parse_world, hit  # existing LAB geometry reader, unmodified


def expand(v): return Path(os.path.expandvars(os.path.expanduser(str(v)))).resolve()
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def must(ok,reason):
    if not ok: raise ValueError(reason)
def transform(x,y,pose):
    a=pose['yaw'];return (pose['x']+math.cos(a)*x-math.sin(a)*y,
                          pose['y']+math.sin(a)*x+math.cos(a)*y)
def load_pgm(file):
    with file.open('rb') as f:
        must(f.readline().strip()==b'P5','Only binary P5 PGM supported')
        v=[]
        while len(v)<3:
            line=f.readline();must(bool(line),'PGM truncated')
            v.extend(line.split(b'#',1)[0].split())
        w,h,maxv=map(int,v[:3]);must(maxv==255,'PGM maxval must be 255')
        raw=f.read();must(len(raw)==w*h,'PGM raster size mismatch')
        return w,h,raw

def main():
    cfg_path=ROOT/'layout.yaml';c=yaml.safe_load(cfg_path.read_text())
    must(c['schema_version']==3,'Expected layout schema 3')
    rev=c['map_revision'];must(type(rev)==int and rev>0,'map_revision must be positive integer')
    world=expand(c['world_file']);mapfile=expand(c['map_file'])
    must(world.is_file() and mapfile.is_file(),'World/map missing')
    must(c['frame_alignment']['mode']=='sdf_aligned_lab_only',
         'Unknown alignment: do not copy Gazebo coordinates into an unrelated SLAM map')
    align=c['frame_alignment']['world_to_map'];must(all(k in align for k in ('x','y','yaw')),'Alignment missing')
    obstacles,models,warnings,skipped=parse_world(world)
    must(not warnings,'Unsupported SDF geometry: '+'; '.join(warnings[:5]))
    mapping={m['name']:m for m in models}
    mcfg=yaml.safe_load(mapfile.read_text());pgm=expand(mapfile.parent/mcfg['image'])
    must(pgm.is_file(),'Map raster missing')
    w,h,raster=load_pgm(pgm)
    r=float(mcfg['resolution']);ox,oy,oyaw=map(float,mcfg['origin'][:3])
    must(r>0 and abs(oyaw)<1e-8,'Unsupported map resolution / yaw')
    clearance=float(c['geometry']['robot_clearance_radius_m'])
    step=float(c['geometry']['sampling_grid_step_m']);must(.01<=step<=clearance/2,'Invalid clearance sampling')
    def free(wx,wy):
        # Occupancy checked in actual map frame, SDF obstacles checked in world frame.
        mx,my=transform(wx,wy,align)
        k=int(math.ceil(clearance/step))
        for i in range(-k,k+1):
            for j in range(-k,k+1):
                dx=i*step;dy=j*step
                if dx*dx+dy*dy>clearance*clearance:continue
                ix=int(math.floor((mx+dx-ox)/r));iy=int(math.floor((my+dy-oy)/r))
                if ix<0 or ix>=w or iy<0 or iy>=h or raster[(h-1-iy)*w+ix]<200:return False
        if any(hit(wx,wy,s,extra=clearance) for s in obstacles):return False
        return True
    def pos(model,offset):
        local=(float(offset[0]),float(offset[1]))
        wx,wy=transform(*local,{'x':model['x'],'y':model['y'],'yaw':model['yaw']})
        mx,my=transform(wx,wy,align)
        fx,fy=transform(model['x'],model['y'],align)
        yaw=math.atan2(fy-my,fx-mx)
        return {'x':round(mx,6),'y':round(my,6),'yaw':round(yaw,6)},free(wx,wy)
    missing=[]
    for name in [c['geometry']['kitchen_model'],*c['geometry']['tables'].values()]:
        if name not in mapping:missing.append(name)
    must(not missing,'SDF models missing: '+str(missing))
    entities=[];waypoints=[];choices={};blocked=[]
    def add(name,pose):
        waypoints.append({'id':'wp_'+name,'entity_id':name,'pose':pose})
    kitchen,ok=pos(mapping[c['geometry']['kitchen_model']],c['geometry']['pickup_offset_local_m'])
    must(ok,'Kitchen/pickup waypoint is blocked or outside the current map')
    add('kitchen',kitchen)
    add('counter_pickup',kitchen)  # Same physical location for LAB only; semantic distinction preserved
    for table,modelname in c['geometry']['tables'].items():
        model=mapping[modelname]
        candidates=[]
        for side,a in c['geometry']['table_approaches'].items():
            p,clear=pos(model,a['offset_local_m']);enabled=bool(a.get('enabled',False)) and clear
            item={'id':f'{table}_{side}','side':side,'pose':p,
                  'enabled':enabled,'clearance_ok':clear,
                  'reason':('enabled' if enabled else ('blocked' if not clear else a.get('reason','disabled')))}
            candidates.append(item)
            if enabled:add(item['id'],p)
            if not clear:blocked.append(item['id'])
        must(any(a['enabled'] for a in candidates),f'{table}: no safe/active approach candidates')
        # Original destination ID retained for legacy clients; chooses first enabled candidate.
        legacy=next(a for a in candidates if a['enabled'])
        add(table,legacy['pose'])
        choices[table]=candidates
        entities.append({'id':table,'gazebo_model':modelname,'center_world':[model['x'],model['y']],
                         'center_map':list(transform(model['x'],model['y'],align))})
    must(len({w['entity_id'] for w in waypoints})==len(waypoints),'Duplicate destination IDs')
    payload={'map_revision':rev,'waypoints':waypoints}
    out=ROOT/'generated';out.mkdir(parents=True,exist_ok=True)
    yamlfile=out/'active_waypoints.yaml'; data=yaml.safe_dump(payload,sort_keys=False,allow_unicode=True)
    # OccupancyGrid data uses bottom-left-first rows: PGM is top-left-first.
    expected_grid=bytearray(1 if val<200 else 0 for val in raster)
    expected_grid=b''.join(expected_grid[i*w:(i+1)*w] for i in range(h-1,-1,-1))
    source={'occupancy_binary_sha256':hashlib.sha256(expected_grid).hexdigest(),
            'world':str(world),'world_sha256':sha(world),'map':str(mapfile),
            'map_yaml_sha256':sha(mapfile),'map_raster':str(pgm),'map_raster_sha256':sha(pgm),
            'layout_sha256':sha(cfg_path),'robot_id':c['robot_id'],'map_revision':rev,
            'map_info':{'width':w,'height':h,'resolution':r,'origin':[ox,oy,oyaw]},
            'waypoints_sha256':hashlib.sha256(data.encode()).hexdigest(),
            'choices':choices,'entities':entities,'pickup_pose':kitchen,'blocked_candidates':blocked,
            'limitations':['SDF aligned LAB only, NOT authoritative SLAM map',
              'No actual LiDAR localization or physical arrival test',
              'Mapped stops may conflict between two robots; Fleet zone/slot locking required',
              'Left/right approaches with chairs are inactive']}
    # Atomic writes, prevent partial files during process interruption.
    for p,s in [(yamlfile,data),(out/'manifest.json',json.dumps(source,ensure_ascii=False,indent=2))]:
        tmp=p.with_name(p.name+'.tmp');tmp.write_text(s,encoding='utf8');os.replace(tmp,p)
    print('GENERATED:',yamlfile)
    print('MAP:',w,'x',h,'resolution',r,'origin',(ox,oy,oyaw),'map_revision',rev)
    print('KITCHEN:',kitchen,'(LAB provisional pickup, staff confirmation required)')
    for t,rows in choices.items():
        print(t,', '.join(x['side']+(' ACTIVE' if x['enabled'] else ' INACTIVE:'+x['reason']) for x in rows))
    print('No ROS nodes or existing project files modified')

if __name__=='__main__':
    try:main()
    except Exception as e:raise SystemExit('PREPARE FAILED: '+str(e))
