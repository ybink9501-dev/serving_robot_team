"""Resolve personal site configuration without modifying TEAM assets."""
from pathlib import Path
import math
import re
import yaml


def merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def configure(root, site=None, people=False, mode="operation"):
    if mode not in ("mapping","operation"):raise ValueError("Invalid mode")
    root=Path(root).resolve()
    data=yaml.safe_load((root/'config/integration/system.yaml').read_text(encoding='utf-8'))
    if site:
        path=Path(site)
        if not path.is_absolute():path=root/path
        overlay=yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(overlay,dict):raise ValueError('Site must be a YAML mapping')
        required=('site_id','map_yaml','waypoints_file','zones_file','robots','world_to_map','people','world_file')
        for key in required:
            if key not in overlay:raise ValueError('Site requires explicit '+key)
        if set(overlay['robots'])!={'robot_01','robot_02'}:
            raise ValueError('Site must define both robot spawn/initial map poses')
        for pose in overlay['robots'].values():
            if not all(k in pose for k in ('x','y','yaw','map_x','map_y','map_yaw')):
                raise ValueError('Site robot pose must be complete')
        if not all(k in overlay['world_to_map'] for k in ('x','y','yaw')):
            raise ValueError('Site requires complete world_to_map transform')
        if 'config' not in overlay['people']:
            raise ValueError('Site requires explicit people config (null when disabled)')
        if not isinstance(overlay['people'].get('enabled',False),bool):
            raise ValueError('people.enabled must be boolean')
        # A new site must never silently use demo furniture geometry or spawn lanes.
        overlay.setdefault('static_world_geometry',False)
        data=merge(data,overlay)
    site_id=data.get('site_id','demo_store')
    if not isinstance(site_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]+',site_id):
        raise ValueError('Invalid site_id')
    def absolute(value):
        p=Path(value)
        return str(p if p.is_absolute() else root/p)
    for key in ('map_yaml','limits_file','waypoints_file','zones_file','database_path'):
        data[key]=absolute(data[key])
    if data.get('mission_state_directory'):
        data['mission_state_directory']=absolute(data['mission_state_directory'])
    data['perception']['weights']=absolute(data['perception']['weights'])
    data['nav2_files']={r:absolute(path) for r,path in data['nav2_files'].items()}
    if data.get('world_file'):
        data['world_file']=absolute(data['world_file'])
        import xml.etree.ElementTree as ET
        world=ET.parse(data['world_file']).getroot().find('world')
        if world is None or world.get('name')!='cafe':
            raise ValueError('Current simulation adapters require SDF world name cafe')
    data.setdefault('people',{})
    data['people']['enabled']=bool(people or data['people'].get('enabled',False))
    if data['people'].get('config'):data['people']['config']=absolute(data['people']['config'])
    if data['people']['enabled'] and not data['people'].get('config'):
        raise ValueError('Enabled people require a site-specific config')
    for key in ('map_yaml','limits_file','waypoints_file','zones_file'):
        if key=='map_yaml' and mode=='mapping':continue
        if not Path(data[key]).is_file():raise ValueError('Missing '+key+': '+data[key])
    if mode=='operation':
        map_data=yaml.safe_load(Path(data['map_yaml']).read_text(encoding='utf-8'))
        if not (Path(data['map_yaml']).parent/map_data['image']).is_file():
            raise ValueError('Map image missing')
        resolution=float(map_data['resolution'])
        if not math.isfinite(resolution) or resolution<=0:raise ValueError('Invalid map resolution')
        if len(map_data['origin'])!=3 or not all(math.isfinite(float(v)) for v in map_data['origin']):
            raise ValueError('Invalid map origin')
    from sys import path as module_path
    source=str(root/'ros_ws/src/cafe_autonomy')
    if source not in module_path:module_path.insert(0,source)
    from cafe_autonomy.waypoints import read_waypoints
    points=read_waypoints(data['waypoints_file'])
    for name in ('kitchen','robot_01_HOME','robot_02_HOME'):
        if name not in points:raise ValueError('Missing destination '+name)
    for robot,pose in data['robots'].items():
        if robot not in ('robot_01','robot_02'):raise ValueError('Unknown robot '+robot)
        if not all(k in pose and math.isfinite(float(pose[k])) for k in ('x','y','yaw','map_x','map_y','map_yaw')):
            raise ValueError('Invalid spawn/initial map pose '+robot)
    zones=yaml.safe_load(Path(data['zones_file']).read_text(encoding='utf-8'))
    if not isinstance(zones.get('zones'),dict):raise ValueError('zones must be a mapping')
    if data['people']['enabled']:
        pc=yaml.safe_load(Path(data['people']['config']).read_text(encoding='utf-8'))
        if pc.get('world')!='cafe' or not pc.get('lanes'):
            raise ValueError('People config requires cafe world and reviewed walking lanes')
    # Persistent requests and mission recovery must not leak between maps.
    if site:
        data['database_path']=str(root/'_local_only/sites'/site_id/'fleet.sqlite3')
        data['mission_state_directory']=str(root/'_local_only/sites'/site_id/'missions')
    output=root/'_local_only/config';output.mkdir(parents=True,exist_ok=True)
    out=output/'system.yaml'
    out.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    return out


if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=['mapping','operation'],default='operation');ap.add_argument('--site');ap.add_argument('--people',action='store_true')
    args=ap.parse_args()
    print(configure(Path(__file__).resolve().parents[1],args.site,args.people,args.mode))
