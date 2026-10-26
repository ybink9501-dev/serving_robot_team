#!/usr/bin/env python3
"""Offline structural tests only. Never labels offline checks as real Gazebo drive."""
import sys,ast,re,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from importlib.machinery import SourceFileLoader
r=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent.parent
p=r/'ros_ws/src/cafe_autonomy'
files=list(p.rglob('*'));errors=[]
for path in files:
    if not path.is_file():continue
    try:
        if path.suffix=='.py':ast.parse(path.read_text())
        if path.suffix in ('.xml','.sdf'):ET.parse(path)
        if path.suffix in ('.yaml','.yml'):yaml.safe_load(path.read_text())
    except Exception as ex:errors.append((str(path.relative_to(r)),str(ex)))
require=['launch/01_world.launch.py','launch/02_robots.launch.py','config/nav2_robot_01.yaml','config/nav2_robot_02.yaml',
         'config/waypoints.yaml','config/zones.yaml','config/runtime_limits.yaml','maps/cafe.pgm','maps/cafe.yaml','worlds/cafe.sdf','models/robot_01.sdf','models/robot_02.sdf',
         'cafe_autonomy/mission.py','cafe_autonomy/fleet.py','cafe_autonomy/safety.py','cafe_autonomy/scan_reframe.py',
         'web/dashboard.html','config/navigate_no_reverse.xml',
         'cafe_autonomy/metrics.py','models/person.sdf','models/movable_obstacle.sdf']
for q in require:
    if not (p/q).is_file() or (p/q).stat().st_size==0:errors.append((q,'MISSING/EMPTY'))
world=(p/'launch/01_world.launch.py').read_text()
nav=(p/'launch/02_robots.launch.py').read_text()
if 'gz_sim.launch.py' not in world:errors.append(('world','missing Gazebo launcher'))
if 'gz_sim.launch.py' in nav:errors.append(('02_robots','must not spawn Gazebo server'))
for rid in ('robot_01','robot_02'):
    y=yaml.safe_load((p/f'config/nav2_{rid}.yaml').read_text())
    bad=[k for k in y if not k.startswith('/'+rid+'/')]
    if bad:errors.append((rid,'YAML keys lack namespace prefix (params will NOT apply): '+str(bad)))
    cs=y['/'+rid+'/controller_server']['ros__parameters']
    xy=cs['general_goal_checker']['xy_goal_tolerance']
    vx=cs['FollowPath']['vx_max']
    cfg=yaml.safe_load((p/'config/runtime_limits.yaml').read_text())
    if xy!=cfg['navigation']['goal_xy_tolerance_m'] or vx!=cfg['navigation']['max_forward_mps']:
        errors.append((rid,'Nav2 values disagree with runtime_limits.yaml'))
    if re.search(r'&id\d+|\*id\d+', (p/f'config/nav2_{rid}.yaml').read_text()):
        errors.append((rid,'Nav2 ROS parser refuses YAML anchors / aliases'))
for key in ('metrics','cafe_gazebo_truth_bridge'):
    if key not in nav:errors.append(('02_robots',f'missing {key}'))
for key in ('tools/sim_scenario.py','tools/acceptance.py','tools/contact_record.py','tools/analyze_metrics.py'):
    if not (r/key).exists():errors.append((key,'MISSING'))
if '/cafe/people_ground_truth' in (p/'cafe_autonomy/safety.py').read_text():
    errors.append(('safety','Simulator ground truth must not serve as detection'))
print('Source files:',len([f for f in files if f.is_file()]))
print('Offline structural checks:','PASS' if not errors else 'FAIL')
for k,v in errors:print('FAIL',k,v)
print('ROS/Gazebo runtime: NOT RUN HERE')
sys.exit(bool(errors))
