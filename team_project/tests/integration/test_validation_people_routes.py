"""Regression for the reviewed test-site walking routes, not a runtime oracle."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[2]

def test_reviewed_people_routes_do_not_cross_test_site_walls_or_furniture():
    c=yaml.safe_load((ROOT/'config/integration/sites/validation_people.yaml').read_text())
    world=ET.parse(ROOT/'ros_ws/src/cafe_autonomy/worlds/validation_cafe.world.sdf').getroot().find('world')
    boxes=[]
    def pose(e):return list(map(float,e.findtext('pose','0 0 0 0 0 0').split()))
    for m in world.findall('model'):
        mp=pose(m)
        for link in m.findall('link'):
            lp=pose(link)
            for col in link.findall('collision'):
                size=col.findtext('geometry/box/size')
                if not size:continue
                cp=pose(col);p=[mp[i]+lp[i]+cp[i] for i in range(6)]
                assert all(abs(v)<1e-8 for v in p[3:])
                sx,sy,sz=map(float,size.split())
                if p[2]+sz/2>0 and p[2]-sz/2<c['person_height_m']:
                    boxes.append((p[0]-sx/2,p[0]+sx/2,p[1]-sy/2,p[1]+sy/2))
    assert boxes
    points=[]
    for lane in c['lanes']:
        a,b=lane['start'],lane['end'];steps=max(1,math.ceil(math.dist(a,b)/.05))
        points.extend((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps) for i in range(steps+1))
    entry=(c['test_spawn']['x'],c['test_spawn']['y']);points.append(entry)
    for x,y in points:
        for x0,x1,y0,y1 in boxes:
            distance=math.hypot(max(x0-x,0.,x-x1),max(y0-y,0.,y-y1))
            assert distance>c['person_radius_m'],(x,y,(x0,x1,y0,y1))

def test_people_spawn_points_meet_both_robots_release_clearance():
    c=yaml.safe_load((ROOT/'config/integration/sites/validation_people.yaml').read_text())
    site=yaml.safe_load((ROOT/'config/integration/sites/validation_cafe.yaml').read_text())
    starts=[lane['start'] for lane in c['lanes']]+[(c['test_spawn']['x'],c['test_spawn']['y'])]
    for p in starts:
        for robot in site['robots'].values():
            assert math.dist(p,(robot['x'],robot['y']))>=c['robot_release_radius_m']
