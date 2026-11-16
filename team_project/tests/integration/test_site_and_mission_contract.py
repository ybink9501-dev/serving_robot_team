"""Cross-site and signal regressions for the personal integration only."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from std_msgs.msg import String
from cafe_autonomy.policy import FleetCore, may_confirm, next_after_nav, should_return_home
from cafe_autonomy.waypoints import read_waypoints, table_destinations, choose_waypoint
from cafe_autonomy.mission import Mission
from cafe_autonomy.dynamic_obstacles import DynamicObstacles

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('configure_runtime_site',ROOT/'scripts/configure_runtime.py')
config=importlib.util.module_from_spec(spec);spec.loader.exec_module(config)


def test_collection_has_distinct_wait_state_and_matching_signal():
    assert next_after_nav('GOING_DIRECT','TABLE_COLLECT')=='WAIT_COLLECTION'
    assert may_confirm('COLLECTED','WAIT_COLLECTION','TABLE_COLLECT')
    assert not may_confirm('COLLECTED','WAIT_HANDOVER','TABLE_COLLECT')
    assert not may_confirm('COMPLETE','WAIT_COLLECTION','TABLE_COLLECT')
    events=[]
    m=SimpleNamespace(request_id='collect-1',confirmed_signals={},mode='TABLE_COLLECT',
        state='WAIT_COLLECTION',xy_tolerance=.1,yaw_tolerance=.25,
        pose_error=lambda:(.08,.01,(0.,0.,0.)),emit=lambda *a,**kw:events.append(a))
    Mission.confirm(m,String(data=json.dumps({'request_id':'other','command':'COLLECTED'})))
    assert m.state=='WAIT_COLLECTION'
    Mission.confirm(m,String(data=json.dumps({'request_id':'collect-1','command':'COLLECTED'})))
    assert m.state=='COMPLETED'
    assert m.confirmed_signals=={'collect-1':['COLLECTED']}


def test_idle_home_requires_empty_queue_and_healthy_completed_robot():
    args=dict(state='COMPLETED',mode='DELIVER_DIRECT',active=False,queued=False,
              ready=True,pose=(1.,1.),home=(0.,0.),tolerance=.1)
    assert should_return_home(**args)
    for change in [dict(queued=True),dict(active=True),dict(ready=False),dict(mode='GO_HOME'),
                   dict(state='WAIT_LOAD'),dict(state='ERROR'),dict(pose=None),dict(pose=(.05,0.))]:
        assert not should_return_home(**(args|change))
    core=FleetCore()
    core.submit('idle-home-123',None,'robot_01','GO_HOME')
    core.submit('customer','table_20','robot_01','DELIVER_DIRECT')
    assert core.dispatch_next({'robot_01'}).request_id=='customer'


def test_new_map_destination_names_and_rotated_table_approach(tmp_path):
    p=tmp_path/'points.yaml'
    p.write_text(yaml.safe_dump({'destinations':{'table_20':{'approaches':{
        'west':{'x':2.,'y':3.,'yaw':0.}}},'kitchen':{'x':0.,'y':1.,'yaw':0.},
        'robot_01_HOME':{'x':0.,'y':0.,'yaw':0.}}}))
    poses=read_waypoints(p)
    assert table_destinations(poses)=={'table_20'}
    assert choose_waypoint(poses,'table_20',(1.,3.))['yaw']==0.
    p.write_text(yaml.safe_dump({'destinations':{'table_20':{'approaches':{
        'bad':{'x':float('nan'),'y':0.,'yaw':0.}}}}}))
    with pytest.raises(ValueError):read_waypoints(p)


def test_occupancy_only_classification_does_not_need_world_shapes():
    # One known wall cell and one free cell: map alone distinguishes foreground.
    m=SimpleNamespace(map_meta=SimpleNamespace(origin=SimpleNamespace(
        position=SimpleNamespace(x=0.,y=0.),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)),
        resolution=1.,width=2,height=1),distance_field=np.array([[0.,1.]]),
        cfg={'static_match_margin_m':.15},static_shapes=[])
    assert DynamicObstacles.static_points(m,np.array([.2,1.2]),np.array([.2,.2])).tolist()==[True,False]
    assert m.static_alignment['accepted'] is False


def test_site_overlay_isolated_and_rejects_demo_fallback(tmp_path):
    # A second tiny site uses a different map origin, names and people routes.
    import shutil
    shutil.copytree(ROOT/'config/integration',tmp_path/'config/integration')
    (tmp_path/'scripts').mkdir()
    shutil.copytree(ROOT/'ros_ws/src/cafe_autonomy/cafe_autonomy',
                    tmp_path/'ros_ws/src/cafe_autonomy/cafe_autonomy',ignore=shutil.ignore_patterns('__pycache__'))
    (tmp_path/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([254]*4))
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[10.,-2.,.5])))
    pts={'kitchen':{'x':10.1,'y':-1.9,'yaw':0.},'table_20':{'x':10.15,'y':-1.85,'yaw':.5}}
    for r in ('robot_01','robot_02'):pts[r+'_HOME']={'x':10.1,'y':-1.9,'yaw':0.}
    (tmp_path/'points.yaml').write_text(yaml.safe_dump({'destinations':pts}))
    (tmp_path/'zones.yaml').write_text('zones: {}\n')
    (tmp_path/'world.sdf').write_text('<sdf version="1.9"><world name="cafe"/></sdf>')
    overlay=dict(site_id='second',map_yaml='map.yaml',waypoints_file='points.yaml',zones_file='zones.yaml',
        world_file='world.sdf',robots={r:dict(x=0.,y=0.,yaw=0.,map_x=10.1,map_y=-1.9,map_yaw=0.)
        for r in ('robot_01','robot_02')},world_to_map=dict(x=10.,y=-2.,yaw=.5),people=dict(enabled=False,config=None))
    p=tmp_path/'site.yaml';p.write_text(yaml.safe_dump(overlay))
    result=yaml.safe_load(config.configure(tmp_path,p).read_text())
    assert result['static_world_geometry'] is False
    assert '/sites/second/' in result['database_path']
    assert '/sites/second/' in result['mission_state_directory']
    assert result['people']['config'] is None
    broken=copy.deepcopy(overlay);del broken['robots']['robot_02']
    p.write_text(yaml.safe_dump(broken))
    with pytest.raises(ValueError,match='both robot'):config.configure(tmp_path,p)
    p.write_text(yaml.safe_dump(overlay))
    with pytest.raises(ValueError,match='site-specific'):config.configure(tmp_path,p,people=True)

    (tmp_path/'map.yaml').unlink()
    config.configure(tmp_path,p,mode='mapping')
    with pytest.raises(ValueError,match='Missing map_yaml'):config.configure(tmp_path,p)
