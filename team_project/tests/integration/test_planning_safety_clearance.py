from types import SimpleNamespace
from pathlib import Path
import yaml
from cafe_autonomy.dynamic_policy import navigation_envelope_radius,mutual_peer_block
from cafe_autonomy.dynamic_mission import candidate_evidence

def cfg():return yaml.safe_load((Path(__file__).resolve().parents[2]/'config/integration/runtime_limits.yaml').read_text())['dynamic_obstacles']

def test_person_planning_envelope_respects_semantic_surface_stop_distance():
    c=cfg();p=SimpleNamespace(radius=.25,label='person')
    r=navigation_envelope_radius(p,c)
    assert r+c['robot_radius_m']-p.radius>=c['person_stop_m']+c['prediction_margin_m']-1e-9
    ev=candidate_evidence([(0.,0.),(3.,0.)],[{'x':1.,'y':1.,'radius':.25,'label':'person'}],c,'GridBased')
    assert ev['obstacle_clearances'][0]['policy_required_m']>=1.05

def test_fleet_recognizes_mutual_dynamic_peer_hold_before_minimum_separation():
    status={r:{'state':'GOING_DIRECT'} for r in ('robot_01','robot_02')}
    report={'sensor_fresh':True,'tracks':[{'class':'robot','blocks_path':True}]}
    assert mutual_peer_block(status,{r:report for r in status},set())
    assert mutual_peer_block(status,{'robot_01':report},{'robot_02'})
    assert not mutual_peer_block(status,{'robot_01':report},set())
    assert not mutual_peer_block(status,{'robot_01':{'sensor_fresh':False,'tracks':report['tracks']}},{'robot_02'})
