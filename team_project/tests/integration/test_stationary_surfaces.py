from pathlib import Path
import yaml
from cafe_autonomy.dynamic_policy import DynamicPolicy,obstacle_samples

def cfg():return yaml.safe_load((Path(__file__).resolve().parents[2]/'config/integration/runtime_limits.yaml').read_text())['dynamic_obstacles']

def engine(points,label='unclassified_object',moving=False):
    p=DynamicPolicy(cfg())
    p.tracks.observe([(.8,.575,.24,label,None,points)],0.,'LiDAR_FOREGROUND')
    tr=next(iter(p.tracks.items.values()));tr.stopped_since=0. if not moving else None;tr.moving=moving
    return p,tr

def test_observed_chair_face_does_not_expand_into_clear_parallel_lane():
    p,tr=engine([(.6,.575),(.8,.575),(1.,.575)])
    r=p.update(2.,(0.,0.,0.),.25,[(0.,0.),(2.,0.)],3.,True)
    assert not r['stop'] and not r['blocker_ids']
    assert len(obstacle_samples(tr,cfg()))==3

def test_observed_surface_on_the_route_still_stops_robot():
    p,_=engine([(.6,.1),(.8,.1),(1.,.1)])
    r=p.update(2.,(0.,0.,0.),.25,[(0.,0.),(2.,0.)],.5,True)
    assert r['stop'] and r['hold_navigation']

def test_people_and_moving_objects_keep_full_body_envelope():
    for label,moving in [('person',False),('unclassified_object',True),('robot',False)]:
        p,tr=engine([(.6,.575),(1.,.575)],label,moving)
        assert obstacle_samples(tr,cfg())==[(tr.x,tr.y,tr.radius)]
