from pathlib import Path
import yaml
from cafe_autonomy.dynamic_policy import DynamicPolicy

def cfg():return yaml.safe_load((Path(__file__).resolve().parents[2]/'config/integration/runtime_limits.yaml').read_text())['dynamic_obstacles']

def test_lost_stationary_track_can_request_a_conservative_validated_detour():
    p=DynamicPolicy(cfg());p.tracks.observe([(2.,0.,.25,'person')],0.,'RGBD_YOLO')
    tr=next(iter(p.tracks.items.values()));tr.stopped_since=0.
    p.update(1.,(0.,0.,0.),0.,[(0.,0.),(5.,0.)],2.,True)
    p.tracks.age(2.,lambda _:False)
    r=p.update(32.,(0.,0.,0.),0.,[(0.,0.),(5.,0.)],2.,True)
    requests=[e for e in r['events'] if e['event']=='DYNAMIC_DETOUR_REQUEST']
    assert requests and r['hold_navigation']
    assert requests[0]['obstacles'][0]['radius']>.25
    assert tr.uncertain  # no deletion and no invented reclassification
    p.detour_result([tr.id],'EXECUTING',33.)
    safe_path=[(0.,0.),(0.,1.4),(3.,1.4),(5.,0.)]
    r=p.update(34.,(0.,0.,0.),0.,safe_path,2.,True,planned_path=True)
    assert not r['hold_navigation'] and tr.uncertain

def test_uncertain_motion_without_stationary_evidence_cannot_trigger_detour():
    p=DynamicPolicy(cfg());p.tracks.observe([(2.,0.,.25,'person')],0.,'RGBD_YOLO',motion_valid=False)
    p.update(1.,(0.,0.,0.),0.,[(0.,0.),(5.,0.)],2.,True)
    r=p.update(40.,(0.,0.,0.),0.,[(0.,0.),(5.,0.)],2.,True)
    assert r['hold_navigation']
    assert not any(e['event']=='DYNAMIC_DETOUR_REQUEST' for e in r['events'])
