"""Reproduce ghost tracks and unsafe clearance without changing safety limits."""
from pathlib import Path
from types import SimpleNamespace
import math
import time
import numpy as np
import yaml
from cafe_autonomy.dynamic_policy import Tracks
from cafe_autonomy.dynamic_obstacles import DynamicObstacles

ROOT=Path(__file__).resolve().parents[2]
def cfg():return yaml.safe_load((ROOT/'config/integration/runtime_limits.yaml').read_text())['dynamic_obstacles']


def test_batch_association_keeps_both_tracks_when_greedy_match_would_orphan_one():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person'),(.6,0.,.2,'person')],0.,'RGBD_YOLO')
    tracks.observe([(.2,0.,.2,'person'),(-.4,0.,.2,'person')],.2,'RGBD_YOLO')
    assert len(tracks.items)==2
    assert tracks.items['object_1'].x==-.4
    assert tracks.items['object_2'].x==.2


def test_invalid_motion_reference_does_not_reuse_old_velocity_or_stationary_history():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person')],0.,'RGBD_YOLO')
    tr=next(iter(tracks.items.values()))
    tr.vx=1.;tr.moving=True;tr.low_motion_since=0.;tr.stationary_anchor=(0.,0.)
    tracks.observe([(.1,0.,.2,'person')],.2,'RGBD_YOLO',motion_valid=False)
    assert tr.uncertain
    assert not tr.moving and tr.vx==0. and tr.vy==0.
    assert not tr.history and tr.low_motion_since is None and tr.stationary_anchor is None
    assert tr.snapshot(.2)['uncertainty_reason']=='MOTION_REFERENCE_UNAVAILABLE'
    tracks.observe([(.1,0.,.2,'person')],.3,'RGBD_YOLO')
    assert not tr.uncertain and tr.snapshot(.3)['uncertainty_reason'] is None
    assert tr.stopped_since is None


def test_reacquisition_after_gap_does_not_reuse_previous_motion():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person')],0.,'RGBD_YOLO')
    tr=next(iter(tracks.items.values()))
    tr.vx=1.;tr.moving=True;tr.low_motion_since=0.;tr.stationary_anchor=(0.,0.)
    tracks.age(2.,lambda _:False)
    assert tr.uncertain
    tracks.observe([(.1,0.,.2,'person')],2.1,'RGBD_YOLO')
    assert tr.vx==0. and not tr.moving and tr.low_motion_since is None


def test_map_left_boundary_does_not_truncate_outside_point_into_occupied_cell():
    m=SimpleNamespace(map_meta=SimpleNamespace(origin=SimpleNamespace(
        position=SimpleNamespace(x=0.,y=0.),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)),
        resolution=1.,width=2,height=1),distance_field=np.array([[0.,1.]]),
        cfg={'static_match_margin_m':.15},static_shapes=[])
    assert not DynamicObstacles.static_points(m,np.array([-.01]),np.array([.2]))[0]


def adapter_scan(ranges, *, stale=False):
    return SimpleNamespace(scan=SimpleNamespace(ranges=ranges,range_min=.1,range_max=10.,
        angle_min=-.2,angle_increment=.1),scan_pose=(0.,0.,0.),
        scan_wall=time.monotonic()-(10. if stale else 0.),
        cfg=cfg(),static_shapes=[],static_alignment={})


def person():return SimpleNamespace(x=1.,y=0.,radius=.2,label='person',sources={'RGBD_YOLO'})


def test_old_lidar_scan_cannot_delete_lost_person_track():
    assert not DynamicObstacles.verified_clear(adapter_scan([math.inf]*5,stale=True),person())


def test_one_clear_centre_ray_does_not_prove_the_person_footprint_is_clear():
    assert not DynamicObstacles.verified_clear(adapter_scan([.9,.9,math.inf,.9,.9]),person())
    assert DynamicObstacles.verified_clear(adapter_scan([math.inf]*5),person())


def test_negative_infinity_is_not_clear_space():
    assert not DynamicObstacles.verified_clear(adapter_scan([-math.inf]*5),person())


def test_motion_reference_change_does_not_turn_coordinate_offset_into_velocity():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person')],0.,'RGBD_YOLO',motion_reference='registered_world')
    tracks.observe([(.1,0.,.2,'person')],.5,'RGBD_YOLO',motion_positions=[(10.,10.)],
                   motion_reference='registered_map')
    tr=next(iter(tracks.items.values()))
    assert tr.vx==0. and tr.vy==0. and not tr.moving
    assert len(tr.history)==1


def test_lost_observation_has_explicit_reason_and_is_not_deleted_without_evidence():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person')],0.,'RGBD_YOLO')
    tracks.age(2.,lambda _:False)
    tr=next(iter(tracks.items.values()))
    assert tr.snapshot(2.)['uncertainty_reason']=='OBSERVATION_LOST_UNCONFIRMED'
    tracks.age(3.,lambda _:True)
    assert not tracks.items


def test_clear_ray_cannot_verify_space_beyond_lidar_range():
    m=adapter_scan([math.inf]*5)
    m.scan.range_max=1.
    assert not DynamicObstacles.verified_clear(m,person())


def test_peer_identity_survives_pose_correction_without_duplicate_ghost():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.33,'robot','robot_02')],0.,'PEER_MAP_TF')
    tracks.observe([(2.,0.,.33,'robot','robot_02')],2.,'PEER_MAP_TF')
    assert len(tracks.items)==1
    tr=next(iter(tracks.items.values()))
    assert tr.x==2. and tr.snapshot(2.)['entity_id']=='robot_02'


def test_known_peer_can_be_cleared_only_with_fresh_identity_evidence():
    tr=SimpleNamespace(x=0.,y=0.,radius=.33,label='robot',entity_id='robot_02')
    m=SimpleNamespace(robot='robot_01',cfg=cfg(),scan=None,scan_pose=None,
                      pose=lambda _: (2.,0.,0.))
    assert DynamicObstacles.verified_clear(m,tr)
    def missing(_):raise ValueError('Stale transform')
    m.pose=missing
    assert not DynamicObstacles.verified_clear(m,tr)


def test_partial_fov_cannot_prove_the_whole_old_footprint_is_clear():
    m=adapter_scan([math.inf]*5)
    m.scan.angle_min=0.
    assert not DynamicObstacles.verified_clear(m,person())


def test_batch_matching_minimizes_identity_jumps_when_all_pairs_are_possible():
    tracks=Tracks(cfg())
    tracks.observe([(0.,0.,.2,'person'),(.6,0.,.2,'person')],0.,'RGBD_YOLO')
    tracks.observe([(.1,0.,.2,'person'),(.2,0.,.2,'person')],.2,'RGBD_YOLO')
    assert len(tracks.items)==2
    assert tracks.items['object_1'].x==.1
    assert tracks.items['object_2'].x==.2


def test_wall_beyond_body_edge_is_clear_even_when_before_centre_far_plane():
    m=adapter_scan([1.25,1.4,1.4,1.4,1.25])
    assert DynamicObstacles.verified_clear(m,person())


def test_echo_inside_body_edge_still_withholds_clearance():
    m=adapter_scan([1.,1.4,1.4,1.4,1.])
    assert not DynamicObstacles.verified_clear(m,person())
