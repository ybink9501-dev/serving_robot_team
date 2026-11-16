import copy,math
from types import SimpleNamespace
import numpy as np
from serving_robot_mapping.mapping_oriented_geometry import OrientedMappingGuard,mapping_oriented_parameters


def wall_grid():
    a=np.zeros((120,160),dtype=int);a[:,:2]=100
    g=OrientedMappingGuard();g.update_grid(a.ravel().tolist(),160,120,.05,-4.,-3.)
    return g,a


def test_parallel_observation_fits_hardstop_but_sideways_rotation_is_blocked():
    g,_=wall_grid()
    assert g.pose_clear(-3.56,1.2,math.pi/2).allowed
    assert not g.pose_clear(-3.56,1.2,0.).allowed
    assert not g.rotation_clear((-3.56,1.2,math.pi/2),0.).allowed
    assert g.check_path([(-3.56,1.,math.pi/2),(-3.56,1.2,math.pi/2)]).allowed


def test_unknown_under_footprint_blocks_even_with_free_robot_center():
    g,a=wall_grid();a[60,80]=-1;g.update_grid(a.ravel().tolist(),160,120,.05,-4.,-3.)
    assert not g.pose_clear(.2,0.,0.).allowed


def test_swept_corner_collision_cannot_hide_between_clear_endpoints():
    a=np.zeros((200,200),dtype=int);a[100,120]=100
    g=OrientedMappingGuard();g.update_grid(a.ravel().tolist(),200,200,.02,-2.,-2.)
    assert g.pose_clear(0.,0.,0.).allowed
    assert g.pose_clear(0.,0.,math.pi/2).allowed
    assert not g.rotation_clear((0.,0.,0.),math.pi/2).allowed


def test_forward_path_does_not_accept_reverse_or_missing_heading():
    g,_=wall_grid()
    assert not g.check_path([(0.,0.,0.),(-.1,0.,0.)]).allowed
    assert not g.check_path([(0.,0.),(.1,0.)]).allowed


def test_raw_side_clearance_uses_expanded_body_and_front_keeps_hardstop():
    g,_=wall_grid();n=721;values=[12.]*n
    scan=SimpleNamespace(ranges=values,angle_min=-math.pi,angle_increment=2*math.pi/(n-1),range_min=.05,range_max=12.)
    values[540]=.4
    assert g.scan_clear(scan,.05)[0]
    values[360]=.36
    assert not g.scan_clear(scan,.05)[0]
    assert not g.scan_clear(scan,.5)[0]


def test_rotated_map_keeps_same_collision_geometry():
    g,a=wall_grid();g2=OrientedMappingGuard();g2.update_grid(a.ravel().tolist(),160,120,.05,3.,-4.,math.pi/2)
    assert g2.pose_clear(-1.2,-3.56,math.pi).allowed==g.pose_clear(-3.56,1.2,math.pi/2).allowed


def test_mapping_configuration_copy_preserves_serving_input_and_forbids_reverse():
    nav={k:{k:{'ros__parameters':{}}} for k in ('global_costmap','local_costmap')}
    nav.update(planner_server={'ros__parameters':{}},controller_server={'ros__parameters':{'general_goal_checker':{},'FollowPath':{'desired_linear_vel':.25,'allow_reversing':False}}})
    original=copy.deepcopy(nav);changed=mapping_oriented_parameters(nav)
    assert nav==original
    assert changed['planner_server']['ros__parameters']['GridBased']['motion_model_for_search']=='DUBIN'
    assert changed['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']==.12
    assert changed['controller_server']['ros__parameters']['FollowPath']==nav['controller_server']['ros__parameters']['FollowPath']
