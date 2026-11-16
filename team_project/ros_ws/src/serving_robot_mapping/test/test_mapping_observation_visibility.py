import pytest
from serving_robot_mapping.mapping_cell_visibility import cell_face_visible
from serving_robot_mapping.mapping_visibility_audit import boundary_visibility_audit


def test_cell_visibility_does_not_tunnel_through_a_blocked_corner():
    free=[False]*25;free[1*5+1]=True
    assert not cell_face_visible(free,5,5,1,1,2,2)
    free[1*5+2]=True
    assert cell_face_visible(free,5,5,1,1,2,2)


def test_real_lidar_range_can_propose_a_view_beyond_old_five_meter_cap():
    w,h=150,35;data=[0]*(w*h)
    for x in range(w):data[x]=100;data[(h-1)*w+x]=100
    for y in range(h):data[y*w]=100;data[y*w+w-1]=100
    data[17*w+130]=-1
    common=dict(data=data,width=w,height=h,resolution=.05,seed_cell=17*w+10,
                stride_cells=1,viewpoint_filter=lambda x,y:(x,y)==(20,17),
                target_cell_faces=True,filter_before_visibility=True)
    assert not boundary_visibility_audit(**common,sensor_range_m=5.)['candidates']
    assert boundary_visibility_audit(**common,sensor_range_m=12.)['candidates']==[(1,20,17)]


def test_early_filter_preserves_eligible_candidates_and_skips_unusable_views():
    w=h=35;data=[0]*(w*h);data[17*w+29]=-1
    common=dict(data=data,width=w,height=h,resolution=.1,seed_cell=17*w+8,
                stride_cells=1,viewpoint_filter=lambda x,y:10<=x<=20 and 10<=y<=20)
    old=boundary_visibility_audit(**common)
    early=boundary_visibility_audit(**common,filter_before_visibility=True)
    assert old['candidates']==early['candidates']
