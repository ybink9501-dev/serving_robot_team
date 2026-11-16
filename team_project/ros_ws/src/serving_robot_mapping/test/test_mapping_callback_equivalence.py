import math
import numpy as np
import pytest
from serving_robot_mapping.mapping_domain import apply_domain,inside
from serving_robot_mapping.mapping_path_risk import MappingPathRiskGuard


def test_domain_batch_exactly_matches_scalar_geometry():
    domain={'interior_polygon':[(-.4,-.2),(1.2,-.2),(1.2,1.4),(-.4,1.4)]}
    w,h,r,ox,oy,yaw=43,31,.05,-.5,-.7,.4
    values=np.random.default_rng(41).choice([-1,0,25,26,50,100],w*h).tolist()
    expected=values[:];counts={'excluded_free':0,'excluded_unknown':0,'excluded_cells':0}
    for y in range(h):
        for x in range(w):
            px,py=(x+.5)*r,(y+.5)*r
            if inside(domain,ox+math.cos(yaw)*px-math.sin(yaw)*py,oy+math.sin(yaw)*px+math.cos(yaw)*py):continue
            i=y*w+x;counts['excluded_cells']+=1
            counts['excluded_free']+=int(0<=values[i]<=25)
            counts['excluded_unknown']+=int(values[i]==-1);expected[i]=100
    assert apply_domain(values,w,h,r,ox,oy,yaw,domain)==(expected,counts)


@pytest.mark.parametrize('value,reason',[(100,'PATH_NEAR_OCCUPIED'),(-1,'PATH_NEAR_UNKNOWN'),(50,'PATH_NEAR_UNCERTAIN')])
@pytest.mark.parametrize('delta',[-1e-10,0.,1e-10])
def test_spatial_index_preserves_exact_threshold(value,reason,delta):
    data=[0]*10000;data[50*100+50]=value
    guard=MappingPathRiskGuard();assert guard.update_grid(data,100,100,.05,0,0)
    radius=.48+.05*math.sqrt(2)/2
    x,y=2.525+radius+delta,2.525
    actual_distance=math.hypot(2.525-x,2.525-y)
    result=guard.check_path([(x,y),(x,y)])
    assert result.allowed==(actual_distance>radius)
    if not result.allowed:assert result.reason==reason


def test_first_row_major_hazard_reason_is_preserved():
    data=[0]*10000;data[45*100+50]=-1;data[50*100+53]=100
    guard=MappingPathRiskGuard();guard.update_grid(data,100,100,.05,0,0)
    result=guard.check_path([(2.525,2.525),(2.525,2.525)])
    assert result.reason=='PATH_NEAR_UNKNOWN'
    assert result.minimum_occupied_distance_m==pytest.approx(.15)
    assert not guard.update_grid(data,100,100,float('nan'),0,0)
    assert guard.check_path([(2.,2.),(3.,3.)]).reason=='MAP_NOT_READY'
