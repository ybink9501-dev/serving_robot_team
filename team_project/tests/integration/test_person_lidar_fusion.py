import pytest
from cafe_autonomy.rgbd_projection import conservative_person_geometry

def test_lidar_body_prevents_rgbd_overestimated_gap():
    p=conservative_person_geometry((1.6,0.,.25,'person'),(1.,0.),.22,(0.,0.),.25)
    assert p[:3]==pytest.approx((1.,0.,.25))

def test_partial_arc_and_farther_return_cannot_increase_free_space():
    p=(1.,0.,.25,'person')
    assert conservative_person_geometry(p,(1.2,0.),.22,(0.,0.),.25)==p
    q=conservative_person_geometry((2.,0.,.25,'person'),(.7,0.),.1,(0.,0.),.25)
    assert q[:3]==pytest.approx((.95,0.,.25))
