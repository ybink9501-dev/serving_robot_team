import numpy as np
import pytest
from cafe_autonomy.rgbd_projection import torso_position

K=[100.,0.,160.,0.,100.,120.,0.,0.,1.]
def test_torso_uses_supported_foreground_depth_and_matching_pixels():
    d=np.full((240,320),6.,dtype=float)
    d[50:110,130:170]=2.
    p=torso_position(d,(110,20,190,200),K,[0.,0.,0.,1.],[0.,0.,1.])
    assert abs(p[0]-2.)<1e-6
    assert abs(p[1]-.21)<.04
    assert .35<p[2]<2.3

def test_invalid_or_floor_depth_cannot_create_a_far_person():
    with pytest.raises(ValueError):torso_position(np.full((240,320),np.nan),(100,20,200,200),K,[0,0,0,1],[0,0,1])
    with pytest.raises(ValueError):torso_position(np.ones((240,320)),(100,150,200,230),K,[0,0,0,1],[0,0,0])

def test_projection_respects_camera_rotation_and_translation():
    d=np.full((240,320),2.);p=torso_position(d,(140,60,180,160),K,[0,0,2**-.5,2**-.5],[3.,4.,1.])
    assert abs(p[1]-6.)<1e-6 and abs(p[0]-3.)<.03
