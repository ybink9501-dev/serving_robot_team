from types import SimpleNamespace
import numpy as np
import pytest
from serving_robot_mapping.mapping_camera_evidence import decode_image


def test_depth_stride_and_endianness_preserve_real_pixels():
    data=np.array([[1.,2.,999.],[3.,4.,999.]],dtype='>f4')
    msg=SimpleNamespace(encoding='32FC1',width=2,height=2,step=12,is_bigendian=True,data=data.tobytes())
    assert decode_image(msg).tolist()==[[1.,2.],[3.,4.]]


def test_color_stride_and_invalid_frame_are_not_silently_invented():
    msg=SimpleNamespace(encoding='bgr8',width=1,height=1,step=4,is_bigendian=False,data=bytes([3,2,1,255]))
    assert decode_image(msg).tolist()==[[[1,2,3]]]
    msg.step=2
    with pytest.raises(ValueError):decode_image(msg)
