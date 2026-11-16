import numpy as np
import pytest
from serving_robot_mapping.mapping_polygon import validate_polygon,contains,polygon_mask


def test_concave_operating_region_does_not_fill_notch_or_depend_on_winding():
    p=validate_polygon([[0,0],[4,0],[4,1],[1,1],[1,4],[0,4]])
    x,y=np.meshgrid(np.arange(-.25,4.5,.5),np.arange(-.25,4.5,.5))
    expected=(x>=0)&(y>=0)&(x<=4)&(y<=4)&((x<=1)|(y<=1))
    for poly in (p,list(reversed(p))):
        assert np.array_equal(polygon_mask(poly,x,y),expected)
        assert not contains(poly,2,2)
        assert contains(poly,1,2)


@pytest.mark.parametrize('p',[
    [[0,0],[2,2],[0,2],[2,0]],
    [[0,0],[1,0],[1,0],[0,1]],
    [[0,0],[1,0],[2,0]],
    [[0,0],[float('nan'),0],[0,1]],
])
def test_invalid_boundary_is_rejected(p):
    with pytest.raises(ValueError):validate_polygon(p)
