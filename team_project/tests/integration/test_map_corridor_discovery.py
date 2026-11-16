import math
from types import SimpleNamespace as NS
from cafe_autonomy.map_corridors import discover_corridors

def scene(angle=0.,translation=(0.,0.),unknown=False):
    width,height,res=160,80,.05;data=[-1 if unknown else 0]*(width*height)
    for iy in range(height):
        y=-2+(iy+.5)*res
        for ix in range(width):
            x=-4+(ix+.5)*res
            if -1.05<x<1.05 and .5<abs(y)<.65:data[iy*width+ix]=100
    c,s=math.cos(angle),math.sin(angle)
    def transform(p):return (translation[0]+c*p[0]-s*p[1],translation[1]+s*p[0]+c*p[1])
    ox,oy=transform((-4.,-2.))
    grid=NS(info=NS(width=width,height=height,resolution=res,origin=NS(position=NS(x=ox,y=oy),orientation=NS(x=0.,y=0.,z=math.sin(angle/2),w=math.cos(angle/2)))),data=data)
    return grid,[transform((-3+i*.05,0.)) for i in range(121)]

def test_narrow_corridor_is_derived_from_two_known_map_walls():
    g,p=scene();regions=discover_corridors(g,p,.66,1.8)
    assert regions and any(b[0]<-.7 and b[1]>.7 and b[2]<-.45 and b[3]>.45 for b in regions)

def test_new_map_origin_and_rotation_require_no_fixed_corridor_coordinates():
    g,p=scene(math.pi/2,(10.,20.));regions=discover_corridors(g,p,.66,1.8)
    assert regions and any(b[0]<9.55 and b[1]>10.45 and b[2]<19.3 and b[3]>20.7 for b in regions)

def test_unknown_space_cannot_establish_corridor_width():
    g,p=scene(unknown=True);assert not discover_corridors(g,p,.66,1.8)


def test_discovery_does_not_duplicate_or_replace_registered_corridor_geometry():
    from cafe_autonomy.map_corridors import same_corridor
    g,p=scene();regions=discover_corridors(g,p,.66,1.8)
    assert regions and all(same_corridor(b,[-1.2,1.2,-.65,.65]) for b in regions)
    assert not same_corridor(regions[0],[5.,6.,5.,6.])
