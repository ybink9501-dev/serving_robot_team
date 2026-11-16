import math
import numpy as np
from scipy.spatial import cKDTree
from cafe_autonomy.occupancy_registration import register_occupancy


def test_map_surfaces_register_without_erasing_foreground_person():
    v=np.linspace(-3.,3.,150)
    walls=np.concatenate([np.column_stack((v,np.full_like(v,3.))),
                          np.column_stack((np.full_like(v,-3.),v)),
                          np.column_stack((np.full_like(v,3.),v))])
    person=np.column_stack((.2*np.cos(np.linspace(0.,6.28,20)),.2*np.sin(np.linspace(0.,6.28,20))))
    a=.03;r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    points=np.concatenate([walls,person])@r.T+np.array([.06,-.04])
    tree=cKDTree(walls)
    aligned,meta=register_occupancy(points,tree,.4,.2,.6)
    assert meta['accepted'] and meta['frame']=='map'
    assert np.median(tree.query(aligned[:len(walls)])[0])<.04
    assert np.min(tree.query(aligned[len(walls):])[0])>2.
    assert not np.shares_memory(points,aligned)


def test_missing_reference_and_large_displacement_are_not_trusted():
    points=np.column_stack((np.linspace(0.,1.,60),np.zeros(60)))
    assert not register_occupancy(points,None,.4,.2,.6)[1]['accepted']
    assert not register_occupancy(points+10.,cKDTree(points),.4,.2,.6)[1]['accepted']
