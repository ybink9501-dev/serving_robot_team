"""Robust sensor-to-occupancy registration; never changes map, TF or goals."""
import math
import numpy as np
from scipy.spatial import cKDTree


def register_occupancy(points, tree, max_shift, max_yaw, match_limit):
    identity={'accepted':False,'frame':'map'}
    if tree is None or len(points)<30:return points,identity
    original=np.asarray(points,dtype=float)
    current=original.copy();rotation=np.eye(2);translation=np.zeros(2)
    before=tree.query(original)[0]
    for _ in range(6):
        residual,indices=tree.query(current)
        valid=residual<match_limit
        if np.count_nonzero(valid)<30:return original,identity
        valid&=residual<=np.percentile(residual[valid],65)
        if np.count_nonzero(valid)<20:return original,identity
        source=current[valid];target=tree.data[indices[valid]]
        a=source.mean(axis=0);b=target.mean(axis=0)
        u,_,vt=np.linalg.svd((source-a).T@(target-b))
        r=vt.T@u.T
        if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
        t=b-r@a
        current=current@r.T+t;translation=r@translation+t;rotation=r@rotation
    angle=math.atan2(rotation[1,0],rotation[0,0])
    centre=original.mean(axis=0)
    shift=np.linalg.norm(rotation@centre+translation-centre)
    after=tree.query(current)[0]
    accepted=(shift<=max_shift and abs(angle)<=max_yaw and
              np.median(after)<=np.median(before)+1e-6 and np.median(after)<match_limit*.5)
    if not accepted:return original,identity
    return current,{'accepted':True,'frame':'map','rotation':rotation.tolist(),
                   'translation':translation.tolist(),'residual_m':float(np.median(after))}


def occupancy_tree(grid, info):
    iy,ix=np.where(grid>=65)
    if not len(ix):return None
    q=info.origin.orientation
    angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    x=(ix+.5)*info.resolution;y=(iy+.5)*info.resolution
    return cKDTree(np.column_stack((info.origin.position.x+x*math.cos(angle)-y*math.sin(angle),
                                   info.origin.position.y+x*math.sin(angle)+y*math.cos(angle))))
