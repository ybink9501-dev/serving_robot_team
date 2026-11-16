"""Pixel-consistent torso projection; reject floor/background depth mixtures."""
import numpy as np

def torso_position(depth,box,k,quaternion,translation):
    x1,y1,x2,y2=box;h,w=depth.shape
    x1,x2=max(0,int(x1)),min(w,int(x2));y1,y2=max(0,int(y1)),min(h,int(y2))
    left=int(x1+.25*(x2-x1));right=int(x1+.75*(x2-x1))
    top=int(y1+.20*(y2-y1));bottom=int(y1+.55*(y2-y1))
    if right-left<2 or bottom-top<2:raise ValueError('Person torso has insufficient pixels')
    vv,uu=np.mgrid[top:bottom,left:right];dd=depth[top:bottom,left:right]
    valid=np.isfinite(dd)&(dd>.1)&(dd<8.)
    d=dd[valid];u=uu[valid];v=vv[valid]
    if len(d)<4:raise ValueError('Person torso has no usable depth')
    q=np.column_stack((d,-(u-k[2])*d/k[0],-(v-k[5])*d/k[4]))
    xyz=np.asarray(quaternion[:3]);qw=quaternion[3]
    world=q+2*np.cross(xyz,np.cross(xyz,q)+qw*q)+np.asarray(translation)
    valid=(world[:,2]>.35)&(world[:,2]<2.3)
    world=world[valid];d=d[valid]
    if len(d)<4:raise ValueError('Person depth lies outside body-height range')
    bins=np.floor(d/.15).astype(int);counts=np.bincount(bins)
    supported=np.flatnonzero(counts>=max(4,int(len(d)*.15)))
    if not len(supported):raise ValueError('Person torso depth has no supported mode')
    mode=supported[0];centre=(mode+.5)*.15
    selected=np.abs(d-centre)<=.2
    return np.median(world[selected],axis=0)


def conservative_person_geometry(person, centre, radius, robot, body_radius):
    """Fuse associated LiDAR body geometry without enlarging free space.

    A partial visible arc needs a body-centre offset. A fitted full body arc
    already provides its centre. RGB-D retains classification, not precedence
    over a closer measured physical surface.
    """
    import math
    x,y=map(float,centre);d=math.hypot(x-robot[0],y-robot[1])
    if radius<body_radius*.7 and d>0:
        x+=(x-robot[0])*body_radius/d;y+=(y-robot[1])*body_radius/d
    body=max(body_radius,float(radius))
    old_gap=math.hypot(person[0]-robot[0],person[1]-robot[1])-person[2]
    new_gap=math.hypot(x-robot[0],y-robot[1])-body
    return (x,y,body,'person') if new_gap<old_gap else person


def lidar_corroborates_person(candidate, centre, radius, fitted, count, association):
    """A lower-confidence camera candidate needs an actual foreground body/leg arc.

    Linear walls, unmatched positions, and clusters without a supported circle
    fit never promote a camera candidate into a person.
    """
    import math
    return bool(fitted and count>=5 and .08<=radius<=.40
                and math.hypot(candidate[0]-centre[0],candidate[1]-centre[1])<=association)
