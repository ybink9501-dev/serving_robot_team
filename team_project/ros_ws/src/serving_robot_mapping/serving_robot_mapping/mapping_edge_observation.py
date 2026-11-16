"""Propose safe observation centers with uncertain, thin edge occlusion.

No occupancy is changed. These are sensor experiments, never path approvals.
Unknown/uncertain intervening cells still occlude; thick occupied objects do too.
"""
import math
import numpy as np
from scipy import ndimage


def attempts_at(x,y,inspections):
    return max((int(n) for ix,iy,n in inspections if math.hypot(x-ix,y-iy)<.10),default=0)


def can_observe_in_place(robot_xy,proposal_xy,inspections):
    # Arrival identity, not the legacy 25cm visit neighborhood. A new
    # vantage 10cm away must actually be reached, never absorbed as a spin.
    return math.dist(robot_xy,proposal_xy)<=.02 and attempts_at(*robot_xy,inspections)<3


def matches_probe(node,x,y):
    return any(math.hypot(x-p['xy'][0],y-p['xy'][1])<.03 for p in getattr(node,'_edge_probe_goals',()))


def edge_ray(data,w,h,sx,sy,tx,ty,ex,ey,res):
    """DDA occupied traversal length; at most one cell diagonal for a probe."""
    dx,dy=ex-sx,ey-sy;length=math.hypot(dx,dy)*res
    xx,yy=math.floor(sx),math.floor(sy)
    stepx,stepy=(1 if dx>0 else -1),(1 if dy>0 else -1)
    ax,ay=abs(1/dx) if dx else math.inf,abs(1/dy) if dy else math.inf
    mx=((xx+1-sx)/dx if dx>0 else (xx-sx)/dx) if dx else math.inf
    my=((yy+1-sy)/dy if dy>0 else (yy-sy)/dy) if dy else math.inf
    t=0.;occupied=0.
    for _ in range(w+h+3):
        if (xx,yy)==(tx,ty):return occupied
        if not(0<=xx<w and 0<=yy<h):return None
        value=data[yy*w+xx]
        if value<0 or 25<value<65:return None
        end=min(mx,my,1.)
        if value>=65:occupied+=max(0.,end-t)*length
        if occupied>res*math.sqrt(2)+1e-9:return None
        if end>=1.:return None
        if abs(mx-my)<1e-12:xx+=stepx;yy+=stepy;mx+=ax;my+=ay
        elif mx<my:xx+=stepx;mx+=ax
        else:yy+=stepy;my+=ay
        t=end
    return None


def select_edge_observations(grid,robot_xy,goal_clear,sensor_range,inspections=(),limit=6):
    data,w,h,res,ox,oy,yaw=grid;values=np.asarray(data).reshape(h,w)
    free=(values>=0)&(values<=25);c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=robot_xy[0]-ox,robot_xy[1]-oy
    col,row=math.floor((c*dx+s*dy)/res),math.floor((-s*dx+c*dy)/res)
    if not(0<=row<h and 0<=col<w and free[row,col]):return [],{'reason':'NO_FREE_SEED'}
    labels,_=ndimage.label(free);region=labels==labels[row,col]
    border=ndimage.binary_dilation(region,structure=ndimage.generate_binary_structure(2,1))&(values<0)
    targets=[(int(x),int(y)) for y,x in zip(*np.where(border))]
    # The existing goal guard is the authority. Erosion only removes positions
    # which cannot possibly satisfy it, reducing background computation.
    distance=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*res
    possible=region&(distance>.48)
    scored=[];examined=0
    for y,x in zip(*np.where(possible)):
        wx,wy=ox+res*(c*(x+.5)-s*(y+.5)),oy+res*(s*(x+.5)+c*(y+.5))
        if not goal_clear(wx,wy)[0]:continue
        attempts=attempts_at(wx,wy,inspections)
        if attempts>=3:continue
        examined+=1;seen=[];thin=0
        for tx,ty in targets:
            if math.hypot(tx-x,ty-y)*res>sensor_range:continue
            paths=[edge_ray(data,w,h,x+.5,y+.5,tx,ty,tx+a,ty+b,res)
                   for a,b in ((.5,.5),(.05,.05),(.05,.95),(.95,.05),(.95,.95))]
            valid=[v for v in paths if v is not None]
            if valid:
                seen.append((tx,ty));thin+=int(min(valid)>0)
        if seen:
            travel=math.hypot(wx-robot_xy[0],wy-robot_xy[1])
            # Information and travel, never map/world truth, determine order.
            # Angular target size approximates LiDAR sampling opportunity.
            # A nearby safe view must beat tiny translation around a distant,
            # already unproductive view of the same occluded target.
            angular_gain=sum(2*math.atan2(res/2,max(res,math.hypot(tx-x,ty-y)*res)) for tx,ty in seen)
            score=angular_gain/(1.+.1*travel)/(1.+.35*attempts)
            scored.append((score,wx,wy,seen,thin,attempts))
    scored.sort(key=lambda p:(-p[0],p[1],p[2]));selected=[]
    for score,x,y,seen,thin,attempts in scored:
        if any(math.hypot(x-p['xy'][0],y-p['xy'][1])<.15 for p in selected):continue
        selected.append(dict(xy=[x,y],targets=[list(p) for p in seen],thin_edge_targets=thin,attempts=attempts,
            target_world=[[ox+res*(c*(tx+.5)-s*(ty+.5)),oy+res*(s*(tx+.5)+c*(ty+.5))] for tx,ty in seen],
            proposal_only=True,max_occupied_ray_length_m=res*math.sqrt(2)))
        if len(selected)>=limit:break
    return selected,dict(reason='EDGE_OBSERVATION_PROPOSALS' if selected else 'NO_EDGE_OBSERVATION_PROPOSAL',
        unknown_targets=len(targets),safe_centers_examined=examined,scored_centers=len(scored))


def observation_endpoint_matches(xy,path_points):
    """A visibility proposal must refer to the actual path destination."""
    return bool(path_points) and math.dist(xy,path_points[-1])<=.02
