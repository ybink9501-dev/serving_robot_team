"""Wall-first routes around the robot-reachable free envelope.

Contours are proposals, not motion permissions. Unknown remains blocked. Every
point still needs the existing goal guard, actual Nav2 path and live safety.
No Gazebo geometry, room dimensions, synthetic walls or velocity publisher.
"""
import math
import cv2
import numpy as np
from scipy import ndimage
from .mapping_geometry import MappingGeometry


def perimeter_routes(grid, robot_xy, spacing_m=.9):
    data,w,h,res,ox,oy,yaw=grid
    budget=MappingGeometry(resolution=res)
    if len(data)!=w*h or w<3 or h<3 or not math.isfinite(spacing_m) or spacing_m<=0:
        raise ValueError('Invalid perimeter input')
    values=np.asarray(data).reshape(h,w)
    free=(values>=0)&(values<=25)
    distance=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*res
    safe=free & (distance>budget.perimeter_radius)
    co,si=math.cos(yaw),math.sin(yaw);dx,dy=robot_xy[0]-ox,robot_xy[1]-oy
    sx,sy=math.floor((co*dx+si*dy)/res),math.floor((-si*dx+co*dy)/res)
    if not (0<=sx<w and 0<=sy<h):raise ValueError('Robot outside perimeter map')
    labels,_=ndimage.label(safe)
    yy,xx=np.where(safe)
    nearby=(xx-sx)**2+(yy-sy)**2
    if not len(nearby) or float(nearby.min())*res*res>.8**2:
        return [],dict(reason='NO_SAFE_SEED_NEAR_ROBOT',perimeter_radius_m=budget.perimeter_radius)
    k=int(nearby.argmin());component=(labels==labels[yy[k],xx[k]]).astype('uint8')
    contours,hierarchy=cv2.findContours(component,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:return [],dict(reason='NO_REACHABLE_CONTOUR')
    records=[]
    for index,contour in enumerate(contours):
        points=contour.reshape(-1,2).astype(float)
        if len(points)<4:continue
        length=float(cv2.arcLength(contour,True))*res
        if length<2*spacing_m:continue
        outer=int(hierarchy[0,index,3])==-1
        area=float(cv2.contourArea(contour,oriented=True))
        if (area>0)!=outer:points=points[::-1]
        world=[(ox+res*(co*(x+.5)-si*(y+.5)),oy+res*(si*(x+.5)+co*(y+.5))) for x,y in points]
        first=min(range(len(world)),key=lambda i:math.dist(world[i],robot_xy))
        world=world[first:]+world[:first]
        samples=[world[0]];accum=0.
        for a,b in zip(world,world[1:]):
            accum+=math.dist(a,b)
            if accum>=spacing_m:samples.append(b);accum=0.
        if len(samples)<3:continue
        if math.dist(samples[-1],samples[0])<spacing_m/2:samples.pop()
        samples.append(samples[0])  # Explicitly validate return to wall start.
        records.append(dict(kind='OUTER_WALL' if outer else 'FIXED_OBSTACLE',points=samples,length_m=length))
    records.sort(key=lambda r:(r['kind']!='OUTER_WALL',math.dist(r['points'][0],robot_xy)))
    return records,dict(reason='PERIMETER_PROPOSALS',safe_component_cells=int(component.sum()),
                        contour_count=len(records),perimeter_radius_m=budget.perimeter_radius)


class PerimeterPlan:
    def __init__(self, routes):
        self.routes=routes;self.loop=0;self.point=0;self.active=None
        self.ledger=[];self.closed_loops=[];self.outer_return_failed=False
        self.anchors={};self.anchor_indices={};self.reached={}

    @property
    def finished(self):return self.loop>=len(self.routes) or self.outer_return_failed

    def matches(self,x,y):
        return self.active is not None and math.dist(self.active,(x,y))<.03

    def _advance(self):
        self.point+=1;self.active=None
        if self.point>=len(self.routes[self.loop]['points']):self.loop+=1;self.point=0

    def candidate(self, goal_clear, rejected, revision):
        while not self.finished:
            route=self.routes[self.loop];point=tuple(route['points'][self.point])
            if self.point==len(route['points'])-1 and route['kind']!='DEFERRED_OBSERVATION':
                point=self.anchors.get(self.loop,point)
            key=tuple(round(v,2) for v in point)
            allowed,reason=goal_clear(*point)
            if key in rejected:allowed,reason=False,'ACTUAL_PATH_OR_NAV2_REJECTED'
            if allowed:self.active=point;return [point]
            self.ledger.append(dict(loop=self.loop,point=self.point,xy=point,status='DEFERRED',reason=reason,revision=revision))
            if self.point==len(route['points'])-1 and route['kind']=='OUTER_WALL':
                self.outer_return_failed=True;self.active=None;return []
            self._advance()
        return []

    def arrived(self,x,y,revision):
        if self.finished or not self.matches(x,y):return False
        route=self.routes[self.loop]
        if self.loop not in self.anchors:
            self.anchors[self.loop]=self.active;self.anchor_indices[self.loop]=self.point
        self.reached.setdefault(self.loop,set()).add(tuple(self.active))
        self.ledger.append(dict(loop=self.loop,point=self.point,xy=self.active,status='OBSERVATION_POSITION_REACHED',revision=revision))
        if self.point==len(route['points'])-1 and route['kind']!='DEFERRED_OBSERVATION':
            if len(self.reached[self.loop])>=3:self.closed_loops.append(self.loop)
            elif route['kind']=='OUTER_WALL':self.outer_return_failed=True;return True
        self._advance();return True

    def tangent_yaw(self):
        if self.finished or self.active is None:return 0.
        route=self.routes[self.loop];points=route['points']
        if self.point<len(points)-1:other=points[self.point+1]
        elif route['kind']!='DEFERRED_OBSERVATION':other=points[min(self.anchor_indices.get(self.loop,0)+1,len(points)-1)]
        elif self.point:other=(2*self.active[0]-points[self.point-1][0],2*self.active[1]-points[self.point-1][1])
        else:return 0.
        return math.atan2(other[1]-self.active[1],other[0]-self.active[0])

    def retry_deferred(self, revision):
        if self.outer_return_failed:
            last=self.ledger[-1] if self.ledger else {}
            if last.get('revision')!=revision:self.outer_return_failed=False
            return
        if not self.finished:return
        latest={tuple(r['xy']):r for r in self.ledger}
        pending=[xy for xy,r in latest.items() if r['status']=='DEFERRED' and r.get('revision')!=revision]
        if pending:self.routes.append(dict(kind='DEFERRED_OBSERVATION',points=pending,length_m=None))

    def snapshot(self):
        return dict(phase='OUTER_RETURN_BLOCKED' if self.outer_return_failed else
                    'FRONTIER_REFINEMENT' if self.finished else self.routes[self.loop]['kind'],
                    loop=self.loop,point=self.point,closed_loops=self.closed_loops,
                    deferred=[r for r in self.ledger if r['status']=='DEFERRED'],
                    ledger=self.ledger,map_complete=False)
