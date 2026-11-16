"""Mapping-only SE(2) footprint checks. No motion, map edits or ROS nodes."""
import math
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class FootprintResult:
    allowed: bool
    reason: str
    samples: int = 0
    failure_point: tuple | None = None
    risk_cell: tuple | None = None


class OrientedMappingGuard:
    # Chassis +/- .26 x .20; unchanged rectangular HardStop expansion .10.
    # The .01 tracking budget is the existing mapping arrival tolerance.
    def __init__(self, half_length=.26, half_width=.20, hard_stop=.10, tracking=.01):
        if any(not math.isfinite(v) or v<=0 for v in (half_length,half_width,hard_stop,tracking)):
            raise ValueError('Invalid oriented mapping geometry')
        self.hx=half_length+hard_stop+tracking;self.hy=half_width+hard_stop+tracking
        self.radius=math.hypot(self.hx,self.hy);self.grid=None;self.tree=None

    def update_grid(self,data,w,h,res,ox,oy,yaw=0.):
        if (not isinstance(w,int) or not isinstance(h,int) or min(w,h)<1 or len(data)!=w*h
                or not all(math.isfinite(v) for v in (res,ox,oy,yaw)) or res<=0
                or any(v < -1 or v > 100 for v in data)):
            self.grid=None;return False
        self.grid=(tuple(data),w,h,float(res),float(ox),float(oy),float(yaw))
        self.ids=np.flatnonzero((np.asarray(data)<0)|(np.asarray(data)>25))
        self.centers=np.c_[(self.ids%w+.5)*res,(self.ids//w+.5)*res]
        self.tree=cKDTree(self.centers) if len(self.ids) else None
        return True

    def pose_clear(self,x,y,yaw,pad=0.):
        if self.grid is None:return FootprintResult(False,'ORIENTED_MAP_NOT_READY')
        if not all(math.isfinite(v) for v in (x,y,yaw,pad)) or pad<0:return FootprintResult(False,'ORIENTED_POSE_INVALID')
        data,w,h,res,ox,oy,mapyaw=self.grid;c,s=math.cos(mapyaw),math.sin(mapyaw)
        dx,dy=x-ox,y-oy;mx,my=c*dx+s*dy,-s*dx+c*dy
        angle=yaw-mapyaw;ca,sa=math.cos(angle),math.sin(angle);hx,hy=self.hx+pad,self.hy+pad
        ex,ey=abs(ca)*hx+abs(sa)*hy,abs(sa)*hx+abs(ca)*hy
        if mx-ex<=0 or mx+ex>=w*res or my-ey<=0 or my+ey>=h*res:
            return FootprintResult(False,'ORIENTED_OUTSIDE_MAP',failure_point=(x,y,yaw))
        if self.tree is None:return FootprintResult(True,'ORIENTED_POSE_CLEAR')
        indices=self.tree.query_ball_point((mx,my),math.hypot(hx,hy)+res/math.sqrt(2)+1e-10)
        if not indices:return FootprintResult(True,'ORIENTED_POSE_CLEAR')
        indices=np.asarray(indices,dtype=int);delta=self.centers[indices]-[mx,my];half=res/2
        # Separating axes: two grid-square axes and two body axes. Touch blocks.
        intersects=(abs(delta[:,0])<=ex+half)&(abs(delta[:,1])<=ey+half)
        intersects&=(abs(ca*delta[:,0]+sa*delta[:,1])<=hx+half*(abs(ca)+abs(sa)))
        intersects&=(abs(-sa*delta[:,0]+ca*delta[:,1])<=hy+half*(abs(ca)+abs(sa)))
        hits=indices[intersects]
        if len(hits):
            i=int(self.ids[int(hits[0])]);v=data[i]
            kind='UNKNOWN' if v<0 else 'OCCUPIED' if v>=65 else 'UNCERTAIN'
            return FootprintResult(False,'ORIENTED_NEAR_'+kind,failure_point=(x,y,yaw),risk_cell=(i%w,i//w,v))
        return FootprintResult(True,'ORIENTED_POSE_CLEAR')

    def check_path(self,poses,forward_only=True):
        try:poses=[tuple(map(float,p)) for p in poses]
        except (TypeError,ValueError):return FootprintResult(False,'ORIENTED_PATH_INVALID')
        if len(poses)<2 or any(len(p)!=3 or not all(map(math.isfinite,p)) for p in poses):
            return FootprintResult(False,'ORIENTED_PATH_INVALID')
        count=0
        for a,b in zip(poses,poses[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            da=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            if forward_only and length>.01 and (dx*math.cos(a[2])+dy*math.sin(a[2]) < -1e-4
                    or dx*math.cos(b[2])+dy*math.sin(b[2]) < -1e-4):
                return FootprintResult(False,'ORIENTED_REVERSE_PATH')
            n=max(1,math.ceil(length/.01),math.ceil(abs(da)/.01))
            # Any continuous intermediate corner is within this distance of
            # the nearest sample. Inflate samples by that exact upper bound.
            pad=length/(2*n)+self.radius*abs(da)/(2*n)+1e-9
            for k in range(n+1):
                t=k/n;p=(a[0]+dx*t,a[1]+dy*t,a[2]+da*t);count+=1
                result=self.pose_clear(*p,pad=pad)
                if not result.allowed:return FootprintResult(False,result.reason,count,p,result.risk_cell)
        return FootprintResult(True,'ORIENTED_PATH_CLEAR',count)

    def rotation_clear(self,pose,target_yaw):
        return self.check_path([pose,(pose[0],pose[1],target_yaw)],forward_only=False)

    def scan_clear(self,scan,age,scan_pose=(0.,0.,0.)):
        if scan is None or not math.isfinite(age) or age<0 or age>.3:return False,'ORIENTED_SCAN_STALE'
        ranges=scan.ranges;increment=scan.angle_increment
        if (len(ranges)<12 or not math.isfinite(increment) or increment<=0
                or abs(increment)*(len(ranges)-1)<2*math.pi-.05):return False,'ORIENTED_SCAN_COVERAGE'
        if not all(math.isfinite(v) for v in (*scan_pose,scan.angle_min,scan.range_min,scan.range_max)):
            return False,'ORIENTED_SCAN_INVALID'
        invalid=0
        for i,d in enumerate(ranges):
            if math.isnan(d) or d<scan.range_min:invalid+=1;continue
            if math.isinf(d):
                if d<0:invalid+=1;continue
                d=scan.range_max
            if d>scan.range_max:invalid+=1;continue
            angle=scan.angle_min+i*increment+scan_pose[2]
            px=scan_pose[0]+d*math.cos(angle);py=scan_pose[1]+d*math.sin(angle)
            # Beam angular sampling uncertainty at the safety boundary.
            beam_pad=self.radius*increment+.002
            if abs(px)<=self.hx+beam_pad and abs(py)<=self.hy+beam_pad:
                return False,'ORIENTED_RAW_FOOTPRINT_BLOCKED'
        if invalid>len(ranges)*.05:return False,'ORIENTED_SCAN_INSUFFICIENT'
        return True,'ORIENTED_RAW_CLEAR'


def mapping_oriented_parameters(nav):
    """Copy-on-write configuration; serving input is never edited."""
    import copy,json
    nav=copy.deepcopy(nav);guard=OrientedMappingGuard()
    footprint=json.dumps([[.26,.20],[.26,-.20],[-.26,-.20],[-.26,.20]])
    for key in ('global_costmap','local_costmap'):
        # Add 1cm beyond the .11m measured-body envelope so that a planner
        # lattice pose cannot hide a collision between its discrete samples.
        params=nav[key][key]['ros__parameters'];params['footprint']=footprint;params['footprint_padding']=.12
    nav['planner_server']['ros__parameters']['GridBased']={
        'plugin':'nav2_smac_planner::SmacPlannerHybrid','allow_unknown':False,'tolerance':.01,
        'motion_model_for_search':'DUBIN','minimum_turning_radius':.4,'angle_quantization_bins':144,
        'downsample_costmap':False,'smooth_path':False,'max_planning_time':3.,
        'allow_primitive_interpolation':True}
    nav['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']=.03
    return nav
