"""Discover known narrow passage segments on actual Nav2 paths from a map.

Only two observed occupied walls qualify. Unknown cells cannot establish width.
This supplies map-local traffic regions; it never generates robot velocity.
"""
import math

def discover_corridors(grid,path,min_width,max_width,spacing=.15):
    if grid is None or len(path)<3:return []
    info=grid.info;res=info.resolution
    if res<=0 or not 0<min_width<max_width:return []
    q=info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c=math.cos(yaw);s=math.sin(yaw)
    def cell(x,y):
        dx=x-info.origin.position.x;dy=y-info.origin.position.y
        ix=math.floor((c*dx+s*dy)/res);iy=math.floor((-s*dx+c*dy)/res)
        if not 0<=ix<info.width or not 0<=iy<info.height:return None
        v=grid.data[iy*info.width+ix]
        return None if v<0 else v
    points=[]
    for p in path:
        if not all(math.isfinite(v) for v in p[:2]):return []
        if not points or math.dist(p[:2],points[-1])>=spacing:points.append(tuple(p[:2]))
    step=max(.025,res)
    def wall(p,n,sign):
        for i in range(1,math.ceil(max_width/step)+1):
            d=i*step;x=p[0]+n[0]*sign*d;y=p[1]+n[1]*sign*d;v=cell(x,y)
            if v is None:return None
            if v>=65:return d,(x,y)
        return None
    regions=[];run=[]
    def flush():
        if len(run)>=3 and sum(math.dist(a[0],b[0]) for a,b in zip(run,run[1:]))>=.3:
            hits=[p for item in run for p in item[2:]]
            bounds=[min(p[0] for p in hits),max(p[0] for p in hits),min(p[1] for p in hits),max(p[1] for p in hits)]
            if bounds[1]-bounds[0]>=res and bounds[3]-bounds[2]>=res:regions.append(bounds)
        run.clear()
    for i in range(1,len(points)-1):
        p=points[i];dx=points[i+1][0]-points[i-1][0];dy=points[i+1][1]-points[i-1][1];length=math.hypot(dx,dy)
        if length<1e-6:flush();continue
        direction=(dx/length,dy/length);normal=(-direction[1],direction[0])
        left=wall(p,normal,1);right=wall(p,normal,-1)
        valid=left and right and min_width<=left[0]+right[0]<=max_width
        if not valid:flush();continue
        if run and (math.dist(run[-1][0],p)>spacing*3 or sum(a*b for a,b in zip(run[-1][1],direction))<.94):flush()
        run.append((p,direction,left[1],right[1]))
    flush();return regions

def same_corridor(a,b):
    overlap=max(0.,min(a[1],b[1])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[2],b[2]))
    area=min((a[1]-a[0])*(a[3]-a[2]),(b[1]-b[0])*(b[3]-b[2]))
    return area>0 and overlap/area>=.6
