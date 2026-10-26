"""Pure helpers for Robot 01 simulation-only localization recovery."""
import math
import re

FLOAT=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
POSE_RE=re.compile(r'Pose\s*\[\s*XYZ\s*\(m\)\s*\]\s*\[\s*RPY\s*\(rad\)\s*\]\s*:\s*\[\s*([^]]+)\]\s*\[\s*([^]]+)\]',re.I|re.S)

def parse_gz_model_pose(output):
    m=POSE_RE.search(output)
    if not m:
        raise ValueError('Gazebo model pose not found in gz model output')
    def values(s):
        parts=re.findall(FLOAT,s)
        if len(parts)!=3:raise ValueError('Expected three pose components')
        vals=tuple(float(x) for x in parts)
        if not all(math.isfinite(x) for x in vals):raise ValueError('Nonfinite Gazebo pose')
        return vals
    xyz=values(m.group(1));rpy=values(m.group(2))
    return xyz[0],xyz[1],rpy[2]

def angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))

def pose_difference(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1]),abs(math.degrees(angle_delta(a[2],b[2])))

def quaternion_yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def map_cell(x,y,info):
    res=float(info.resolution);yaw=quaternion_yaw(info.origin.orientation)
    if res<=0:raise ValueError('Bad map resolution')
    dx=x-info.origin.position.x;dy=y-info.origin.position.y
    mx=math.cos(yaw)*dx+math.sin(yaw)*dy
    my=-math.sin(yaw)*dx+math.cos(yaw)*dy
    return math.floor(mx/res), math.floor(my/res)

def footprint_free(grid,x,y,radius=0.32):
    # Require full clearance for simulated robot at the proposed AMCL pose.
    for dx,dy in [(0,0),(radius,0),(-radius,0),(0,radius),(0,-radius),
                  (radius*.707,radius*.707),(-radius*.707,radius*.707),
                  (radius*.707,-radius*.707),(-radius*.707,-radius*.707)]:
        ix,iy=map_cell(x+dx,y+dy,grid.info)
        if ix<0 or iy<0 or ix>=grid.info.width or iy>=grid.info.height:return False
        occ=grid.data[iy*grid.info.width+ix]
        if occ<0 or occ>=50:return False
    return True

def allowed_state(obj):
    return obj.get('robot_id')=='robot_01' and obj.get('state') in ('CANCELED','IDLE','ERROR','COMPLETED') and obj.get('map_valid') is True
