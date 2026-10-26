"""Pure, ROS-independent geometry and motion policy. Uses project SDF, never inferred personal maps."""
import math
import xml.etree.ElementTree as ET
from collections import deque


def angle(a,b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def model_collision_boxes(world_file, model_name):
    root=ET.parse(world_file).getroot()
    for model in root.iter('model'):
        if model.get('name') != model_name:continue
        parts=[float(x) for x in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        mx,my=parts[0:2];theta=parts[5]
        boxes=[]
        for c in model.findall('.//collision'):
            dims=c.find('./geometry/box/size')
            if dims is None:continue
            d=[float(x) for x in dims.text.split()]
            p=[float(x) for x in (c.findtext('pose') or '0 0 0 0 0 0').split()]
            x=mx + math.cos(theta)*p[0]-math.sin(theta)*p[1]
            y=my + math.sin(theta)*p[0]+math.cos(theta)*p[1]
            boxes.append((x,y,theta+p[5],d[0],d[1]))
        if not boxes:raise ValueError('No collision boxes in counter model')
        return (mx,my,theta),boxes
    raise ValueError(f'Missing required team SDF model: {model_name}')


def distance_from_rotated_box(point, box):
    cx,cy,a,w,h=box
    dx,dy=point[0]-cx,point[1]-cy
    x=math.cos(a)*dx+math.sin(a)*dy
    y=-math.sin(a)*dx+math.cos(a)*dy
    ex=max(abs(x)-w/2,0.0)
    ey=max(abs(y)-h/2,0.0)
    outside=math.hypot(ex,ey)
    if ex==ey==0.0:return -min(w/2-abs(x),h/2-abs(y))
    return outside


def inspect_pickup(world_file, counter_model, pose, clearance_radius, min_body_gap, max_face_error_deg):
    center,boxes=model_collision_boxes(world_file,counter_model)
    x,y,yaw=map(float,(pose['x'],pose['y'],pose['yaw']))
    if not all(math.isfinite(v) for v in (x,y,yaw,clearance_radius)):
        raise ValueError('Nonfinite waypoint values')
    min_distance=min(distance_from_rotated_box((x,y),b) for b in boxes)
    body_clearance=min_distance-clearance_radius
    cx,cy,_=center
    facing=abs(math.degrees(angle(math.atan2(cy-y,cx-x),yaw)))
    return {'goal_map':{'x':x,'y':y,'yaw':yaw},'counter_world_center':[cx,cy],
            'distance_from_counter_collision_m':round(min_distance,5),
            'body_clearance_estimate_m':round(body_clearance,5),
            'counter_facing_error_deg':round(facing,3),
            'pass_static':body_clearance>=min_body_gap and facing<=max_face_error_deg,
            'is_actual_service_position_approved':False}


class StallWindow:
    """Detect long periods of virtually no translation, not merely small net displacement."""
    def __init__(self, seconds=30., progress=.08, travel=.15):
        if not(5<=seconds<=600 and 0<progress<=travel):raise ValueError('Invalid stall thresholds')
        self.seconds=seconds;self.progress=progress;self.travel=travel
        self.samples=deque();self.path_length=0.0
        self.prev=None
    def clear(self):
        self.samples.clear();self.path_length=0.0;self.prev=None
    def observe(self,t,x,y):
        if not all(math.isfinite(v) for v in (t,x,y)):
            self.clear();return None
        if self.prev and (t<=self.prev[0] or t-self.prev[0]>3.0):self.clear()
        if self.prev:self.path_length+=math.hypot(x-self.prev[1],y-self.prev[2])
        self.prev=(t,x,y)
        self.samples.append((t,x,y,self.path_length))
        while len(self.samples)>1 and t-self.samples[0][0]>self.seconds:
            self.samples.popleft()
        if len(self.samples)<2 or t-self.samples[0][0]<self.seconds-.25:return None
        start=self.samples[0]
        elapsed=t-start[0]
        travel=self.path_length-start[3]
        net=math.hypot(x-start[1],y-start[2])
        return {'window_s':round(elapsed,2),'travel_m':round(travel,4),
                'net_m':round(net,4),'stalled':travel<self.travel and net<self.progress}
