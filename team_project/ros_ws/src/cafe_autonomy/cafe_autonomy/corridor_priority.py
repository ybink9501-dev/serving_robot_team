"""Map-local corridor presence order, with durable request order as tie-break."""
import math
from .policy import in_bounds
from .yield_policy import yield_goal_clear

def portals(bounds):
    x0,x1,y0,y1=bounds
    return ((x0,(y0+y1)/2),(x1,(y0+y1)/2)) if x1-x0>=y1-y0 else (((x0+x1)/2,y0),((x0+x1)/2,y1))

def distance_to_portal(position,bounds):
    x,y=position[:2];x0,x1,y0,y1=bounds
    return math.hypot(max(x0-x,0.,x-x1),max(y0-y,0.,y-y1))

def forward_entry_observed(position,heading,bounds,report,grid,recognition,half_angle,path=None):
    if not all(report.get(k) is True for k in ('sensor_fresh','camera_fresh','lidar_fresh')):return False
    entry=entry_on_path(path or [],bounds)
    if entry is None:entry=(min(bounds[1],max(bounds[0],position[0])),min(bounds[3],max(bounds[2],position[1])))
    d=math.dist(position[:2],entry)
    if not 0.<d<=recognition:return False
    bearing=math.atan2(entry[1]-position[1],entry[0]-position[0])-heading
    if abs(math.atan2(math.sin(bearing),math.cos(bearing)))>half_angle:return False
    if report.get('front_clear_m',0.)+0.1<d:return False
    if grid is None:return False
    steps=max(1,math.ceil(d/max(.01,grid.info.resolution/2)))
    return all(yield_goal_clear(grid,(position[0]+(entry[0]-position[0])*i/steps,position[1]+(entry[1]-position[1])*i/steps),0.) for i in range(steps+1))

class CorridorArrivalPriority:
    def __init__(self):self.entries={};self.batch=0;self.map_key=None
    def reset(self,map_key):
        self.entries.clear();self.batch=0;self.map_key=map_key
    def observe(self,poses,zones,now,margin):
        self.batch+=1
        for zone,bounds in zones.items():
            seen=self.entries.setdefault(zone,{})
            for robot,position in poses.items():
                present=in_bounds(position,bounds) or distance_to_portal(position,bounds)<=margin
                if present:seen.setdefault(robot,{'batch':self.batch,'observed_at':now})
                else:seen.pop(robot,None)
    def choose(self,zone,robots,request_sequences,poses,bounds):
        inside=[r for r in robots if in_bounds(poses.get(r),bounds)]
        available=[r for r in robots if r in self.entries.get(zone,{})]
        if len(inside)==1:winner=inside[0];reason='ALREADY_INSIDE'
        elif not available:return None
        else:
            earliest=min(self.entries[zone][r]['batch'] for r in available)
            tied=[r for r in available if self.entries[zone][r]['batch']==earliest]
            if len(tied)==1:winner=tied[0];reason='FIRST_ARRIVAL'
            else:
                if any(request_sequences.get(r) is None for r in tied):return None
                order=min(request_sequences[r] for r in tied)
                candidates=[r for r in tied if request_sequences[r]==order]
                if len(candidates)!=1:return None
                winner=candidates[0];reason='EARLIER_REQUEST'
        return {'zone':zone,'winner':winner,'reason':reason,
            'arrivals':{r:dict(self.entries.get(zone,{}).get(r,{})) for r in robots},
            'request_sequences':dict(request_sequences),'map_key':self.map_key}

def priority_yield_options(options,priority):
    return [o for o in options if not priority or o['robot']!=priority['winner']]


def entry_on_path(path,bounds):
    for a,b in zip(path,path[1:]):
        lo,hi=0.,1.
        for axis in (0,1):
            lower,upper=bounds[axis*2:axis*2+2];delta=b[axis]-a[axis]
            if abs(delta)<1e-9:
                if not lower<=a[axis]<=upper:hi=-1.;break
            else:
                t0,t1=sorted(((lower-a[axis])/delta,(upper-a[axis])/delta));lo=max(lo,t0);hi=min(hi,t1)
        if lo<=hi:return (a[0]+lo*(b[0]-a[0]),a[1]+lo*(b[1]-a[1]))
    return None


def reservation_intents(poses,goals,paths,contexts,expected,route_times,now):
    """Plan absence may be caused by the other robot at the entrance.

    Goal intent permits arbitration, never velocity or unverified path execution.
    A valid current Nav2 route always takes precedence, including a bypass route.
    """
    routes={};sources={}
    for robot,position in poses.items():
        goal=goals.get(robot)
        if not goal:continue
        if contexts.get(robot)==expected.get(robot) and len(paths.get(robot,[]))>=2:
            routes[robot]=paths[robot]
            sources[robot]='NAV2_PATH' if now-route_times.get(robot,-1e9)<2. else 'SAME_GOAL_PATH_INTENT'
        else:
            routes[robot]=[tuple(position[:2]),tuple(goal[:2])];sources[robot]='GOAL_INTENT_ONLY'
    return routes,sources
