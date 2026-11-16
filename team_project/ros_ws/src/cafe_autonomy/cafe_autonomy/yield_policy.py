"""Geometry-only candidate screening for two-robot traffic yields.

Nav2 must still approve each candidate path before a robot receives a YIELD.
No candidate is a HOME waypoint or a command to reverse the robot.
"""
import math


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def in_rectangle(point, bounds, margin=0.0):
    x0, x1 = sorted(bounds[:2])
    y0, y1 = sorted(bounds[2:])
    return x0 - margin <= point[0] <= x1 + margin and y0 - margin <= point[1] <= y1 + margin


def _angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def candidate_poses(here, peer, history, zones, no_go, homes, peer_goal,
                    separation, zone_margin):
    """Offer recent clear positions and nearby exits; never declare them safe."""
    x, y, heading = here
    options = [p[:2] for p in reversed(history)]
    for bounds in zones.values():
        if in_rectangle(here, bounds):
            x0, x1 = sorted(bounds[:2]); y0, y1 = sorted(bounds[2:])
            offset = zone_margin + separation / 2
            options.extend(((x0 - offset, y), (x1 + offset, y),
                            (x, y0 - offset), (x, y1 + offset)))
    away = math.atan2(y - peer[1], x - peer[0])
    for radius in (separation * 1.5, separation * 2):
        for turn in (0, math.pi / 2, -math.pi / 2):
            angle = away + turn
            options.append((x + radius * math.cos(angle),
                            y + radius * math.sin(angle)))
    result = []
    for point in options:
        if any(distance(point, other[:2]) < separation / 2 for other in result):
            continue
        if distance(point, here) < separation:
            continue
        if distance(point, here) > 4 * separation:
            continue
        if distance(point, peer) < separation + zone_margin:
            continue
        if peer_goal and distance(point, peer_goal) < separation + zone_margin:
            continue
        if any(distance(point, home) < separation for home in homes):
            continue
        if any(in_rectangle(point, b, zone_margin) for b in zones.values()):
            continue
        if any(in_rectangle(point, b) for b in no_go.values()):
            continue
        direction = math.atan2(point[1] - y, point[0] - x)
        result.append((point[0], point[1], direction,
                       abs(_angle(direction - heading)) > math.pi / 2))
    return result


def segment_distance(point, a, b):
    dx = b[0] - a[0]; dy = b[1] - a[1]
    length_sq = dx * dx + dy * dy
    fraction = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                                    (point[1] - a[1]) * dy) / length_sq)) if length_sq else 0.0
    return distance(point, (a[0] + fraction * dx, a[1] + fraction * dy))


def assess_path(points, here, candidate, peer, separation, start_tolerance,
                goal_tolerance, contact_clearance=0.0):
    """Reject a stale, incomplete, or peer-intersecting Nav2 path."""
    if len(points) < 2 or distance(points[0], here) > start_tolerance:
        return None
    if distance(points[-1], candidate) > goal_tolerance:
        return None
    clearance = min(segment_distance(peer, a, b) for a, b in zip(points, points[1:]))
    initial=distance(points[0],peer)
    escaping=initial<separation
    if escaping:
        if initial<contact_clearance:return None
        recovered=False
        for a,b in zip(points,points[1:]):
            before=distance(a,peer);after=distance(b,peer)
            required=separation if recovered else before
            if segment_distance(peer,a,b)+1e-6<required:return None
            if not recovered and after+1e-6<before:return None
            recovered=recovered or after>=separation
        if not recovered:return None
    elif clearance<separation:return None
    length = sum(distance(a, b) for a, b in zip(points, points[1:]))
    return {"length_m": length, "peer_clearance_m": clearance, "escape_from_close_start":escaping,"initial_peer_clearance_m":initial}


def choose_yield(options, last_yielder=None):
    """Prefer feasible front/side motion, then the shorter safe route."""
    if not options:
        return None
    return min(options, key=lambda o: (bool(o["rear_turn"]), o["length_m"],
                                       o["turn_rad"], o["robot"] == last_yielder))


def yield_goal_clear(grid, point, radius):
    """A yield endpoint needs known free space for final approach/rotation.

    Nav2's path planner can accept a goal whose point is free but whose
    final body envelope reaches furniture. Unknown cells are not free.
    """
    if grid is None:return False
    info=grid.info;res=info.resolution
    if not math.isfinite(res) or res<=0:return False
    q=info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx=point[0]-info.origin.position.x;dy=point[1]-info.origin.position.y
    x=(math.cos(yaw)*dx+math.sin(yaw)*dy)/res
    y=(-math.sin(yaw)*dx+math.cos(yaw)*dy)/res
    r=radius/res
    if x-r<0 or y-r<0 or x+r>=info.width or y+r>=info.height:return False
    for iy in range(math.floor(y-r),math.floor(y+r)+1):
        for ix in range(math.floor(x-r),math.floor(x+r)+1):
            ex=max(ix-x,0.,x-(ix+1));ey=max(iy-y,0.,y-(iy+1))
            if ex*ex+ey*ey>r*r:continue
            value=grid.data[iy*info.width+ix]
            if value<0 or value>=50:return False
    return True


def passed_blocking_position(position, winner_start, blocker_start, clearance):
    """Outside a zone before entry is not evidence of a completed passage.

    Require progress past the original blocker along the approach direction.
    The caller also checks current zone occupancy and destination clearance.
    """
    if position is None or winner_start is None or blocker_start is None:return False
    values=(*position[:2],*winner_start[:2],*blocker_start[:2],clearance)
    if not all(math.isfinite(v) for v in values):return False
    dx=blocker_start[0]-winner_start[0];dy=blocker_start[1]-winner_start[1]
    length=math.hypot(dx,dy)
    if length<1e-6:return False
    progress=((position[0]-blocker_start[0])*dx+(position[1]-blocker_start[1])*dy)/length
    return progress>=clearance


def segment_crosses_rectangle(a,b,bounds):
    lo,hi=0.,1.
    for axis in (0,1):
        lower,upper=sorted(bounds[axis*2:axis*2+2])
        delta=b[axis]-a[axis]
        if abs(delta)<1e-9:
            if not lower<=a[axis]<=upper:return False
        else:
            t0,t1=sorted(((lower-a[axis])/delta,(upper-a[axis])/delta))
            lo=max(lo,t0);hi=min(hi,t1)
            if lo>hi:return False
    return True

def opposed_corridor_paths(poses,goals,paths,zones):
    """Reserve only a shared corridor on both actual Nav2 paths before entry."""
    robots=('robot_01','robot_02');shared=[]
    if any(r not in poses or not goals.get(r) or len(paths.get(r,[]))<2 for r in robots):return shared
    for name,b in zones.items():
        if not all(any(segment_crosses_rectangle(a,c,b) for a,c in zip(paths[r],paths[r][1:])) for r in robots):continue
        # Opposing paths can cross either dimension of a map's traffic region.
        # No assumed world orientation or fixed corridor name is needed.
        for axis in (0,1):
            lower,upper=b[axis*2:axis*2+2];sides=[]
            for r in robots:
                p,g=poses[r][axis],goals[r][axis]
                side=-1 if p<lower else 1 if p>upper else 0
                if not side or not (g>upper if side<0 else g<lower):break
                sides.append(side)
            if len(sides)==2 and sides[0]!=sides[1]:shared.append(name);break
    return shared

def reserved_winner_clear(poses,winner,loser,zones,names,separation,body_margin):
    if winner not in poses or loser not in poses:return False
    if distance(poses[winner],poses[loser])<separation:return False
    return not any(in_rectangle(poses[loser],zones[name],body_margin) for name in names)


def reservation_setup_pending(probe,recovery,statuses):
    if probe and probe.get('proactive'):return True
    return bool(recovery and recovery.get('proactive') and statuses.get(recovery['loser'],{}).get('state') not in ('YIELDING','YIELD_WAIT'))

def preentry_pause_acknowledged(statuses,request_ids):
    return bool(request_ids) and all(
        statuses.get(r,{}).get('request_id')==rid
        and (statuses[r].get('nav_paused') is True or statuses[r].get('state')=='RECOVERY_WAIT')
        and statuses[r].get('goal_execution_verified_idle') is True
        and statuses[r].get('state') in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','GOING_KITCHEN','RETURNING_HOME','RECOVERY_WAIT')
        for r,rid in request_ids.items())
