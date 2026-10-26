#!/usr/bin/env python3
"""Near-goal stalled handoff: motion must never overlap an active Nav2 Action.
Fake ROS + kinematic plant only; not evidence of Gazebo safety or mm accuracy.
"""
import math
import test_mock_ros as T
from pathlib import Path
# Reuse the REAL class mock/plant setup, not the test runner's sys.exit().
base=Path(__file__).with_name('test_v4_server.py')
src=base.read_text().split('# S1 normal case:')[0]
ns={'__file__':str(base),'__name__':'v4_test_helpers'}
exec(compile(src,str(base),'exec'),ns)
CMDS,W,run=ns['CMDS'],ns['W'],ns['run']
def start_active(dest, offset, **params):
    CMDS.clear()
    s=ns['Sim'](str(Path(__file__).with_name('destination_server_lab_v4.py')), **params)
    w=T.WAYPOINTS[dest]
    W.set((w['x']+offset[0],w['y']+offset[1],w['yaw']+offset[2]),(0.03,-0.02,0.05))
    s.go(dest)
    return s,s.accept(0)

failed=[]
def check(label, ok, extra=''):
    print(('PASS ' if ok else 'FAIL ') + label + (' '+extra if extra else ''))
    if not ok: failed.append(label)

# Default feature is OFF; explicit opt-in for this focused test.
T.ARRIVAL['near_goal_handoff'] = {
    'enabled': True,
    'radius_m': 0.06,
    'stalled_for_s': 6.0,
}

# Case A: ~51.6mm away and stopped, matching user trial. Nav2 remains active.
s,h = start_active('table_01', (0.042, 0.030, 0.0))
run(s, 9)
check('H1 near-goal idle requests Nav2 cancel', h.cancelled and 'NEAR_GOAL_HANDOFF_CANCEL_REQUESTED' in s.events)
check('H2 no fine motion before terminal nav status', not any(abs(v)>1e-9 or abs(w)>1e-9 for v,w in CMDS))
# Action server now terminally reports canceled, not merely a cancel request.
s.cancel_result(h)
check('H3 handoff enabled only AFTER Nav2 terminal CANCELED', s.node.handoff_ready and not s.node.unresolved_goal)
run(s, 120)
w = T.WAYPOINTS['table_01']
err = math.hypot(W.x-w['x'],W.y-w['y'])
check('H4 correction reaches physical arrival and no additional nav goal', s.states[-1]=='SERVING' and s.goals==1,
      f'state={s.states[-1]} xy_error={err*1000:.3f}mm')
check('H5 never reverse and ends with zero', CMDS[-1]==(0.0,0.0) and min(v for v,w in CMDS)>=0)
s.close()

# Case B: outside handoff radius -> no cancellation or driving.
s,h = start_active('table_02', (0.12, 0.0, 0.0)); run(s, 10)
check('H6 outside 6cm => no handoff',not h.cancelled and not s.node.handoff_ready)
s.close()

# Case C: User CANCEL during pending handoff supersedes internal transfer.
s,h = start_active('table_03', (0.04,0.0,0.0))
run(s, 9)
s.cancel(); s.cancel_result(h); run(s,2)
check('H7 external cancel supersedes handoff',s.states[-1]=='PAUSED' and not s.node.handoff_ready)
s.close()

# Case D: cancel returns ABORTED, not CANCELED => never drive.
s,h = start_active('table_04', (0.04,0.0,0.0));run(s,9)
h.result_future.set(T.Result(6));run(s,2)
check('H8 aborted result cannot authorize fine motion',s.states[-1]=='ERROR' and not s.node.handoff_ready)
s.close()

# ---- LINGER trigger: robot keeps MOVING (orbit) ~15 cm from goal, no progress ----
def orbit_hook(dest, r, h, rate=0.15, cx_off=(0.0,0.0)):
    w=T.WAYPOINTS[dest]
    state={'a':0.0}
    def hook(i):
        if h.cancelled: return           # stop forcing the pose once Nav2 is canceled
        state['a']+=rate*0.05
        a=state['a']
        W.x=w['x']+cx_off[0]+r*math.cos(a); W.y=w['y']+cx_off[1]+r*math.sin(a)
        W.yaw=a+math.pi/2; W.v=0.0225; W.w=rate   # clearly NOT stationary
    return hook
T.ARRIVAL['near_goal_handoff'] = {'enabled': True,'radius_m': 0.06,'stalled_for_s': 6.0,
    'linger_radius_m': 0.20,'linger_for_s': 8.0,'linger_min_progress_m': 0.01}

s,h = start_active('table_01', (0.15, 0.0, 0.0))
run(s, 6, orbit_hook('table_01',0.15,h))
check('L0 orbiting 6 s (< linger_for) => no cancel yet', not h.cancelled)
run(s, 6, orbit_hook('table_01',0.15,h))
check('L1 orbit ~15cm, no progress => handoff cancel requested', h.cancelled and 'NEAR_GOAL_HANDOFF_CANCEL_REQUESTED' in s.events)
check('L2 no fine motion before terminal CANCELED', not any(abs(v)>1e-9 or abs(w_)>1e-9 for v,w_ in CMDS))
s.cancel_result(h)
check('L3 handoff_ready only after terminal CANCELED', s.node.handoff_ready)
run(s, 150)
w=T.WAYPOINTS['table_01']; err=math.hypot(W.x-w['x'],W.y-w['y'])
check('L4 fine approach from orbit reaches SERVING, one nav goal, no reverse',
      s.states[-1]=='SERVING' and s.goals==1 and min(v for v,_ in CMDS)>=0 and CMDS[-1]==(0.0,0.0),
      f'state={s.states[-1]} xy_error={err*1000:.3f}mm')
s.close()

# orbit outside linger radius => no cancel
s,h = start_active('table_02', (0.30, 0.0, 0.0)); run(s, 20, orbit_hook('table_02',0.30,h))
check('L5 orbit at 30cm (outside 20cm) => no handoff', not h.cancelled)
s.close()

# steady approach (>=1 cm / window of progress) must NOT be cut off
def approach_hook_factory(dest,h):
    w=T.WAYPOINTS[dest]; st={'d':0.19}
    def hook(i):
        if h.cancelled: return
        st['d']=max(0.03,st['d']-0.03*0.05)
        W.x=w['x']+st['d']; W.y=w['y']; W.yaw=w['yaw']; W.v=0.03; W.w=0.0
    return hook
s,h = start_active('table_03', (0.19, 0.0, 0.0)); run(s, 5, approach_hook_factory('table_03',h))
check('L6 steady progress inside 20cm => no handoff', not h.cancelled)
s.close()

# feature disabled (linger_radius_m=0) => old behaviour, orbit never cancelled
T.ARRIVAL['near_goal_handoff'] = {'enabled': True,'radius_m': 0.06,'stalled_for_s': 6.0}
s,h = start_active('table_04', (0.15, 0.0, 0.0)); run(s, 20, orbit_hook('table_04',0.15,h))
check('L7 linger disabled by default => no cancel (backward compatible)', not h.cancelled)
s.close()

# external CANCEL during linger handoff still wins
T.ARRIVAL['near_goal_handoff'] = {'enabled': True,'radius_m': 0.06,'stalled_for_s': 6.0,
    'linger_radius_m': 0.20,'linger_for_s': 8.0,'linger_min_progress_m': 0.01}
s,h = start_active('table_01', (0.15, 0.0, 0.0)); run(s, 12, orbit_hook('table_01',0.15,h))
s.cancel(); s.cancel_result(h); run(s,2)
check('L8 external cancel supersedes linger handoff', s.states[-1]=='PAUSED' and not s.node.handoff_ready)
s.close()
print('HANDOFF PASS' if not failed else 'HANDOFF FAILED '+str(failed))
raise SystemExit(1 if failed else 0)
