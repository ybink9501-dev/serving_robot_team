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
print('HANDOFF PASS' if not failed else 'HANDOFF FAILED '+str(failed))
raise SystemExit(1 if failed else 0)
