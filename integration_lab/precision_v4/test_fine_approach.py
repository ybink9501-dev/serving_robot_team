#!/usr/bin/env python3
"""Closed-loop offline simulation of fine_approach (NOT Gazebo).
Plant: unicycle + 0.15 s command latency + smoother accel limits + odom noise."""
import collections, math, random, sys
from fine_approach import FineApproach, fine_params, map_pose_to_odom, wrap

DT = 0.05
def simulate(start_err, target=(0.0, 0.0, 0.0), latency=0.15, noise=0.0002, seed=1, blocked=False):
    rnd = random.Random(seed)
    p = fine_params(None)
    x, y, yaw = target[0] + start_err[0], target[1] + start_err[1], target[2] + start_err[2]
    fa = FineApproach(p, target)
    q = collections.deque([(0.0, 0.0)] * int(round(latency / DT)))
    v = w = 0.0; vmin_seen = 0.0; vmax_seen = 0.0; t = 0.0
    while t < p['timeout_s'] + 1:
        meas = (x + rnd.gauss(0, noise), y + rnd.gauss(0, noise), yaw + rnd.gauss(0, noise / 2))
        cv, cw, st, info = fa.step(meas, DT)
        if st != 'RUN':
            break
        vmin_seen = min(vmin_seen, cv); vmax_seen = max(vmax_seen, cv)
        q.append((cv, cw)); tv, tw = q.popleft()
        dv = max(-2.5 * DT, min(2.5 * DT, tv - v)); v += dv
        dw = max(-3.2 * DT, min(3.2 * DT, tw - w)); w += dw
        if not blocked:
            x += v * math.cos(yaw) * DT; y += v * math.sin(yaw) * DT; yaw = wrap(yaw + w * DT)
        t += DT
    for _ in range(40):                       # coast to rest with zero command
        dv = max(-2.5 * DT, min(2.5 * DT, -v)); v += dv
        dw = max(-3.2 * DT, min(3.2 * DT, -w)); w += dw
        x += v * math.cos(yaw) * DT; y += v * math.sin(yaw) * DT; yaw = wrap(yaw + w * DT)
    return st, math.hypot(x - target[0], y - target[1]), abs(wrap(yaw - target[2])), t, vmin_seen, vmax_seen

CASES = {
 'ahead 25mm':          (-0.025, 0.0, 0.10),
 'behind 25mm':         (0.025, 0.0, 0.10),
 'lateral 30mm':        (0.0, 0.030, -0.20),
 'diag 100mm yaw 0.3':  (-0.07, 0.07, 0.30),
 'far 200mm':           (-0.20, 0.0, 0.25),
 '8mm only':            (0.004, 0.007, 0.0),
 'yaw only 0.25':       (0.0, 0.0, 0.25),
 'target-yaw pi':       (-0.03, 0.02, 3.0),
}
ok_all = True
print('%-22s %-7s %9s %9s %6s %7s' % ('case', 'status', 'xy[mm]', 'yaw[rad]', 't[s]', 'min v'))
for name, e in CASES.items():
    worst = 0.0; worst_yaw = 0.0; stt = 'DONE'; tt = 0; mv = 0
    for seed in range(1, 21):
        st, xy, yw, t, vmin, vmax = simulate(e, seed=seed)
        worst = max(worst, xy); worst_yaw = max(worst_yaw, yw); tt = max(tt, t); mv = min(mv, vmin)
        if st != 'DONE': stt = st
    ok = stt == 'DONE' and worst <= 0.001 and worst_yaw <= 0.25 and mv >= 0.0
    ok_all &= ok
    print('%-22s %-7s %9.2f %9.4f %6.1f %7.3f  %s' % (name, stt, worst * 1000, worst_yaw, tt, mv, 'PASS' if ok else 'FAIL'))
# blocked robot: must time out, never DONE, never reverse
st, *_ = simulate((-0.02, 0, 0), blocked=True)
print('blocked -> %s' % st); ok_all &= (st == 'TIMEOUT')
# frame transform sanity
tp = map_pose_to_odom((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0))
ok_all &= abs(tp[0] - 1.0) < 1e-9 and abs(tp[1] - 3.0) < 1e-9 and abs(tp[2] - math.pi / 2) < 1e-9
for bad in ({'v_max': 0.5}, {'bogus': 1}, {'xy_tol_m': -1}, {'align_rad': 2.0}):
    try: fine_params(bad); ok_all = False; print('validation FAIL', bad)
    except ValueError: pass
print('ALL PASS' if ok_all else 'FAILED'); sys.exit(0 if ok_all else 1)
