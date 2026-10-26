#!/usr/bin/env python3
"""Closed-loop test of the REAL destination_server_lab_v4 class against fake ROS + a
simple kinematic plant (latency, accel limits, AMCL map->odom offset). Not Gazebo."""
import collections, math, sys, types
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_mock_ros as T
from test_mock_ros import Sim, Msg, FakeNode, WAYPOINTS, at
from fine_approach import wrap

T.install_fakes()
sys.modules['geometry_msgs.msg'].Twist = lambda: Msg()
T.ARRIVAL['arrival'].update(detection_radius_m=0.005, final_error_m=0.001, yaw_tolerance_rad=0.25,
                            stopped_linear_mps=0.01, stopped_angular_rps=0.02, final_verify_timeout_s=200.0)
T.ARRIVAL['fine_approach'] = {}
CMDS = []
class CmdPub:
    def publish(self, m):
        g = lambda o, k: o.__dict__.get(k, 0.0) if isinstance(o, Msg) else 0.0
        CMDS.append((g(m.linear, 'x'), g(m.angular, 'z')))
_orig = FakeNode.create_publisher
FakeNode.create_publisher = lambda self, t, topic, d: CmdPub() if topic.endswith('cmd_vel_nav') else _orig(self, t, topic, d)

class World:
    def __init__(self): self.set((0, 0, 0), (0, 0, 0), True)
    def set(self, map_pose, amcl, free=True):
        self.x, self.y, self.yaw = map_pose; self.amcl = amcl; self.free = free
        self.v = self.w = 0.0; self.q = collections.deque([(0.0, 0.0)] * 3)
    def odom_pose(self):          # odom pose = inverse(map->odom) o map pose
        ox, oy, oth = self.amcl; c, s = math.cos(-oth), math.sin(-oth)
        dx, dy = self.x - ox, self.y - oy
        return (c * dx - s * dy, s * dx + c * dy, wrap(self.yaw - oth))
    def map_in_odom(self):
        ox, oy, oth = self.amcl; c, s = math.cos(-oth), math.sin(-oth)
        return (c * -ox - s * -oy, s * -ox + c * -oy, wrap(-oth))
    def advance(self, dt=0.05):
        cv, cw = CMDS[-1] if CMDS else (0.0, 0.0)
        self.q.append((cv, cw)); tv, tw = self.q.popleft()
        self.v += max(-2.5 * dt, min(2.5 * dt, tv - self.v)); self.w += max(-3.2 * dt, min(3.2 * dt, tw - self.w))
        if self.free:
            self.x += self.v * math.cos(self.yaw) * dt; self.y += self.v * math.sin(self.yaw) * dt
            self.yaw = wrap(self.yaw + self.w * dt)
W = World()
def lookup(self, target, source, t):
    r = Msg()
    if (target, source) == ('map', 'base_footprint'): p = (W.x, W.y, W.yaw)
    elif (target, source) == ('odom', 'base_footprint'): p = W.odom_pose()
    elif (target, source) == ('odom', 'map'): p = W.map_in_odom()
    else: raise RuntimeError('unexpected lookup %s %s' % (target, source))
    r.transform.translation.x, r.transform.translation.y = p[0], p[1]
    r.transform.rotation.x = r.transform.rotation.y = 0.0
    r.transform.rotation.z = math.sin(p[2] / 2); r.transform.rotation.w = math.cos(p[2] / 2)
    return r
T.Buffer.lookup_transform = lookup

def run(s, seconds, hook=None):
    for i in range(int(seconds / 0.05)):
        W.advance(); s.t += 0.05; s.odom(W.v, W.w)
        if hook: hook(i)
        s.node.fine_tick()
        if i % 4 == 3: s.node.tick()
        if s.states and s.states[-1] in ('SERVING', 'ERROR', 'PAUSED'): break

def start(dest, offset, amcl=(0.03, -0.02, 0.05), **params):
    CMDS.clear(); s = Sim(str(Path(__file__).resolve().parent / 'destination_server_lab_v4.py'), **params)
    w = WAYPOINTS[dest]; W.set((w['x'] + offset[0], w['y'] + offset[1], w['yaw'] + offset[2]), amcl)
    s.go(dest); h = s.accept(0); s.succeed(h); return s

results = []
def check(name, ok, extra=''):
    results.append(ok); print(('PASS ' if ok else 'FAIL ') + name + ('  ' + extra if extra else ''))

# S1 normal case: Nav2 stops 2.5 cm / 1.5 cm off, 0.2 rad yaw off, AMCL offset present
s = start('table_03', (0.025, 0.015, 0.2)); run(s, 120)
w = WAYPOINTS['table_03']; err = math.hypot(W.x - w['x'], W.y - w['y'])
check('S1 reaches SERVING with fine approach', s.states[-1] == 'SERVING', f'final xy={err*1000:.2f}mm yaw={abs(wrap(W.yaw-w["yaw"])):.4f}rad t={s.t-1000:.1f}s')
check('S1 exactly one Nav2 goal (no reissue)', s.goals == 1)
check('S1 never commanded reverse', min(c[0] for c in CMDS) >= 0.0 and max(c[0] for c in CMDS) <= 0.05 + 1e-9 and max(abs(c[1]) for c in CMDS) <= 0.45 + 1e-9)
check('S1 final zero command published', CMDS[-1] == (0.0, 0.0) and 'FINE_APPROACH_STARTED' in s.events)
s.close()
# S1c: target is BEHIND the robot (Nav2 overshoot) -> forward-only U-turn must not trip the spin watchdog
w3 = WAYPOINTS['table_03']; CMDS.clear()
s = start('table_03', (0.025 * math.cos(w3['yaw']), 0.025 * math.sin(w3['yaw']), 0.0)); run(s, 120)
check('S1c overshoot (target behind) -> SERVING, no reverse, not killed by spin watchdog',
      s.states[-1] == 'SERVING' and min(c[0] for c in CMDS) >= 0.0, f'state={s.states[-1]} t={s.t-1000:.1f}s'); s.close()
# S1b: Nav2 stopped exactly at target already -> no motion needed
s = start('table_01', (0.0, 0.0, 0.0)); run(s, 20)
check('S1b already-in-tolerance arrives without commands', s.states[-1] == 'SERVING' and not any(c != (0.0, 0.0) for c in CMDS)); s.close()
# S2 AMCL map->odom jump during approach: must re-freeze and still converge
def jump(i):
    if i == 80: W.amcl = (W.amcl[0] + 0.02, W.amcl[1] - 0.015, W.amcl[2])
s = start('table_02', (-0.03, 0.01, 0.1)); run(s, 160, jump)
check('S2 recovers from AMCL jump (re-attempt)', s.states[-1] == 'SERVING' and s.goals == 1,
      f'attempts={s.node.fine_attempts} err={math.hypot(W.x-WAYPOINTS["table_02"]["x"], W.y-WAYPOINTS["table_02"]["y"])*1000:.2f}mm'); s.close()
# S3 blocked robot -> bounded attempts then ERROR, still one goal
s = start('table_04', (0.02, 0.0, 0.0)); W.free = False; run(s, 260)
check('S3 blocked -> ERROR after bounded attempts', s.states[-1] == 'ERROR' and s.goals == 1 and s.node.fine_attempts == 3,
      f'reason attempts={s.node.fine_attempts} events={sorted(set(e for e in s.events if "FINE" in e or "FINAL" in e))}')
check('S3 stops commanding after ERROR', CMDS[-1] == (0.0, 0.0)); s.close()
# S4 CANCEL while fine approach is running
s = start('table_01', (0.03, 0.0, 0.1)); run(s, 3)
before = len(CMDS); s.cancel(); run(s, 2)
check('S4 CANCEL stops fine approach (PAUSED, zero cmd)', s.states[-1] == 'PAUSED' and CMDS[-1] == (0.0, 0.0) and before > 0); s.close()
# S5 too far -> refuse (no blind driving)
s = start('table_01', (0.5, 0.0, 0.0)); run(s, 10)
check('S5 Nav2 ended 50cm away -> ERROR, no motion', s.states[-1] == 'ERROR' and not any(c != (0.0, 0.0) for c in CMDS)); s.close()
# S6 disabled -> V3 behaviour
s = start('table_01', (0.02, 0.0, 0.0), fine_approach_enabled=False); T.ARRIVAL['arrival']['final_verify_timeout_s'] = 3.0
run(s, 5)
check('S6 fine_approach_enabled=false keeps V3 behaviour', not CMDS or all(c == (0.0, 0.0) for c in CMDS)); s.close()
print('ALL PASS' if all(results) else 'FAILED'); sys.exit(0 if all(results) else 1)
