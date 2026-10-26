#!/usr/bin/env python3
"""Final-centimetre approach controller (pure Python, no ROS import).

Nav2 completion requires the configured 2.5 mm goal tolerance. This controller then closes the
remaining XY error and the final yaw error WITHOUT sending any new Nav2 goal.
Differential drive, forward-only: v >= 0 always (reverse is never commanded).
It runs in the odom frame with the target frozen at start, so AMCL map->odom
jumps cannot move the target while the robot is creeping.
"""
import math

DEFAULTS = dict(
    engage_max_m=0.25,    # refuse fine approach if Nav2 ended farther than this
    xy_tol_m=0.0006,      # 0.6 mm design target; arrival final_error_m is 1 mm
    yaw_tol_rad=0.03,     # fine yaw goal; arrival.yaw_tolerance_rad (0.25) is the judge
    v_max=0.05, v_min=0.006,
    w_max=0.45, w_min=0.04,
    k_v=1.0, k_w=1.5, k_h=1.2,
    align_rad=0.35,       # rotate in place when heading error exceeds this
    lead_s=0.20,          # command latency compensation (stop early by v*lead)
    timeout_s=30.0,       # per attempt
    max_attempts=3,
    settle_s=1.0,         # wait at rest before an attempt / before re-judging
    cmd_topic='cmd_vel_nav',  # smoother input: keeps Semantic Safety + Collision Monitor
)
HARD_V_MAX = 0.10
HARD_W_MAX = 0.50


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fine_params(section):
    p = dict(DEFAULTS)
    if section:
        unknown = set(section) - set(DEFAULTS)
        if unknown:
            raise ValueError('unknown fine_approach keys: %s' % sorted(unknown))
        p.update(section)
    for k, v in p.items():
        if k == 'cmd_topic':
            if not isinstance(v, str) or not v:
                raise ValueError('cmd_topic invalid')
        elif not isinstance(v, (int, float)) or isinstance(v, bool) \
                or not math.isfinite(v) or v <= 0:
            raise ValueError('fine_approach.%s must be a positive finite number' % k)
    p['max_attempts'] = int(p['max_attempts'])
    if p['v_min'] > p['v_max'] or p['v_max'] > HARD_V_MAX:
        raise ValueError('need v_min <= v_max <= %.2f' % HARD_V_MAX)
    if p['w_min'] > p['w_max'] or p['w_max'] > HARD_W_MAX:
        raise ValueError('need w_min <= w_max <= %.2f' % HARD_W_MAX)
    if not 0 < p['align_rad'] < math.pi / 2:
        raise ValueError('align_rad must be < pi/2 so forward motion always reduces error')
    return p


def map_pose_to_odom(t_odom_map, pose):
    """t_odom_map = pose of the map frame expressed in odom (tf lookup('odom','map'))."""
    tx, ty, th = t_odom_map
    x, y, yaw = pose
    c, s = math.cos(th), math.sin(th)
    return (c * x - s * y + tx, s * x + c * y + ty, wrap(yaw + th))


class FineApproach:
    def __init__(self, params, target):
        self.p = params
        self.target = target
        self.phase = 'XY'
        self.rotating = False
        self.elapsed = 0.0
        self.v_last = 0.0

    def step(self, pose, dt):
        p = self.p
        self.elapsed += dt
        x, y, yaw = pose
        tx, ty, tyaw = self.target
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        yerr = wrap(tyaw - yaw)
        info = dict(xy_error_m=round(dist, 6), yaw_error_rad=round(abs(yerr), 6),
                    elapsed_s=round(self.elapsed, 2))
        if self.elapsed > p['timeout_s']:
            return 0.0, 0.0, 'TIMEOUT', info
        if dist > 1.5 * p['engage_max_m']:
            return 0.0, 0.0, 'DIVERGED', info
        if self.phase == 'YAW' and dist > 3 * p['xy_tol_m']:
            self.phase = 'XY'
        if self.phase == 'XY' and dist <= p['xy_tol_m']:
            self.phase = 'YAW'
            self.rotating = False
        if self.phase == 'YAW':
            if abs(yerr) <= p['yaw_tol_rad']:
                self.v_last = 0.0
                return 0.0, 0.0, 'DONE', info
            w = math.copysign(min(p['w_max'], max(p['w_min'], p['k_w'] * abs(yerr))), yerr)
            self.v_last = 0.0
            return 0.0, w, 'RUN', info
        herr = wrap(math.atan2(dy, dx) - yaw)
        if self.rotating:
            if abs(herr) < p['align_rad'] / 2:
                self.rotating = False
        elif abs(herr) > p['align_rad']:
            self.rotating = True
        if self.rotating:
            w = math.copysign(min(p['w_max'], max(p['w_min'], p['k_w'] * abs(herr))), herr)
            self.v_last = 0.0
            return 0.0, w, 'RUN', info
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy      # forward distance, > 0 here
        if self.v_last > 0 and ex <= self.v_last * p['lead_s']:
            self.v_last = 0.0                              # stop early, let it coast
            return 0.0, 0.0, 'RUN', info
        v = min(p['v_max'], max(p['v_min'], p['k_v'] * ex))
        w = max(-p['w_max'], min(p['w_max'], p['k_h'] * herr))
        v = max(0.0, v)
        self.v_last = v
        return v, w, 'RUN', info
