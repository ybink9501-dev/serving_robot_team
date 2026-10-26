"""ROS-free conservative anti-spin policy; values loaded from the installed V6 config."""
import math
from collections import deque


class SpinWindow:
    def __init__(self, seconds, min_progress_m, angular_limit_rad):
        for val in (seconds, min_progress_m, angular_limit_rad):
            if not math.isfinite(val) or val <= 0:
                raise ValueError('Window thresholds must be positive finite numbers')
        self.seconds = seconds
        self.min_progress_m = min_progress_m
        self.angular_limit_rad = angular_limit_rad
        self.samples = deque()
        self.previous_yaw = None
        self.turn_abs_sum = 0.0

    def clear(self):
        self.samples.clear()
        self.previous_yaw = None
        self.turn_abs_sum = 0.0

    def observe(self, when, x, y, yaw):
        if not all(math.isfinite(z) for z in (when,x,y,yaw)):
            raise ValueError('Non-finite odometry')
        if self.samples and when <= self.samples[-1][0]:
            self.clear()  # Clock reset / out-of-order samples; never calculate across it
        delta = 0.0 if self.previous_yaw is None else abs(math.atan2(
            math.sin(yaw-self.previous_yaw),math.cos(yaw-self.previous_yaw)))
        self.previous_yaw = yaw
        self.turn_abs_sum += delta
        self.samples.append((when,x,y,self.turn_abs_sum))
        while len(self.samples) > 1 and when-self.samples[0][0] > self.seconds:
            self.samples.popleft()
        first = self.samples[0]
        dt=when-first[0]
        rotation=self.turn_abs_sum-first[3]
        distance=math.hypot(x-first[1],y-first[2])
        triggered=(dt >= self.seconds * .90 and
                   rotation > self.angular_limit_rad and
                   distance < self.min_progress_m)
        return {'triggered':triggered,'window_s':round(dt,3),
                'rotation_rad':rotation, 'net_displacement_m':distance}
