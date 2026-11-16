"""Admission only: never changes navigation, cancellation or automatic return policy."""
import math


def home_refusal(robot, requests, supervisors, pose, home, tolerance, queued):
    work = [t for t in requests.values() if t['robot'] == robot and
            t['state'] not in ('SUCCEEDED', 'FAILED', 'CANCELED')]
    if any(t['mode'] == 'GO_HOME' for t in work):
        return 'ALREADY_RETURNING'
    state = supervisors.get(robot, {}).get('state', 'UNKNOWN')
    if state == 'RETURNING_HOME':
        return 'ALREADY_RETURNING'
    if work or queued or state not in ('IDLE', 'COMPLETED'):
        return 'ROBOT_BUSY'
    if pose is None or not all(math.isfinite(float(v)) for v in (*pose[:2], *home[:2])):
        return 'NOT_LOCALIZED'
    if math.hypot(pose[0]-home[0], pose[1]-home[1]) <= tolerance:
        return 'ALREADY_HOME'
    return None
