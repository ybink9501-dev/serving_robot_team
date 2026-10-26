import math


def decide_state(distance, detected, fresh, limits):
    if not fresh:
        return "SENSOR_STALE"

    if not detected:
        return "NORMAL"

    if distance is None or not math.isfinite(distance) or distance < 0:
        return "SENSOR_STALE"

    if distance < limits["person_stop_distance_m"]:
        return "STOP"

    if distance <= limits["person_slow_distance_m"]:
        return "SLOW"

    return "NORMAL"


def filter_velocity(linear, angular, state, limits):
    if state not in ("NORMAL", "SLOW"):
        return 0.0, 0.0

    if not math.isfinite(linear) or not math.isfinite(angular):
        return 0.0, 0.0

    max_v = limits["max_linear_mps"]
    max_w = limits["max_angular_rps"]

    # 후진 금지
    linear = max(0.0, min(linear, max_v))

    # 좌우 제자리 회전 허용
    angular = max(-max_w, min(angular, max_w))

    if state == "SLOW":
        linear = min(linear, limits["slow_linear_mps"])

    return linear, angular
