"""자율주행 경로 차단 시 대기·알림·우회 요청 정책."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Decision:
    elapsed_s: float
    motion: str
    notify: bool
    request_detour: bool
    keep_waiting: bool


class BlockagePolicy:
    def __init__(self, judge_s=3.0, alert_s=15.0, detour_s=30.0):
        if not 0 < judge_s < alert_s < detour_s:
            raise ValueError("시간 기준 순서 오류")
        self.judge_s = judge_s
        self.alert_s = alert_s
        self.detour_s = detour_s
        self.reset()

    def reset(self):
        self.blocked_since = None
        self.motion_since = None
        self.previous_motion = None
        self.last_time = None
        self.alerted = False
        self.detour_requested = False

    def update(
        self, now_s, blocked,
        obstacle_type="unknown",
        person_motion="unknown"
    ):
        if not isfinite(now_s):
            raise ValueError("시간 오류")

        if not blocked:
            self.reset()
            return Decision(0.0, "unknown", False, False, False)

        if obstacle_type not in ("person", "object", "unknown"):
            raise ValueError("장애물 종류 오류")

        if person_motion not in ("moving", "stationary", "unknown"):
            raise ValueError("사람 움직임 값 오류")

        if self.last_time is not None and now_s < self.last_time:
            raise ValueError("시간 역행: 상태 재설정 필요")

        self.last_time = now_s

        if self.blocked_since is None:
            self.blocked_since = now_s

        elapsed = now_s - self.blocked_since

        observed_motion = (
            person_motion if obstacle_type == "person" else "unknown"
        )

        if observed_motion != self.previous_motion:
            self.previous_motion = observed_motion
            self.motion_since = now_s

        motion = "unknown"

        if (
            obstacle_type == "person"
            and observed_motion != "unknown"
            and now_s - self.motion_since >= self.judge_s
        ):
            motion = observed_motion

        notify = elapsed >= self.alert_s and not self.alerted
        if notify:
            self.alerted = True

        detour_allowed = (
            obstacle_type == "object"
            or (obstacle_type == "person" and motion == "stationary")
        )

        request_detour = (
            elapsed >= self.detour_s
            and detour_allowed
            and not self.detour_requested
        )

        if request_detour:
            self.detour_requested = True

        return Decision(
            elapsed_s=elapsed,
            motion=motion,
            notify=notify,
            request_detour=request_detour,
            keep_waiting=True,
        )
