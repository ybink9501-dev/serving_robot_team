#!/usr/bin/env python3
"""Mapping Semantic Waypoint를 Nav2 NavigateToPose 목표로 변환한다."""

import argparse
import fcntl
import os
from contextlib import contextmanager
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x: float
    y: float
    yaw: float


def resolve_destination(path, destination_id):
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"Waypoint 파일 읽기 실패: {exc}") from exc

    if not isinstance(data, dict) or not isinstance(data.get("waypoints"), list):
        raise ValueError("waypoints 목록 없음: Mapping 데이터 미준비 또는 형식 오류")

    matches = [
        row for row in data["waypoints"]
        if isinstance(row, dict) and row.get("entity_id") == destination_id
    ]

    if not matches:
        raise ValueError(f"등록되지 않은 destination_id: {destination_id}")

    if len(matches) != 1:
        raise ValueError(f"중복 destination_id: {destination_id}")

    row = matches[0]
    waypoint_id = row.get("id")
    pose = row.get("pose")

    if not isinstance(waypoint_id, str) or not waypoint_id.strip():
        raise ValueError("Waypoint ID 형식 오류")

    if not isinstance(pose, dict):
        raise ValueError(f"{waypoint_id}: pose 형식 오류")

    values = []
    for key in ("x", "y", "yaw"):
        value = pose.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{waypoint_id}: pose.{key} 숫자 아님")

        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{waypoint_id}: pose.{key} 유효하지 않음")

        values.append(value)

    return Waypoint(waypoint_id, *values)


def wait_future(node, future, seconds):
    import rclpy

    deadline = time.monotonic() + seconds

    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))

    return future.done()


@contextmanager
def single_goal_guard(robot):
    domain = os.environ.get("ROS_DOMAIN_ID", "0")

    if not domain.isdecimal():
        raise ValueError("ROS_DOMAIN_ID 형식 오류")

    lock_dir = Path.home() / ".cache" / "serving_robot_navigation"
    lock_dir.mkdir(parents=True, exist_ok=True, mode=0o700)

    lock_path = lock_dir / f"nav_goal_{domain}_{robot}.lock"

    with lock_path.open("a+") as lock_file:
        try:
            fcntl.flock(
                lock_file.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        except BlockingIOError as exc:
            raise RuntimeError(
                f"{robot}: 이미 목표 처리 중, 새 요청 거부"
            ) from exc

        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def send_goal(robot, waypoint, server_timeout, goal_timeout, use_sim_time):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    node = Node("destination_goal_client", namespace=f"/{robot}")
    client = None
    goal_handle = None

    try:
        node.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)
        ])

        client = ActionClient(
            node, NavigateToPose, f"/{robot}/navigate_to_pose"
        )

        if not client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError("Nav2 NavigateToPose 서버 없음")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()

        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

        response = client.send_goal_async(goal)

        if not wait_future(node, response, server_timeout):
            raise RuntimeError("목표 요청 응답 시간 초과: 수락 여부 미확인")

        goal_handle = response.result()

        if not goal_handle.accepted:
            raise RuntimeError("Nav2 목표 거부")

        print(f"GOAL_ACCEPTED: {waypoint.waypoint_id}", flush=True)

        result = goal_handle.get_result_async()

        if not wait_future(node, result, goal_timeout):
            cancel = goal_handle.cancel_goal_async()

            if wait_future(node, cancel, 5.0) and cancel.result().goals_canceling:
                print("GOAL_CANCEL_REQUEST_ACCEPTED", flush=True)

                if wait_future(node, result, 5.0):
                    status = result.result().status

                    if status == GoalStatus.STATUS_CANCELED:
                        print("GOAL_CANCELED_CONFIRMED", flush=True)
                    else:
                        print(
                            f"WARNING: 취소 후 최종 상태={status}",
                            file=sys.stderr,
                        )
                else:
                    print(
                        "WARNING: 취소 최종 결과 미확인",
                        file=sys.stderr,
                    )
            else:
                print(
                    "WARNING: 취소 접수 여부 미확인",
                    file=sys.stderr,
                )

            raise RuntimeError("주행 시간 초과: Nav2 종료 상태 확인 필요")

        status = result.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("GOAL_SUCCEEDED: Nav2 결과, 실제 도착 검증은 별도")
            return 0

        if status == GoalStatus.STATUS_CANCELED:
            print("GOAL_CANCELED")
            return 2

        print(f"GOAL_NOT_SUCCEEDED: status={status}")
        return 3

    except KeyboardInterrupt:
        if goal_handle is not None and rclpy.ok():
            cancel = goal_handle.cancel_goal_async()
            if wait_future(node, cancel, 5.0) and cancel.result().goals_canceling:
                print("GOAL_CANCEL_REQUEST_ACCEPTED")
            else:
                print("WARNING: 취소 접수 여부 미확인", file=sys.stderr)
        return 130

    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="destination_id → Semantic Waypoint → Nav2"
    )
    parser.add_argument("destination_id")
    parser.add_argument("--waypoints", type=Path, required=True)
    parser.add_argument(
        "--robot", choices=["robot_01", "robot_02"], default="robot_01"
    )
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--server-timeout", type=float, default=5.0)
    parser.add_argument("--goal-timeout", type=float, default=120.0)

    args = parser.parse_args()

    if (not args.destination_id.strip()
            or not math.isfinite(args.server_timeout)
            or not math.isfinite(args.goal_timeout)
            or args.server_timeout <= 0
            or args.goal_timeout <= 0):
        parser.error("목적지 ID 또는 timeout 값 오류")

    try:
        waypoint = resolve_destination(
            args.waypoints, args.destination_id
        )

        print(
            f"RESOLVED: robot={args.robot} "
            f"destination={args.destination_id} "
            f"waypoint={waypoint.waypoint_id} "
            f"frame=map x={waypoint.x} "
            f"y={waypoint.y} yaw={waypoint.yaw}"
        )

        if not args.send:
            print("DRY_RUN_PASS: 실제 Nav2 명령 전송 안 함")
            return 0

        with single_goal_guard(args.robot):
            return send_goal(
                args.robot,
                waypoint,
                args.server_timeout,
                args.goal_timeout,
                args.use_sim_time,
            )

    except (ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
