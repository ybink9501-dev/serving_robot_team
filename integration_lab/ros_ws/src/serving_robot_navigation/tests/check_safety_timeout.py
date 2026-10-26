#!/usr/bin/env python3

import math
import os
import statistics
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import (
    Point32, PolygonStamped, TransformStamped, Twist
)
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from serving_robot_interfaces.msg import PersonSafetyEvent
from tf2_ros import StaticTransformBroadcaster

assert os.environ.get("ROS_DOMAIN_ID") == "97", "시험 도메인은 97만 허용"

limits = yaml.safe_load(
    (Path.home() / "team_robot_lab/config/common/runtime_limits.yaml").read_text()
)

rclpy.init()
node = Node("safety_timeout_test")
records = {"smoothed": [], "semantic": [], "final": []}

try:
    for _ in range(12):
        rclpy.spin_once(node, timeout_sec=0.05)

    assert not node.get_subscriptions_info_by_topic(
        "/robot_01/cmd_vel"
    ), "최종 속도 구독자가 존재하여 시험 중단"

    assert not node.get_publishers_info_by_topic(
        "/robot_01/cmd_vel_nav"
    ), "다른 속도 명령 발행자가 있어 시험 중단"

    assert len(node.get_publishers_info_by_topic(
        "/robot_01/cmd_vel"
    )) == 1, "Collision Monitor 발행자 확인 필요"

    cmd_pub = node.create_publisher(
        Twist, "/robot_01/cmd_vel_nav", 10
    )
    event_pub = node.create_publisher(
        PersonSafetyEvent,
        "/robot_01/person_safety_event_lab", 10
    )
    scan_pub = node.create_publisher(
        LaserScan, "/robot_01/scan", 10
    )
    footprint_pub = node.create_publisher(
        PolygonStamped,
        "/robot_01/local_costmap/published_footprint", 10
    )

    for key, topic in [
        ("smoothed", "/robot_01/cmd_vel_smoothed"),
        ("semantic", "/robot_01/cmd_vel_semantic"),
        ("final", "/robot_01/cmd_vel"),
    ]:
        def callback(msg, name=key):
            records[name].append(
                (time.monotonic(), msg.linear.x)
            )

        node.create_subscription(Twist, topic, callback, 10)

    tf_pub = StaticTransformBroadcaster(node)

    def make_tf(parent, child):
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.rotation.w = 1.0
        return msg

    tf_pub.sendTransform([
        make_tf("odom", "base_footprint"),
        make_tf("base_footprint", "laser_link"),
    ])

    def publish_inputs(send_event=True, send_cmd=True):
        stamp = node.get_clock().now().to_msg()

        if send_cmd:
            cmd = Twist()
            cmd.linear.x = float(limits["max_linear_mps"]) * 0.8
            cmd_pub.publish(cmd)

        if send_event:
            event = PersonSafetyEvent()
            event.robot_id = "robot_01"
            event.state = "NORMAL"
            event.distance_m = float(
                limits["person_slow_distance_m"]
            ) + 1.0
            event_pub.publish(event)

        footprint = PolygonStamped()
        footprint.header.stamp = stamp
        footprint.header.frame_id = "base_footprint"
        footprint.polygon.points = [
            Point32(x=x, y=y, z=0.0)
            for x, y in [
                (0.26, 0.20),
                (0.26, -0.20),
                (-0.26, -0.20),
                (-0.26, 0.20),
            ]
        ]
        footprint_pub.publish(footprint)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser_link"
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 720
        scan.angle_max = math.pi - scan.angle_increment
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.scan_time = 0.05
        scan.ranges = [3.0] * 720
        scan_pub.publish(scan)

    def run_case(name, send_event=True, send_cmd=True, duration=2.0):
        started = time.monotonic()

        while time.monotonic() - started < duration:
            publish_inputs(send_event, send_cmd)

            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.008)

        cutoff = time.monotonic() - 0.45
        result = {}

        for key, values in records.items():
            recent = [
                value for stamp, value in values
                if stamp >= cutoff
            ]
            result[key] = (
                statistics.median(recent) if recent else None
            )

        print(
            f"{name}: "
            f"smoothed={result['smoothed']} "
            f"semantic={result['semantic']} "
            f"final={result['final']}",
            flush=True,
        )
        return result

    print("===== STEP 79 시험 =====", flush=True)

    run_case("WARMUP", duration=2.5)

    fresh = run_case("FRESH")
    person_stale = run_case(
        "PERSON_EVENT_STALE",
        send_event=False,
        duration=1.8,
    )
    recovery = run_case("RECOVERY")
    cmd_stale = run_case(
        "CMD_STALE",
        send_cmd=False,
        duration=1.8,
    )

    # 시험 종료 후 명시적 정지
    for _ in range(12):
        cmd_pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    def positive(v):
        return v is not None and v > 0.03

    def zero(v):
        return v is not None and abs(v) <= 0.015

    checks = {
        "FRESH": positive(fresh["final"]),
        "PERSON_EVENT_STALE": (
            positive(person_stale["smoothed"])
            and zero(person_stale["semantic"])
            and zero(person_stale["final"])
        ),
        "RECOVERY": positive(recovery["final"]),
        "CMD_STALE": (
            zero(cmd_stale["semantic"])
            and zero(cmd_stale["final"])
        ),
    }

    print("\n===== STEP 79 판정 =====")
    for name, passed in checks.items():
        print(name, "PASS" if passed else "FAIL")

    print(
        "STEP 79:",
        "PASS" if all(checks.values()) else "추가 진단 필요"
    )

finally:
    node.destroy_node()
    rclpy.shutdown()
