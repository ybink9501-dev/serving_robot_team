import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serving_robot_interfaces.msg import PersonSafetyEvent

from serving_robot_perception.semantic_safety import filter_velocity


class SemanticSafetyNode(Node):
    def __init__(self):
        super().__init__("semantic_safety")

        self.robot_id = self.declare_parameter(
            "robot_id", "robot_01"
        ).value
        event_topic = self.declare_parameter(
            "person_event_topic", ""
        ).value
        limits_file = self.declare_parameter(
            "limits_file", ""
        ).value
        self.sensor_timeout = float(
            self.declare_parameter("sensor_timeout_s", 0.0).value
        )

        if not event_topic or not limits_file or self.sensor_timeout <= 0:
            raise ValueError(
                "person_event_topic, limits_file, "
                "sensor_timeout_s 설정이 필요합니다"
            )

        self.limits = yaml.safe_load(
            Path(limits_file).read_text()
        )
        self.cmd_timeout = float(
            self.limits["cmd_vel_timeout_s"]
        )
        if self.cmd_timeout <= 0:
            raise ValueError("cmd_vel_timeout_s 값이 잘못됐습니다")

        self.last_cmd = None
        self.last_cmd_time = None
        self.last_event_time = None
        self.safety_state = "SENSOR_STALE"

        self.pub = self.create_publisher(
            Twist, "/robot_01/cmd_vel_semantic", 10
        )
        self.create_subscription(
            Twist,
            "/robot_01/cmd_vel_smoothed",
            self.on_cmd,
            10,
        )
        self.create_subscription(
            PersonSafetyEvent,
            event_topic,
            self.on_event,
            10,
        )
        self.create_timer(0.05, self.tick)

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_event(self, msg):
        if msg.robot_id != self.robot_id:
            return

        state = msg.state
        allowed = {"NORMAL", "SLOW", "STOP", "SENSOR_STALE"}

        if state not in allowed:
            state = "SENSOR_STALE"

        if state in {"NORMAL", "SLOW"}:
            distance = float(msg.distance_m)
            if math.isnan(distance) or distance < 0:
                state = "SENSOR_STALE"
            elif math.isfinite(distance):
                if distance < self.limits["person_stop_distance_m"]:
                    state = "STOP"
                elif distance <= self.limits["person_slow_distance_m"]:
                    if state == "NORMAL":
                        state = "SLOW"

        self.safety_state = state
        self.last_event_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        output = Twist()

        cmd_fresh = (
            self.last_cmd_time is not None
            and now - self.last_cmd_time <= self.cmd_timeout
        )
        event_fresh = (
            self.last_event_time is not None
            and now - self.last_event_time <= self.sensor_timeout
        )

        if cmd_fresh and event_fresh:
            linear, angular = filter_velocity(
                self.last_cmd.linear.x,
                self.last_cmd.angular.z,
                self.safety_state,
                self.limits,
            )
            output.linear.x = float(linear)
            output.angular.z = float(angular)

        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticSafetyNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
