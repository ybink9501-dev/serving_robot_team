from pathlib import Path
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

ROOT = Path.home() / "team_robot_lab"
NAV = ROOT / "ros_ws/src/serving_robot_navigation"

NAV2_CONFIG = NAV / "config/nav2.yaml"
COLLISION_CONFIG = NAV / "config/collision_monitor.yaml"
LIMITS_CONFIG = ROOT / "config/common/runtime_limits.yaml"

def generate_launch_description():
    for path in (NAV2_CONFIG, COLLISION_CONFIG, LIMITS_CONFIG):
        if not path.is_file():
            raise FileNotFoundError(path)

    nav2 = yaml.safe_load(NAV2_CONFIG.read_text())
    limits = yaml.safe_load(LIMITS_CONFIG.read_text())

    params = dict(nav2["velocity_smoother"]["ros__parameters"])

    max_v = float(limits["max_linear_mps"])
    max_w = float(limits["max_angular_rps"])

    if not (0 < max_v <= 1.0 and 0 < max_w <= 3.0):
        raise ValueError("시험용 속도 범위 확인 필요")

    params.update({
        "max_velocity": [max_v, 0.0, max_w],
        "min_velocity": [0.0, 0.0, -max_w],
        "feedback": "OPEN_LOOP",
        "use_sim_time": False,
    })

    return LaunchDescription([
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            namespace="robot_01",
            name="velocity_smoother",
            parameters=[params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
            output="screen",
        ),
        Node(
            package="serving_robot_perception",
            executable="semantic_safety",
            namespace="robot_01",
            name="semantic_safety",
            parameters=[{
                "robot_id": "robot_01",
                "person_event_topic": "/robot_01/person_safety_event_lab",
                "limits_file": str(LIMITS_CONFIG),
                "sensor_timeout_s": 1.0,
                "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            namespace="robot_01",
            name="collision_monitor",
            parameters=[
                str(COLLISION_CONFIG),
                {"use_sim_time": False},
            ],
            output="screen",
        ),
    ])
