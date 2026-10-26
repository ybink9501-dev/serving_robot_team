import os
import math
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml



def _speed_overrides(limits_file=None):
    if limits_file is None:
        for parent in Path(__file__).resolve().parents:
            candidate = parent / "config/common/runtime_limits.yaml"
            if candidate.is_file():
                limits_file = candidate
                break
        else:
            raise FileNotFoundError(
                "config/common/runtime_limits.yaml not found"
            )

    limits = yaml.safe_load(Path(limits_file).read_text())

    linear = float(limits["max_linear_mps"])
    angular = float(limits["max_angular_rps"])
    slow = float(limits["slow_linear_mps"])

    if (
        not all(math.isfinite(v) for v in (linear, angular, slow))
        or linear <= 0
        or angular <= 0
        or not 0 <= slow <= linear
    ):
        raise ValueError("Invalid common speed limits")

    return {
        "controller_server": {
            "FollowPath.vx_max": linear,
            "FollowPath.vx_min": 0.0,
            "FollowPath.vy_max": 0.0,
            "FollowPath.wz_max": angular,
        },
        "behavior_server": {
            "max_rotational_vel": angular,
            "min_rotational_vel": min(0.4, angular),
        },
        "velocity_smoother": {
            "max_velocity": [linear, 0.0, angular],
            "min_velocity": [0.0, 0.0, -angular],
        },
    }

def generate_launch_description():
    enabled = LaunchConfiguration("start_navigation")
    sim_time = LaunchConfiguration("use_sim_time")

    share = get_package_share_directory("serving_robot_navigation")
    config = os.path.join(share, "config", "nav2.yaml")
    speed_overrides = _speed_overrides()

    params = ParameterFile(
        RewrittenYaml(
            source_file=config,
            root_key="robot_01",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    components = [
        ("nav2_planner", "planner_server", []),
        ("nav2_controller", "controller_server",
         [("cmd_vel", "cmd_vel_nav")]),
        ("nav2_behaviors", "behavior_server",
         [("cmd_vel", "cmd_vel_nav")]),
        ("nav2_bt_navigator", "bt_navigator", []),
        ("nav2_velocity_smoother", "velocity_smoother",
         [("cmd_vel", "cmd_vel_nav")]),
    ]

    actions = [
        DeclareLaunchArgument(
            "start_navigation",
            default_value="false",
            description="Disabled until safety integration is verified",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
    ]

    names = []

    for package, executable, remappings in components:
        names.append(executable)
        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            namespace="robot_01",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[
                params,
                speed_overrides.get(executable, {}),
                {"use_sim_time": ParameterValue(sim_time, value_type=bool)},
            ],
            remappings=remappings,
        ))

    actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace="robot_01",
        output="screen",
        condition=IfCondition(enabled),
        parameters=[{
            "use_sim_time": ParameterValue(sim_time, value_type=bool),
            "autostart": False,
            "node_names": names,
        }],
    ))

    return LaunchDescription(actions)
