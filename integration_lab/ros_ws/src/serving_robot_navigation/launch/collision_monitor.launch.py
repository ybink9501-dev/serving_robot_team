
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration("start_collision_monitor")
    sim_time = LaunchConfiguration("use_sim_time")

    share = get_package_share_directory("serving_robot_navigation")
    config = os.path.join(share, "config", "collision_monitor.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_collision_monitor",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            namespace="robot_01",
            name="collision_monitor",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[
                config,
                {"use_sim_time": ParameterValue(sim_time, value_type=bool)},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="robot_01",
            name="lifecycle_manager_collision",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[{
                "use_sim_time": ParameterValue(sim_time, value_type=bool),
                "autostart": False,
                "node_names": ["collision_monitor"],
            }],
        ),
    ])
