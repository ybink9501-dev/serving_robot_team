from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(
        get_package_share_directory("serving_robot_navigation")
    )

    enabled = LaunchConfiguration("start_autonomy")
    sim_time = LaunchConfiguration("use_sim_time")

    navigation_launch = share / "launch/navigation.launch.py"
    collision_launch = share / "launch/collision_monitor.launch.py"

    for path in (navigation_launch, collision_launch):
        if not path.is_file():
            raise FileNotFoundError(path)

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_autonomy",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "person_event_topic",
            default_value="",
        ),
        DeclareLaunchArgument(
            "limits_file",
            default_value="",
        ),
        DeclareLaunchArgument(
            "sensor_timeout_s",
            default_value="1.0",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(navigation_launch)
            ),
            condition=IfCondition(enabled),
            launch_arguments={
                "start_navigation": "true",
                "use_sim_time": sim_time,
            }.items(),
        ),

        Node(
            package="serving_robot_perception",
            executable="semantic_safety",
            namespace="robot_01",
            name="semantic_safety",
            output="screen",
            condition=IfCondition(enabled),
            parameters=[{
                "robot_id": "robot_01",
                "person_event_topic": LaunchConfiguration(
                    "person_event_topic"
                ),
                "limits_file": LaunchConfiguration(
                    "limits_file"
                ),
                "sensor_timeout_s": ParameterValue(
                    LaunchConfiguration("sensor_timeout_s"),
                    value_type=float,
                ),
                "use_sim_time": ParameterValue(
                    sim_time,
                    value_type=bool,
                ),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(collision_launch)
            ),
            condition=IfCondition(enabled),
            launch_arguments={
                "start_collision_monitor": "true",
                "use_sim_time": sim_time,
            }.items(),
        ),
    ])
