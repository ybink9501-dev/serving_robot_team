"""Gazebo world ONLY. Robot entities are created solely by 02_robots.launch.py."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('cafe_autonomy'))
    gz=Path(get_package_share_directory('ros_gz_sim'))/'launch'/'gz_sim.launch.py'
    return LaunchDescription([
        LogInfo(msg='[WORLD] One cafe Gazebo instance; no robot spawn from 01_world'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz)),launch_arguments={
            'gz_args': '-r -v 3 '+str(share/'worlds/cafe.sdf'),
            'on_exit_shutdown':'true'}.items()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='gazebo_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],output='screen')
    ])
