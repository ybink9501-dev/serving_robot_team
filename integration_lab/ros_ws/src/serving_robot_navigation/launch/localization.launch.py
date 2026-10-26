import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    amcl_params_file = LaunchConfiguration('amcl_params_file')

    navigation_share = get_package_share_directory(
        'serving_robot_navigation'
    )

    default_amcl_params_file = os.path.join(
        navigation_share,
        'config',
        'amcl.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Required path to saved map.yaml',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=default_amcl_params_file,
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace='robot_01',
            name='amcl',
            output='screen',
            parameters=[
                amcl_params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('scan', '/robot_01/scan'),
                ('map', '/map'),
                ('initialpose', '/initialpose'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace='robot_01',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl'],
            }],
        ),
    ])
