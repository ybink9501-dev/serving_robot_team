"""MAPPING-only: robot_01 assisted LiDAR SLAM.
Do NOT run 02_robots.launch.py in same domain: AMCL and SLAM would publish map->odom.
World is started separately by 01_world.launch.py.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('cafe_autonomy'))
    robot='robot_01'
    nav=str(p/'config/nav2_robot_01.yaml')
    return LaunchDescription([
        LogInfo(msg='MAPPING MODE: AMCL/Nav2 operation launch MUST be stopped. Robot 01 only.'),
        TimerAction(period=2.0,actions=[Node(package='ros_gz_sim',executable='create',name='spawn_mapping_robot',
            arguments=['-world','cafe','-file',str(p/'models/robot_01.sdf'),'-name',robot,
                       '-x','-3.2','-y','-2.25','-z','0.18','-Y','0'],output='screen')]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='mapping_bridge',
            arguments=['/robot_01/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                       '/model/robot_01/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                       '/model/robot_01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            remappings=[('/model/robot_01/odometry','/robot_01/odom'),
                        ('/model/robot_01/cmd_vel','/robot_01/cmd_vel')],output='screen'),
        Node(package='cafe_autonomy',executable='scan_reframe',namespace=robot,name='scan_reframe',
             parameters=[{'robot_id':robot,'use_sim_time':True}]),
        Node(package='cafe_autonomy',executable='odom_tf',namespace=robot,name='odom_tf',
             parameters=[{'robot_id':robot,'use_sim_time':True}]),
        Node(package='tf2_ros',executable='static_transform_publisher',namespace=robot,
             name='laser_static_tf',arguments=['--x','0.06','--y','0','--z','0.16',
             '--roll','0','--pitch','0','--yaw','0',
             '--frame-id','robot_01/base_footprint','--child-frame-id','robot_01/laser_link']),
        Node(package='slam_toolbox',executable='async_slam_toolbox_node',namespace=robot,name='slam_toolbox',
             parameters=[str(p/'config/slam_mapping.yaml')],output='screen'),
        Node(package='nav2_collision_monitor',executable='collision_monitor',namespace=robot,name='collision_monitor',
             parameters=[nav],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace=robot,
             name='lifecycle_manager_mapping_safety',parameters=[{
                'use_sim_time':True,'autostart':True,'node_names':['collision_monitor']}],output='screen'),
    ])
