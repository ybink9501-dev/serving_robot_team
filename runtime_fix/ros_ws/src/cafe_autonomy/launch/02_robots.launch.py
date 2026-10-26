"""Add two separately namespaced robots and explicit Nav2/Safety/Fleet path to running cafe world.
This file does not start a simulator process.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('cafe_autonomy'))
    wp=str(share/'config/waypoints.yaml')
    runtime=str(share/'config/runtime_limits.yaml')
    zones=str(share/'config/zones.yaml')
    nav_bt=str(share/'config/navigate_no_reverse.xml')
    actions=[DeclareLaunchArgument('map_yaml',default_value=str(share/'maps/cafe.yaml'),description='Validated saved SLAM map for OPERATION'),DeclareLaunchArgument('robot02',default_value='true',description='Start both robots by default'),
             LogInfo(msg='[ROBOTS] World must already exist. Robot 01 and Robot 02 are independently namespaced.')]
    for idx,robot in enumerate(('robot_01','robot_02')):
        # robot02 is the baseline full project; use launch 02_robots_single.launch.py for one robot.
        x,y,a=((-3.2,-2.25,0.) if idx==0 else (2.4,-2.25,3.14159265))
        name=robot
        params=str(share/f'config/nav2_{robot}.yaml')
        # Create runs on existing Gazebo world, not Gazebo simulator process.
        spawn=Node(package='ros_gz_sim',executable='create',name='spawn_'+robot,output='screen',
            arguments=['-world','cafe','-file',str(share/f'models/{robot}.sdf'),'-name',robot,
                       '-x',str(x),'-y',str(y),'-z','0.18','-Y',str(a)])
        bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='bridge_'+robot,
            arguments=[f'/{robot}/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                       f'/model/{robot}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                       *[f'/{robot}/{view}/{sensor}@sensor_msgs/msg/Image[gz.msgs.Image'
                         for view in ('front','rear_left','rear_right') for sensor in ('rgb','depth')],
                       f'/model/{robot}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            remappings=[(f'/model/{robot}/odometry',f'/{robot}/odom'),
                        (f'/model/{robot}/cmd_vel',f'/{robot}/cmd_vel')],output='screen')
        actions += [TimerAction(period=1.0+idx*3.0,actions=[spawn]),bridge]
        def n(executable,package='cafe_autonomy',param=None, remaps=None, name=None):
            return Node(package=package,executable=executable,name=name or executable,
                        namespace=robot,parameters=param or [{'use_sim_time':True}],
                        remappings=remaps or [],output='screen')
        actions += [
            n('scan_reframe',param=[{'use_sim_time':True,'robot_id':robot}]),
            n('odom_tf',param=[{'use_sim_time':True,'robot_id':robot}]),
            Node(package='tf2_ros',executable='static_transform_publisher',namespace=robot,
                 name='laser_static_tf',arguments=['--x','0.06','--y','0','--z','0.16',
                  '--roll','0','--pitch','0','--yaw','0',
                  '--frame-id',robot+'/base_footprint','--child-frame-id',robot+'/laser_link']),
            n('map_server',package='nav2_map_server',name='map_server',param=[params,
              {'use_sim_time':True,'yaml_filename':LaunchConfiguration('map_yaml')}]),
            n('amcl',package='nav2_amcl',name='amcl',param=[params]),
            n('lifecycle_manager',package='nav2_lifecycle_manager',name='lifecycle_manager_localization',
              param=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','amcl']}]),
            n('pose_initializer',param=[{'use_sim_time':True,'x':x,'y':y,'yaw':a}]),
            n('planner_server',package='nav2_planner',name='planner_server',param=[params]),
            n('controller_server',package='nav2_controller',name='controller_server',param=[params],
              remaps=[('cmd_vel','cmd_vel_nav')]),
            n('behavior_server',package='nav2_behaviors',name='behavior_server',param=[params],
              remaps=[('cmd_vel','cmd_vel_nav')]),
            n('bt_navigator',package='nav2_bt_navigator',name='bt_navigator',param=[params,
              {'default_nav_to_pose_bt_xml':nav_bt}]),
            n('velocity_smoother',package='nav2_velocity_smoother',name='velocity_smoother',param=[params],
              remaps=[('cmd_vel','cmd_vel_nav'),('cmd_vel_smoothed','cmd_vel_nav_smooth')]),
            n('collision_monitor',package='nav2_collision_monitor',name='collision_monitor',param=[params]),
            n('lifecycle_manager',package='nav2_lifecycle_manager',name='lifecycle_manager_navigation',
              param=[{'use_sim_time':True,'autostart':True,'node_names':[
                'planner_server','controller_server','behavior_server','bt_navigator',
                'velocity_smoother','collision_monitor']}]),
            n('semantic_safety',param=[{'use_sim_time':True,'robot_id':robot,'runtime_limits_file':runtime}]),
            n('mission',param=[{'use_sim_time':True,'robot_id':robot,'waypoints_file':wp,'runtime_limits_file':runtime}]),
        ]
    actions += [
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='cafe_gazebo_truth_bridge',
             arguments=['/world/cafe/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/world/cafe/pose/info','/cafe/gazebo_pose_truth')],output='screen'),
        Node(package='cafe_autonomy',executable='metrics',name='cafe_metrics',output='screen',
             parameters=[{'use_sim_time':True}]),
        Node(package='cafe_autonomy',executable='people',name='people_simulator',output='screen',
             parameters=[{'use_sim_time':True,'enabled':False,'count':0}]),
        Node(package='cafe_autonomy',executable='fleet',name='cafe_fleet',output='screen',
             parameters=[{'use_sim_time':True,'http_port':8765,'runtime_limits_file':runtime,'zones_file':zones}]),
    ]
    return LaunchDescription(actions)
