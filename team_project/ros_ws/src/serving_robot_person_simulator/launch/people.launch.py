"""Start only the independent people subsystem; the robot stack is untouched."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def nodes(context, *args, **kwargs):
    default=os.path.join(get_package_share_directory('serving_robot_person_simulator'),'config','policy.yaml')
    config=LaunchConfiguration('config').perform(context)
    if not config:
        config=default
    with open(config,encoding='utf-8') as f:
        cfg=yaml.safe_load(f)
    enabled_raw=LaunchConfiguration('deadlock_resolution').perform(context).strip().lower()
    overlay_arg=LaunchConfiguration('deadlock_overlay').perform(context).strip()
    if enabled_raw not in ('true','false'):
        raise ValueError("deadlock_resolution must be 'true' or 'false', got " + repr(enabled_raw))
    enabled=enabled_raw=='true'
    if not enabled and overlay_arg:
        raise ValueError('deadlock_overlay requires deadlock_resolution:=true')
    if enabled:
        overlay=overlay_arg or os.path.join(
            get_package_share_directory('serving_robot_person_simulator'),
            'config','deadlock_resolution.yaml')
        if not os.path.isfile(overlay):
            raise FileNotFoundError('deadlock overlay not found: '+overlay)
    encounter=LaunchConfiguration('robot_encounter').perform(context).strip().lower()
    if encounter not in ('yield','mixed'):
        raise ValueError('robot_encounter must be yield or mixed')
    world=cfg['world']
    prefix=cfg['model_prefix']
    if prefix!='test_person_':
        raise ValueError('Unexpected model prefix')
    # Single bridge process, max 8 person velocity feeds only.
    # Gazebo named poses are read by manager's independent gz CLI stream.
    topics = [f'/model/{prefix}{i:02d}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
               for i in range(1,9)]
    params={'config_path':config,'use_sim_time':True,'robot_encounter':encounter}
    if enabled:
        params['overlay_config_path']=overlay
    actions=[
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='people_bridge',
             arguments=topics,output='screen'),
        Node(package='serving_robot_person_simulator',executable='watchdog',name='person_simulator_watchdog',output='screen'),
        Node(package='serving_robot_person_simulator',executable='manager',name='person_simulator_manager',
             parameters=[params],output='screen'),
    ]
    if enabled:
        actions.insert(0,LogInfo(msg=(
            'TEST-ONLY deadlock overlay: '+overlay+
            '; minimum approach 0.50m and timeout 10s are test values pending Gazebo validation'
            '; person 0.80/1.00m and robot 0.85/1.05m remain unchanged')))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value='',description='Absolute path to approved policy.yaml'),
        DeclareLaunchArgument('robot_encounter',default_value='yield',
            description='Optional robot encounter behavior: yield or mixed'),
        DeclareLaunchArgument('deadlock_resolution',default_value='false',
            description="'true' enables the TEST-ONLY person-person deadlock breaker (default 'false')"),
        DeclareLaunchArgument('deadlock_overlay',default_value='',
            description='Optional absolute path of an overlay yaml; only valid with deadlock_resolution:=true'),
        OpaqueFunction(function=nodes),
    ])
