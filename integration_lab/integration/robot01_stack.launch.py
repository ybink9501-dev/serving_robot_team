#!/usr/bin/env python3
"""Robot01 LAB terminal consolidation. Reuses existing launch files unmodified.

IMPORTANT: start this ONLY after stopping former spawn/nav/localization/people/safety
launches. The running V3 supervisor and V6/V7/V10 detached guards are not re-created.
No navigation goal or cmd_vel is sent by this file.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAB = Path.home() / 'team_robot_lab'
V3 = LAB / 'serving_v3'

# Core components this launch OWNS. It must not start alongside their old launches.
OWNED_NODE_NAMES = {
    '/robot_state_publisher',
    '/serving_robot_bridge',
    '/map_server',
    '/robot_01/amcl',
    '/robot_01/planner_server',
    '/robot_01/controller_server',
    '/robot_01/behavior_server',
    '/robot_01/bt_navigator',
    '/robot_01/velocity_smoother',
    '/robot_01/semantic_safety',
    '/robot_01/collision_monitor',
    '/person_simulator_manager',
    '/person_simulator_watchdog',
    '/people_bridge',
    '/gazebo_person_event_bridge',
}


def _checked_file(p):
    p = Path(p).expanduser().resolve()
    if not p.is_file():
        raise RuntimeError('Missing existing launch or source file: ' + str(p))
    return p


def _existing_launch(package, name):
    share = Path(get_package_share_directory(package))
    return _checked_file(share / 'launch' / name)


def _included(filename, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(filename)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _assert_no_duplicates():
    if os.environ.get('ROS_DOMAIN_ID') != '99':
        raise RuntimeError('STOP: ROS_DOMAIN_ID must be 99')
    if os.environ.get('GZ_PARTITION') != 'team_robot_lab_gazebo_99':
        raise RuntimeError('STOP: GZ_PARTITION must match current LAB partition')
    try:
        r = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon'],
            capture_output=True, text=True, timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        raise RuntimeError('STOP: cannot verify existing ROS node ownership') from e
    if r.returncode:
        raise RuntimeError('STOP: ros2 node list failed: ' + r.stderr[-500:])
    running = set(r.stdout.splitlines())
    duplicates = sorted(running & OWNED_NODE_NAMES)
    if duplicates:
        raise RuntimeError(
            'STOP: old launch(es) still running; do not start duplicates: '
            + ', '.join(duplicates)
            + '. Stop the original spawn, Nav2, Localization, people and Safety '
            'launch terminals first. Keep Gazebo running.'
        )


def _setup(context, *_args, **_kwargs):
    _assert_no_duplicates()

    chosen_map = _checked_file(LaunchConfiguration('map').perform(context))
    manifest_path = _checked_file(V3 / 'generated' / 'manifest.json')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    expected_map = _checked_file(manifest['map'])
    if chosen_map != expected_map:
        raise RuntimeError(
            'STOP: selected map differs from installed V3 serving snapshot. '
            f'Chosen: {chosen_map}, V3: {expected_map}'
        )

    reuse_setting = LaunchConfiguration("reuse_existing_robot").perform(context).lower()
    if reuse_setting not in ("true", "false"):
        raise RuntimeError("reuse_existing_robot must be true or false")
    reuse_existing = reuse_setting == "true"

    gazebo_pkg = Path(get_package_share_directory('serving_robot_gazebo'))
    spawns = [gazebo_pkg / 'launch' / name for name in
              ('spawn_robot.launc.py', 'spawn_robot.launch.py')]
    matches = [p for p in spawns if p.is_file()]
    if not reuse_existing and not matches:
        raise RuntimeError('STOP: missing installed Robot01 spawn launch: '
                           + ', '.join(str(p) for p in spawns))
    # The user's observed running launch was spawn_robot.launc.py; if both
    # historic spellings exist prefer that exact known live command.
    chosen_spawn = matches[0] if matches else None

    person_launch = _existing_launch('serving_robot_person_simulator', 'people.launch.py')
    localization = _existing_launch('serving_robot_navigation', 'localization_preflight.launch.py')
    nav2_trial = _checked_file(LAB / 'nav2_rotation_trial.launch.py')
    safety = _checked_file(
        LAB / 'ros_ws/src/serving_robot_navigation/launch/gazebo_safety_lab.launch.py'
    )
    people_bridge = _checked_file(
        LAB / 'ros_ws/src/serving_robot_person_simulator/gazebo_person_event_bridge.py'
    )

    # Gazebo does NOT delete an existing entity just because its original
    # ros2 launch / ros_gz_sim create parent exited. Reuse the existing entity
    # without calling create a second time. Only start RSP and the ROS/GZ bridge.
    if reuse_existing:
        import xacro
        description_share = Path(get_package_share_directory('serving_robot_description'))
        xacro_file = _checked_file(description_share / 'urdf' / 'serving_robot.urdf.xacro')
        bridge_config = _checked_file(gazebo_pkg / 'config' / 'bridge.yaml')
        robot_description = xacro.process_file(
            str(xacro_file), mappings={'robot_name': 'robot_01'}
        ).toxml()
        robot_actions = [
            LogInfo(msg='[Robot01 STACK] REUSE: no Gazebo entity creation; robot_01 must already exist'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': True,
                }],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='serving_robot_bridge',
                output='screen',
                parameters=[{'config_file': str(bridge_config)}],
            ),
        ]
    else:
        robot_actions = [
            LogInfo(msg='[Robot01 STACK] SPAWN: creating robot_01 for a new empty Gazebo world'),
            _included(chosen_spawn),
        ]

    # Two physical terminals: Gazebo separately, this stack in the second.
    # Time offsets only stagger startup; they are NOT proof of ROS readiness.
    # No GO or initialpose is emitted by this launch.
    return [
        LogInfo(msg=f'[Robot01 STACK] map={chosen_map}'),
        LogInfo(msg='[Robot01 STACK] Gazebo must be up already; verify safety/Nav2 before any mission.'),
        *robot_actions,
        TimerAction(period=5.0, actions=[_included(person_launch)]),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(cmd=[sys.executable, '-u', str(people_bridge)],
                           output='screen'),
            _included(safety),
        ]),
        TimerAction(period=10.0, actions=[
            _included(localization, {'map': str(chosen_map)}),
        ]),
        TimerAction(period=16.0, actions=[_included(nav2_trial)]),
        TimerAction(period=20.0, actions=[
            ExecuteProcess(cmd=[sys.executable, '-u', str(LAB / 'integration' / 'robot01_ready.py')], output='screen'),
        ]),
    ]


def generate_launch_description():
    default_map = str(LAB / 'lab_maps/generated/cafe_full_provisional.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('reuse_existing_robot', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
