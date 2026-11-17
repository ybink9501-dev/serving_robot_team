import copy
import importlib.util
from pathlib import Path
import pytest

path=Path(__file__).resolve().parents[2]/'cafe_autonomy/launch/mapping_robot.launch.py'
spec=importlib.util.spec_from_file_location('mapping_robot_precision_test',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_only_mapping_geometry_and_tolerance_change():
    slam={'slam_toolbox':{'ros__parameters':{'resolution':.05,'transform_timeout':.2,'max_laser_range':12.}}}
    nav={'controller_server':{'ros__parameters':{'general_goal_checker':{'xy_goal_tolerance':.15,'yaw_goal_tolerance':.15},'max_speed':.25}},'safety':{'stop':.1}}
    old_slam,old_nav=copy.deepcopy(slam),copy.deepcopy(nav)
    new_slam,new_nav=module.mapping_precision_parameters(slam,nav,.025,.01)
    assert slam==old_slam and nav==old_nav
    assert new_slam['slam_toolbox']['ros__parameters']['resolution']==.025
    assert new_nav['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']==.01
    new_slam['slam_toolbox']['ros__parameters']['resolution']=.05
    new_nav['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']=.15
    assert new_slam==slam and new_nav==nav


@pytest.mark.parametrize('resolution,tolerance',[(.025,.15),(.025,.02),(-.05,.01),(.025,float('nan'))])
def test_precision_cannot_exceed_clearance_or_cell_budget(resolution,tolerance):
    with pytest.raises(ValueError):module.mapping_precision_parameters({}, {},resolution,tolerance)


def test_delayed_stage_keeps_resolved_mapping_parameter_paths():
    from launch import LaunchContext
    mode_path=Path(__file__).resolve().parents[2]/'serving_robot_bringup/launch/mapping_mode.launch.py'
    mode_spec=importlib.util.spec_from_file_location('mapping_mode_scope_test',mode_path)
    mode=importlib.util.module_from_spec(mode_spec);mode_spec.loader.exec_module(mode)
    context=LaunchContext()
    context.launch_configurations.update(slam_params_file='/temporary/slam.yaml',nav2_params_file='/temporary/nav.yaml')
    files=mode.mapping_param_files(context)
    context.launch_configurations.clear()
    assert files==('/temporary/slam.yaml','/temporary/nav.yaml')
    assert all(isinstance(p,str) for p in files)


def test_bootstrap_behavior_does_not_enable_general_spin_recovery():
    slam={'slam_toolbox':{'ros__parameters':{'resolution':.05}}}
    nav={'controller_server':{'ros__parameters':{'general_goal_checker':{'xy_goal_tolerance':.15}}},
         'behavior_server':{'ros__parameters':{'behavior_plugins':['wait']}}}
    updated_slam,updated=module.mapping_precision_parameters(slam,nav,.025,.01,True)
    behavior=updated['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins']==['wait','mapping_scan_spin']
    assert 'spin' not in behavior['behavior_plugins']
    assert behavior['max_rotational_vel']<=.30
    assert nav['behavior_server']['ros__parameters']=={'behavior_plugins':['wait']}
    assert updated_slam['slam_toolbox']['ros__parameters']['check_min_dist_and_heading_precisely'] is True
    assert 'check_min_dist_and_heading_precisely' not in slam['slam_toolbox']['ros__parameters']


def test_mapping_observation_spin_is_available_without_enabling_operation_recovery():
    nav={'behavior_server':{'ros__parameters':{'behavior_plugins':['wait']}}}
    updated=module.mapping_scan_spin_parameters(nav)
    behavior=updated['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins']==['wait','mapping_scan_spin']
    assert behavior['mapping_scan_spin']['plugin']=='nav2_behaviors::Spin'
    assert behavior['max_rotational_vel']==.30
    assert behavior['min_rotational_vel']==.05
    assert nav['behavior_server']['ros__parameters']=={'behavior_plugins':['wait']}


def test_mapping_observation_spin_rejects_unbounded_rotation():
    with pytest.raises(ValueError):
        module.mapping_scan_spin_parameters(
            {'behavior_server':{'ros__parameters':{'behavior_plugins':['wait']}}},
            max_angular_speed=.31)
