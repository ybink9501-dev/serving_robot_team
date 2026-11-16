"""Same TEAM robot chassis, dedicated TEAM mapping system; no serving stack."""
from pathlib import Path
import math,sys,yaml
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,OpaqueFunction,IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,EmitEvent,LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetRemap

def mapping_precision_parameters(slam, navigation, resolution, tolerance, bootstrap=False, align_clearance=False):
    """Only mapping resolution/arrival precision; motion limits remain unchanged."""
    import copy
    resolution=float(resolution);tolerance=float(tolerance)
    if not all(math.isfinite(v) and v>0 for v in (resolution,tolerance)):
        raise ValueError('Invalid mapping precision')
    # Same static-map triangle bound as the Mapping .52m goal / .48m path guards.
    if tolerance>min(resolution/2., .52-.48):
        raise ValueError('Mapping arrival error exceeds grid/clearance budget')
    slam=copy.deepcopy(slam);navigation=copy.deepcopy(navigation)
    slam['slam_toolbox']['ros__parameters']['resolution']=resolution
    navigation['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']=tolerance
    if bootstrap:
        slam['slam_toolbox']['ros__parameters']['check_min_dist_and_heading_precisely']=True
        behavior=navigation['behavior_server']['ros__parameters']
        behavior['behavior_plugins']=list(behavior['behavior_plugins'])+['mapping_scan_spin']
        behavior['mapping_scan_spin']={'plugin':'nav2_behaviors::Spin'}
        behavior['max_rotational_vel']=.30
        behavior['min_rotational_vel']=.05
    if align_clearance:
        import json
        costmap=navigation['global_costmap']['global_costmap']['ros__parameters']
        costmap_resolution=float(costmap['resolution'])
        if not math.isfinite(costmap_resolution) or costmap_resolution<=0:
            raise ValueError('Invalid mapping planner grid resolution')
        # Preserve the .48m path check including map half-diagonal, and
        # budget a costmap half-diagonal for interpolated path positions.
        # Also preserve the .52m goal check including map half-diagonal.
        from serving_robot_mapping.mapping_geometry import MappingGeometry
        radius=MappingGeometry(resolution=resolution,costmap_resolution=costmap_resolution,
                               arrival_error=tolerance).planner_radius
        vertices=16
        outer=math.nextafter(radius/math.cos(math.pi/vertices),math.inf)
        costmap['footprint']=json.dumps([[outer*math.cos(2*math.pi*i/vertices),
                                        outer*math.sin(2*math.pi*i/vertices)] for i in range(vertices)])
        costmap['footprint_padding']=0.
        costmap['inflation_layer']['inflation_radius']=max(
            float(costmap['inflation_layer']['inflation_radius']),radius+2*costmap_resolution)
        navigation['planner_server']['ros__parameters']['GridBased']['allow_unknown']=False
    return slam,navigation

def mapping_setup(context):
    config=Path(LaunchConfiguration('config').perform(context));c=yaml.safe_load(config.read_text())
    world=Path(c.get('world_file') or Path(get_package_share_directory('serving_robot_gazebo'))/'worlds/cafe_closed.world.sdf')
    generated=config.parent/'mapping_robot_start.yaml';pose=c['robots']['robot_01']
    generated.write_text(yaml.safe_dump({'robot_01':{'start_pose':{'x':pose['x'],'y':pose['y'],'yaw_deg':math.degrees(pose['yaw'])}}}),encoding='utf-8')
    gz_args='-r -v3 '+str(world)
    if LaunchConfiguration('gui').perform(context)=='false':gz_args='-s --headless-rendering '+gz_args
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':gz_args,'on_exit_shutdown':'true'}.items())
    mapping_args={'world':'false','rviz':'false','start_config':str(generated),'store':'demo_store','wait_timeout':'180'}
    if 'mapping_grid_resolution_m' in c or 'mapping_goal_tolerance_m' in c:
        import json,hashlib
        nav_share=Path(get_package_share_directory('serving_robot_navigation'))
        slam_source=nav_share/'config/slam.yaml';nav_source=nav_share/'config/nav2.yaml'
        slam,nav=mapping_precision_parameters(yaml.safe_load(slam_source.read_text()),
            yaml.safe_load(nav_source.read_text()),c['mapping_grid_resolution_m'],c['mapping_goal_tolerance_m'],bool(c.get('mapping_bootstrap_yaw_rad')),bool(c.get('mapping_planner_clearance_alignment')))
        if c.get('mapping_oriented_observation',False):
            from serving_robot_mapping.mapping_oriented_geometry import mapping_oriented_parameters
            nav=mapping_oriented_parameters(nav)
        slam_path=config.parent/'mapping_slam_params.yaml';nav_path=config.parent/'mapping_nav2_params.yaml'
        slam_path.write_text(yaml.safe_dump(slam));nav_path.write_text(yaml.safe_dump(nav))
        mapping_args.update(slam_params_file=str(slam_path),nav2_params_file=str(nav_path))
        (config.parent/'mapping_precision_provenance.json').write_text(json.dumps({
            'mapping_only':True,'resolution_m':c['mapping_grid_resolution_m'],
            'goal_tolerance_m':c['mapping_goal_tolerance_m'],
            'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (slam_source,nav_source)},
            'changed_parameters':['slam_toolbox.resolution','controller_server.general_goal_checker.xy_goal_tolerance']+(
                ['slam_toolbox.check_min_dist_and_heading_precisely','behavior_server.behavior_plugins',
                 'behavior_server.mapping_scan_spin','behavior_server.max_rotational_vel','behavior_server.min_rotational_vel']
                if c.get('mapping_bootstrap_yaw_rad') else [])+(
                ['global_costmap.footprint','global_costmap.footprint_padding','global_costmap.inflation_layer.inflation_radius',
                 'planner_server.GridBased.allow_unknown'] if c.get('mapping_planner_clearance_alignment') else [])},indent=2))
    team=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('serving_robot_bringup'))/'launch/mapping_mode.launch.py')),
        launch_arguments=mapping_args.items())
    wait=ExecuteProcess(cmd=[sys.executable,str(Path(get_package_prefix('serving_robot_bringup'))/'lib/serving_robot_bringup/wait_for_ready.py'),'--name','mapping_world','--timeout','180','--gz-topic','/clock'],output='screen')
    def after_world(event,context):
        if event.returncode==0:return [LogInfo(msg='Personal mapping role: reuse TEAM SLAM + Nav2 + frontier stack'),GroupAction([SetRemap(src='serving_robot_bridge:/clock',dst='/mapping/raw_clock'),team])]
        return [EmitEvent(event=Shutdown(reason='Mapping world did not become ready'))]
    return [gazebo,Node(package='cafe_autonomy_gazebo',executable='team_sim_clock',name='team_sim_clock'),RegisterEventHandler(OnProcessExit(target_action=wait,on_exit=after_world)),wait]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config'),DeclareLaunchArgument('gui',default_value='true'),OpaqueFunction(function=mapping_setup)])
