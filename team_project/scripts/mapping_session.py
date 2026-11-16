"""Own a TEAM exploration session for the dedicated mapping role; save drafts."""
import argparse,fcntl,json,math,os,signal,subprocess,time,shutil,hashlib,yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from serving_robot_navigation.interfaces import names_for

root=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--run',action='store_true',help='Existing Backend mapping session compatibility');ap.add_argument('--max-seconds',type=int,default=0);ap.add_argument('--ready-timeout',type=float,default=180.);ap.add_argument('--log-dir');ap.add_argument('--boundary-contract',default='');args=ap.parse_args()
if args.ready_timeout<=0:raise SystemExit('Use a positive --ready-timeout')
if args.max_seconds<0:raise SystemExit('Use --max-seconds 0 for natural completion or a positive test bound')
runtime=root/'_local_only/runtime.json'
entry=json.loads(runtime.read_text())
if entry.get('mode')!='mapping' or entry.get('mapping_stack')!='team':raise SystemExit('Start scripts/start_mapping.sh first; operation and the SLAM-only lab are different roles')
config=yaml.safe_load(Path(entry['config']).read_text());map_path=Path(config['map_yaml'])
if not args.boundary_contract:
    args.boundary_contract = config.get('mapping_boundary_contract', '')
if args.boundary_contract:
    boundary_path = Path(args.boundary_contract)
    args.boundary_contract = str(boundary_path if boundary_path.is_absolute() else root/boundary_path)
if args.boundary_contract:
    from serving_robot_mapping.mapping_domain import load_domain
    if load_domain(args.boundary_contract)['site_id'] != entry.get('site_id'):
        raise SystemExit('Boundary contract site mismatch')
def map_digest():
    paths=[map_path]
    if map_path.exists():
        image=yaml.safe_load(map_path.read_text()).get('image')
        if image:paths.append(map_path.parent/image)
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}
original_map=map_digest()
lock=open(root/'_local_only/mapping_session.lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('A mapping session already owns the explorer')
stamp=time.strftime('%Y%m%d_%H%M%S');output=Path(args.log_dir).resolve() if args.log_dir else root/'maps/drafts'/(entry.get('site_id','site')+'_'+stamp)
if output.exists() and any(p.name!='mapping_api_runner.log' for p in output.iterdir()):raise SystemExit('Refusing to overwrite an existing mapping result')
output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('personal_mapping_session_observer');nm=names_for('robot_01');state={'map':None,'map_wall':0.,'odom':None,'odom_wall':0.,'first_odom':None,'max_odom_displacement_m':0.,'map_updates':0};stopping=False;child=None
def on_map(msg):
    state.update(map=msg,map_wall=time.monotonic());state['map_updates']+=1
def on_odom(msg):
    p=(msg.pose.pose.position.x,msg.pose.pose.position.y);state['odom']=p;state['odom_wall']=time.monotonic()
    q=msg.pose.pose.orientation;state['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    if state['first_odom'] is None:state['first_odom']=p
    state['max_odom_displacement_m']=max(state['max_odom_displacement_m'],math.dist(state['first_odom'],p))
    state['linear_speed']=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y);state['angular_speed']=abs(msg.twist.twist.angular.z)
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
subscriptions=[node.create_subscription(OccupancyGrid,'/map',on_map,q),node.create_subscription(Odometry,nm.topic('odom'),on_odom,qos_profile_sensor_data)]
subscriptions.append(node.create_subscription(LaserScan,nm.topic('scan'),lambda msg:state.update(scan=msg,scan_wall=time.monotonic()),qos_profile_sensor_data))
names=('planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor')
clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in names};pending={};sent={};active=set();before=None;error=None;returncode=None
def owner_alive():
    if not runtime.exists():return False
    try:return json.loads(runtime.read_text()).get('pid')==entry['pid'] and Path('/proc/'+str(entry['pid'])+'/cmdline').exists()
    except (OSError,ValueError):return False
def request_stop(*_):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
try:
    deadline=time.monotonic()+args.ready_timeout;next_status=0.
    while time.monotonic()<deadline:
        if stopping or not owner_alive():raise RuntimeError('Mapping owner stopped before readiness')
        rclpy.spin_once(node,timeout_sec=.1)
        for name,client in clients.items():
            future=pending.get(name)
            if future is not None and not future.done() and time.monotonic()-sent[name]>3.:
                client.remove_pending_request(future);pending.pop(name);future=None
            if future is not None and future.done():
                if future.result().current_state.id==3:active.add(name)
                else:active.discard(name)
                pending.pop(name)
            if name not in active and name not in pending and client.service_is_ready():pending[name]=client.call_async(GetState.Request());sent[name]=time.monotonic()
        nodes={'/'+name for name,ns in node.get_node_names_and_namespaces() if ns=='/'}
        forbidden=nodes&{'/cafe_fleet','/amcl','/person_simulator_manager','/person_detector','/person_state','/person_speed_limiter','/exploration_manager'}
        if time.monotonic()>=next_status:
            print('Waiting for mapping: '+str({'active':sorted(active),'services_ready':[n for n,c in clients.items() if c.service_is_ready()],'map_updates':state['map_updates'],'odom':state['odom'],'forbidden':sorted(forbidden)}),flush=True);next_status=time.monotonic()+5.
        if active==set(names) and state['map'] is not None and time.monotonic()-state['map_wall']<5. and state['odom'] is not None and time.monotonic()-state['odom_wall']<1. and not forbidden:
            before={'nodes':sorted(nodes),'map_dimensions':[state['map'].info.width,state['map'].info.height],'known_cells':sum(v>=0 for v in state['map'].data)};break
    else:raise RuntimeError('Mapping readiness timed out: lifecycle='+str(sorted(active)))
    print('Mapping role ready: same Robot 01 model; TEAM SLAM/Nav2/frontier, no serving Fleet or AMCL',flush=True)
    if config.get('mapping_wall_first'):
        from serving_robot_mapping.mapping_geometry import MappingGeometry
        generated=root/'_local_only/config'
        slam=yaml.safe_load((generated/'mapping_slam_params.yaml').read_text())
        nav=yaml.safe_load((generated/'mapping_nav2_params.yaml').read_text())
        geometry=MappingGeometry(resolution=float(config['mapping_grid_resolution_m']),
            costmap_resolution=float(nav['global_costmap']['global_costmap']['ros__parameters']['resolution']),
            arrival_error=float(config['mapping_goal_tolerance_m']))
        evidence=geometry.validate_generated(slam,nav)
        local=nav['local_costmap']['local_costmap']['ros__parameters']
        vertices=json.loads(local['footprint'])
        limit_path=Path(config['limits_file'])
        if not limit_path.is_absolute():limit_path=root/limit_path
        limits=yaml.safe_load(limit_path.read_text())
        evidence.update(geometry.validate_body(vertices,float(limits['dynamic_obstacles']['hard_stop_clearance_m']),float(limits['dynamic_obstacles']['robot_radius_m'])))
        evidence['body_footprint']=vertices
        evidence['limits_source']=str(limit_path)
        from rcl_interfaces.srv import GetParameters
        checks={}
        for name in ('slam_toolbox',)+names:
            client=node.create_client(GetParameters,'/'+name+'/get_parameters')
            if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('Clock parameter service unavailable: '+name)
            pending_clock=client.call_async(GetParameters.Request(names=['use_sim_time']))
            until=time.monotonic()+4.
            while not pending_clock.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            if not pending_clock.done() or not pending_clock.result().values or not pending_clock.result().values[0].bool_value:
                raise RuntimeError('Mapping runtime does not use simulation clock: '+name)
            checks[name]=True;node.destroy_client(client)
        evidence['runtime_use_sim_time']=checks
        (output/'mapping_geometry_contract.json').write_text(json.dumps(evidence,indent=2))
    if config.get('mapping_bootstrap_yaw_rad'):
        from serving_robot_mapping.mapping_bootstrap import run_bootstrap
        map_before=state['map'];known_before=sum(v>=0 for v in map_before.data)
        sweep=run_bootstrap(node,state,lambda:stopping or not owner_alive(),config['mapping_bootstrap_yaw_rad'])
        evidence={'steps':sweep,'known_cells_before':known_before,
                  'known_cells_after':sum(v>=0 for v in state['map'].data),'map_updates':state['map_updates']}
        (output/'bootstrap_evidence.json').write_text(json.dumps(evidence,indent=2))
        print('MAPPING_BOOTSTRAP_VERIFIED: '+json.dumps(evidence),flush=True)
    tool=root/'ros_ws/src/serving_robot_mapping/tools/run_lidar_mapping_session.py'
    # Scope the discovery workaround to this child; keep TEAM preflight checks.
    real_ros2=shutil.which('ros2');cli_dir=output/'cli';cli_dir.mkdir()
    shim=cli_dir/'ros2'
    shim.write_text('#!/usr/bin/python3\nimport os,sys\na=sys.argv[1:]\nif a[:2] in (["node","list"],["lifecycle","get"]):a += ["--no-daemon","--spin-time","3"]\nos.execv('+repr(real_ros2)+',["ros2"]+a)\n')
    shim.chmod(0o755);child_env=os.environ.copy();child_env['PATH']=str(cli_dir)+os.pathsep+child_env['PATH']
    with (output/'personal_session_console.log').open('wb') as log:
        child=subprocess.Popen(['python3',str(tool),'--run','--max-seconds',str(args.max_seconds),'--log-dir',str(output)]+(['--boundary-contract',args.boundary_contract] if args.boundary_contract else [])+(['--wall-first'] if config.get('mapping_wall_first') else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=child_env)
        stop_at=None;session_deadline=(time.monotonic()+args.max_seconds+180.) if args.max_seconds else None
        while child.poll() is None:
            rclpy.spin_once(node,timeout_sec=.1)
            if session_deadline is not None and time.monotonic()>session_deadline and stop_at is None:
                error='TEAM mapping session exceeded exploration and finalization deadline'
                stopping=True
            if (stopping or not owner_alive()) and stop_at is None:
                os.killpg(child.pid,signal.SIGINT);stop_at=time.monotonic()
            if stop_at is not None and time.monotonic()-stop_at>65.:
                os.killpg(child.pid,signal.SIGKILL);raise RuntimeError('Owned mapping finalizer did not finish')
        returncode=child.returncode
    report_path=output/'R16-147_session_result.json'
    result=json.loads(report_path.read_text()) if report_path.exists() else {}
    if (config.get('mapping_wall_first') and owner_alive()
            and result.get('nav2_shutdown_status_observed') in ('NO_ACTIVE_MAPPING_GOAL','CANCEL_CONFIRMED_TERMINAL')):
        limits=yaml.safe_load(Path(config['limits_file']).read_text())['arrival']
        stop_until=time.monotonic()+5.;stable_since=None
        while time.monotonic()<stop_until:
            rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
            fresh=now-state['odom_wall']<.5
            stopped=(fresh and state.get('linear_speed',float('inf'))<=limits['settle_linear_mps']
                     and state.get('angular_speed',float('inf'))<=limits['settle_angular_rps'])
            stable_since=(now if stable_since is None else stable_since) if stopped else None
            if stable_since is not None and now-stable_since>=limits['settle_s']:break
        if stable_since is not None and time.monotonic()-stable_since>=limits['settle_s']:
            from serving_robot_mapping.mapping_camera_evidence import collect_stopped_cameras
            try:
                camera=collect_stopped_cameras(node,output)
                camera['physical_stop_observed']=True
                camera['nav2_shutdown_status_observed']=result.get('nav2_shutdown_status_observed')
                (output/'stopped_camera_evidence/camera_evidence.json').write_text(json.dumps(camera,indent=2))
            except Exception as exc:
                (output/'camera_diagnostic_failure.json').write_text(json.dumps(dict(error=str(exc),map_or_motion_modified=False)))
        else:
            (output/'camera_diagnostic_skipped.json').write_text(json.dumps(dict(reason='PHYSICAL_STOP_NOT_CONFIRMED',motion_command_sent=False)))
except Exception as exc:error=str(exc);print(error,flush=True)
finally:
    if child and child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=65)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)
    source=output/'R16-147_session_result.json';team_result=json.loads(source.read_text()) if source.exists() else None
    manifest={'role':'mapping','stack':'mapping-only patched SLAM + TEAM Nav2/exploration' if config.get('mapping_slam_overlay') else 'TEAM mapping/exploration','same_robot_model':'serving_robot_description/urdf/serving_robot.urdf.xacro','robot':'robot_01','owner':entry,'error':error,'runner_returncode':returncode,'startup':before,'map_updates':state['map_updates'],'max_odom_displacement_m':state['max_odom_displacement_m'],'team_session_report':str(source) if source.exists() else None,'map_complete':bool(team_result and team_result.get('map_complete')),'raw_map_saved':bool(team_result and team_result.get('raw_map_saved')),'operation_map_changed':map_digest()!=original_map,'operation_map_sha256':original_map}
    (output/'personal_mapping_role.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(manifest,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
raise SystemExit(1 if error else returncode if returncode is not None else 1)
