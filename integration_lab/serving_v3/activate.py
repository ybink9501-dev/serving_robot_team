#!/usr/bin/env python3
"""Opt-in LAB activation. Only destination_server is restarted; all other ROS nodes stay up.
Ensures robot is stopped and no Nav2 goal is currently executing before replacing YAML.
"""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import yaml
ROOT=Path(__file__).resolve().parent
SNAP=ROOT/'generated/manifest.json'
DEST_SCRIPT=Path.home()/'team_serving_robot/ros_ws/src/serving_robot_navigation/scripts/destination_server.py'
ARRIVAL=Path.home()/'team_serving_robot/ros_ws/src/serving_robot_navigation/config/arrival.yaml'

def proc_args(pid):
    try:return [v.decode() for v in (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\x00') if v]
    except (FileNotFoundError,PermissionError,ProcessLookupError):return []
def find_exact(script):
    return [(int(p.name),v) for p in Path('/proc').iterdir() if p.name.isdecimal()
            if (v:=proc_args(int(p.name))) and str(script) in v]
def assert_file(path):
    if not path.is_file():raise RuntimeError('FILE MISSING: '+str(path))
def getparam(name):
    result=subprocess.run(['ros2','param','get','/destination_server',name],capture_output=True,text=True,timeout=15,check=True)
    vals=[v.partition('String value is:')[2].strip() for v in result.stdout.splitlines() if 'String value is:' in v]
    if len(vals)!=1:raise RuntimeError('ROS parameter unreadable: '+name)
    return vals[0]
def guard_robot():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    rclpy.init();node=Node('serving_v3_activation_safety_check')
    odom=[];status=[]
    node.create_subscription(Odometry,'/robot_01/odom',lambda m:odom.append(m),10)
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(GoalStatusArray,'/robot_01/navigate_to_pose/_action/status',lambda m:status.append(m),qos)
    until=time.monotonic()+8
    while time.monotonic()<until and (not odom or not status):rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node();rclpy.shutdown()
    if not odom:raise RuntimeError('No live odometry: refuse to restart destination server')
    t=odom[-1].twist.twist
    if abs(t.linear.x)>.01 or abs(t.linear.y)>.01 or abs(t.angular.z)>.01:
        raise RuntimeError('Robot is moving: refuse deployment')
    if not status:raise RuntimeError('Nav2 action status unavailable: refuse deployment')
    executing=[s.status for s in status[-1].status_list if s.status in (1,2,3)]
    if executing:raise RuntimeError('Nav2 has active or canceling goal(s): '+str(executing))
    print('SAFETY: robot stopped; no active Nav2 action goals',flush=True)
def spawn(args,log):
    log.parent.mkdir(parents=True,exist_ok=True)
    fd=open(log,'a',encoding='utf8')
    try:return subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=fd,stderr=subprocess.STDOUT,
                                cwd=str(ROOT),start_new_session=True)
    finally:fd.close()
def wait_exit(pid,seconds=8):
    for _ in range(int(seconds*10)):
        if not Path('/proc',str(pid)).exists():return True
        time.sleep(.1)
    return False

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='99':raise RuntimeError('ROS_DOMAIN_ID must be 99')
    assert_file(SNAP);assert_file(DEST_SCRIPT);assert_file(ARRIVAL)
    assert_file(ROOT/'generated/active_waypoints.yaml')
    c=yaml.safe_load((ROOT/'layout.yaml').read_text());m=json.loads(SNAP.read_text())
    if m['map_revision']!=c['map_revision']:raise RuntimeError('snapshot map revision mismatch')
    import hashlib
    if hashlib.sha256((ROOT/'layout.yaml').read_bytes()).hexdigest()!=m['layout_sha256']:
        raise RuntimeError('layout edited since prepare: run prepare.py again')
    if hashlib.sha256((ROOT/'generated/active_waypoints.yaml').read_bytes()).hexdigest()!=m['waypoints_sha256']:
        raise RuntimeError('waypoint snapshot edited: run prepare.py again')
    # Don't replace a running V3 manager or another server using the same ROS topic.
    managers=find_exact(ROOT/'supervisor_v3.py')
    if managers:raise RuntimeError('V3 supervisor already running; do not start a duplicate')
    current=find_exact(DEST_SCRIPT)
    if len(current)!=1:raise RuntimeError(f'Expected exactly one active destination server; got {len(current)}')
    guard_robot()
    pid,previous_args=current[0];old_path=getparam('waypoints_file')
    if getparam('robot_id')!='robot_01':raise RuntimeError('Wrong robot destination server')
    print('Old waypoint:',old_path, 'old PID:',pid,flush=True)
    goalfile=str(ROOT/'generated/active_waypoints.yaml')
    old=subprocess.run(['kill','-INT',str(pid)],capture_output=True,text=True)
    if old.returncode!=0:raise RuntimeError('Could not stop existing destination_server')
    if not wait_exit(pid):raise RuntimeError('Destination server did not exit; no second server launched')
    args=[sys.executable,'-u',str(DEST_SCRIPT),'--ros-args',
           '-p','robot_id:=robot_01','-p',f'waypoints_file:={goalfile}',
           '-p',f'arrival_file:={ARRIVAL}','-p','use_sim_time:=true']
    replacement=spawn(args,Path('/tmp/robot01_destination_v3.log'))
    try:
        success=False
        for _ in range(12):
            if replacement.poll() is not None:break
            try:
                if getparam('waypoints_file')==goalfile:success=True;break
            except Exception:pass
            time.sleep(1)
        if not success:raise RuntimeError('Replacement destination server failed to load V3 semantic YAML')
        manager=spawn([sys.executable,'-u',str(ROOT/'supervisor_v3.py')],
                      Path('/tmp/robot01_supervisor_v3.log'))
        time.sleep(2)
        if manager.poll() is not None:raise RuntimeError('V3 supervisor startup failed; check /tmp/robot01_supervisor_v3.log')
        print('V3 ACTIVATED, destination PID',replacement.pid,'supervisor PID',manager.pid,flush=True)
        print('Supervisor IDLE: no GO was sent. To move, use serving_control_v3.py start ...',flush=True)
    except Exception:
        # On failed activation, restore original command where feasible.
        if replacement.poll() is None:
            replacement.send_signal(signal.SIGINT)
            try:replacement.wait(timeout=6)
            except subprocess.TimeoutExpired:pass
        if previous_args and not find_exact(DEST_SCRIPT):
            restored=spawn(previous_args,Path('/tmp/robot01_destination_v3_rollback.log'))
            print('Rollback: attempted previous destination_server, PID',restored.pid,flush=True)
        raise

if __name__=='__main__':
    try:main()
    except Exception as e:raise SystemExit('V3 ACTIVATION ABORTED: '+str(e))
