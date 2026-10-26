#!/usr/bin/env python3
"""One-shot V4 activation (LAB only; TEAM files are never modified).

 1. runs the offline tests (aborts on any failure)
 2. refuses unless the robot is stopped and Nav2 has no active goal
 3. verifies existing Nav2 params WITHOUT writing any parameters or YAML
 4. restarts ONLY /destination_server as destination_server_lab_v4.py with arrival_v4.yaml
    (rolls back to the previous command if the replacement does not come up)
 Options: --check-only (offline tests + live read-only checks; changes nothing)
          No YAML replacement or dynamic Nav2 parameter setting is supported.
Gazebo / AMCL / Nav2 / Safety / Supervisor are not restarted. No goal is sent.
"""
import os, signal, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HOME = Path.home()
TEAM_SCRIPTS = HOME / 'team_serving_robot/ros_ws/src/serving_robot_navigation/scripts'
NAMES = {'destination_server.py', 'destination_server_lab.py', 'destination_server_lab_v4.py'}
# Runtime Nav2 values confirmed on 2026-10-20. Verify; NEVER change.
EXPECTED_NAV2 = {
    'progress_checker.plugin': 'nav2_controller::PoseProgressChecker',
    'progress_checker.required_movement_radius': '0.03',
    'progress_checker.movement_time_allowance': '10.0',
    'general_goal_checker.xy_goal_tolerance': '0.0025',
    'general_goal_checker.yaw_goal_tolerance': '0.25',
    'general_goal_checker.stateful': 'false',
}

def check_nav2_readonly():
    for name, expected in EXPECTED_NAV2.items():
        result = subprocess.run(['ros2', 'param', 'get', '/robot_01/controller_server', name],
                                capture_output=True, text=True, timeout=15, check=True)
        fields = result.stdout.strip().split('is:', 1)
        if len(fields) != 2:
            raise RuntimeError('cannot read Nav2 parameter: ' + name)
        actual = fields[1].strip()
        same = actual.lower() == expected.lower()
        if not same:
            try:
                same = float(actual) == float(expected)
            except ValueError:
                pass
        if not same:
            raise RuntimeError(f'Nav2 mismatch (not changed): {name} = {actual}, expected {expected}')
        print('NAV2 UNCHANGED ' + name + ' = ' + actual, flush=True)

def args_of(pid):
    try: return [v.decode() for v in (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0') if v]
    except OSError: return []
def find_servers():
    return [(int(p.name), a) for p in Path('/proc').iterdir() if p.name.isdecimal()
            for a in [args_of(p.name)] if a and any(Path(x).name in NAMES for x in a) and 'python' in Path(a[0]).name]
def getparam(name):
    r = subprocess.run(['ros2', 'param', 'get', '/destination_server', name], capture_output=True, text=True, timeout=15, check=True)
    v = [l.partition('String value is:')[2].strip() for l in r.stdout.splitlines() if 'String value is:' in l]
    if len(v) != 1: raise RuntimeError('unreadable parameter ' + name)
    return v[0]
def topic_nodes(topic):
    r = subprocess.run(['ros2', 'topic', 'info', topic, '-v'], capture_output=True, text=True, timeout=20)
    pubs, subs, mode = set(), set(), None
    for line in r.stdout.splitlines():
        if line.startswith('Publisher count'): mode = pubs
        elif line.startswith('Subscription count'): mode = subs
        elif line.strip().startswith('Node name:') and mode is not None: mode.add(line.split(':', 1)[1].strip())
    return pubs, subs
def check_velocity_chain():
    # V4 publishes to the SAME topic Nav2's controller uses, so it gets the same safety path.
    p, s = topic_nodes('/robot_01/cmd_vel_nav')
    want = [('/robot_01/cmd_vel_nav', 'controller_server', p, 'publisher'), ('/robot_01/cmd_vel_nav', 'velocity_smoother', s, 'subscriber'),
            ('/robot_01/cmd_vel_smoothed', 'semantic_safety', topic_nodes('/robot_01/cmd_vel_smoothed')[1], 'subscriber'),
            ('/robot_01/cmd_vel_semantic', 'collision_monitor', topic_nodes('/robot_01/cmd_vel_semantic')[1], 'subscriber')]
    bad = [f'{t}: no {role} {n} (found {sorted(found)})' for t, n, found, role in want if n not in found]
    if bad: raise RuntimeError('velocity/safety chain not as expected, refusing:\n  ' + '\n  '.join(bad))
    print('CHAIN OK: controller -> cmd_vel_nav -> velocity_smoother -> semantic_safety -> collision_monitor', flush=True)
def guard_robot():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    rclpy.init(); n = Node('v4_activation_guard'); odom = []; st = []
    n.create_subscription(Odometry, '/robot_01/odom', odom.append, 10)
    q = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(GoalStatusArray, '/robot_01/navigate_to_pose/_action/status', st.append, q)
    end = time.monotonic() + 8
    while time.monotonic() < end and (not odom or not st): rclpy.spin_once(n, timeout_sec=.1)
    n.destroy_node(); rclpy.shutdown()
    if not odom: raise RuntimeError('no live odometry')
    t = odom[-1].twist.twist
    if max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)) > .01: raise RuntimeError('robot is moving')
    if not st: raise RuntimeError('Nav2 action status unavailable')
    if [s.status for s in st[-1].status_list if s.status in (1, 2, 3)]: raise RuntimeError('Nav2 has an active goal')
    print('SAFETY OK: robot stopped, no active Nav2 goal', flush=True)
def spawn(args, log):
    env = dict(os.environ); env['PYTHONPATH'] = os.pathsep.join([str(ROOT), str(TEAM_SCRIPTS), env.get('PYTHONPATH', '')])
    with open(log, 'a') as fd:
        return subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=fd, stderr=subprocess.STDOUT, cwd=str(ROOT), env=env, start_new_session=True)

def main():
    if any(arg != '--check-only' for arg in sys.argv[1:]):
        raise RuntimeError('Unsupported flag; Nav2 YAML/param edits are prohibited')
    if os.environ.get('ROS_DOMAIN_ID') != '99': raise RuntimeError('ROS_DOMAIN_ID must be 99')
    for need in (TEAM_SCRIPTS / 'arrival_policy.py', TEAM_SCRIPTS / 'navigate_destination.py'):
        if not need.is_file(): raise RuntimeError('missing ' + str(need))
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT), str(TEAM_SCRIPTS)]))
    for t in ('test_fine_approach.py', 'test_v4_server.py', 'test_v3_regression_on_v4.py', 'test_near_goal_handoff.py'):
        r = subprocess.run([sys.executable, str(ROOT / t)], cwd=str(ROOT), capture_output=True, text=True, env=env)
        print(('OFFLINE PASS ' if r.returncode == 0 else 'OFFLINE FAIL ') + t)
        if r.returncode: print(r.stdout[-1500:], r.stderr[-800:]); raise RuntimeError('offline test failed: refusing to deploy')
    cur = find_servers()
    if len(cur) != 1: raise RuntimeError('expected exactly one destination server, found %d' % len(cur))
    check_velocity_chain()
    check_nav2_readonly()
    guard_robot()
    if '--check-only' in sys.argv:
        print('CHECK-ONLY PASSED: offline tests, velocity chain and robot-stopped guard are OK. Nothing was changed.'); return
    pid, prev = cur[0]; waypoints = getparam('waypoints_file')
    if getparam('robot_id') != 'robot_01': raise RuntimeError('wrong robot')
    # Existing runtime Nav2 configuration was verified read-only above.
    # No param set, YAML overwrite, Gazebo, Safety, or Nav2 restart.
    if subprocess.run(['kill', '-INT', str(pid)]).returncode: raise RuntimeError('could not stop old destination server')
    for _ in range(80):
        if not Path('/proc', str(pid)).exists(): break
        time.sleep(.1)
    else: raise RuntimeError('old destination server did not exit; nothing launched')
    new = [sys.executable, '-u', str(ROOT / 'destination_server_lab_v4.py'), '--ros-args', '-p', 'robot_id:=robot_01',
           '-p', 'waypoints_file:=' + waypoints, '-p', 'arrival_file:=' + str(ROOT / 'arrival_v4.yaml'), '-p', 'use_sim_time:=true']
    proc = spawn(new, '/tmp/robot01_destination_v4.log')
    try:
        for _ in range(15):
            if proc.poll() is not None: break
            try:
                if 'arrival_v4.yaml' in getparam('arrival_file'): print('V4 ACTIVATED, destination_server PID', proc.pid, '(log /tmp/robot01_destination_v4.log)'); return
            except Exception: pass
            time.sleep(1)
        raise RuntimeError('V4 destination server did not come up; see /tmp/robot01_destination_v4.log')
    except Exception:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=6)
            except subprocess.TimeoutExpired: pass
        if not find_servers(): print('ROLLBACK: restarted previous server, PID', spawn(prev, '/tmp/robot01_destination_rollback.log').pid)
        raise

if __name__ == '__main__':
    main()
