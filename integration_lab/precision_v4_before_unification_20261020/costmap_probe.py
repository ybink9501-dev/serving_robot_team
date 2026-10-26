#!/usr/bin/env python3
"""Why does the global costmap block Table 01?  Reads /map and the global costmap at the
robot, Table 01 Top/Bottom and Kitchen, reports the distance to the nearest LETHAL cell in
each, and (with --experiment) disables the obstacle layer, clears the costmap, re-reads, and
ALWAYS re-enables it.  Costmap value 99 = INSCRIBED, 100 = LETHAL (not an inflation gradient).

  python3 costmap_probe.py            # read only
  python3 costmap_probe.py --experiment
Needs: ROS_DOMAIN_ID=99 and the ROS environment sourced.
"""
import math, subprocess, sys, time

POINTS = {'robot': None, 'table01_top': (-1.60, 1.60), 'table01_bottom': (-1.60, -0.30), 'kitchen': (-2.30, 1.45)}
G = '/robot_01/global_costmap/global_costmap'
CLEAR = '/robot_01/global_costmap/clear_entirely_global_costmap'

def cell(grid, x, y):
    i = int(math.floor((x - grid['ox']) / grid['res'])); j = int(math.floor((y - grid['oy']) / grid['res']))
    if not (0 <= i < grid['w'] and 0 <= j < grid['h']): return None
    return grid['data'][j * grid['w'] + i]

def nearest_lethal(grid, x, y, radius=1.0, lethal=100):
    best = None; r = int(radius / grid['res'])
    ci = int(math.floor((x - grid['ox']) / grid['res'])); cj = int(math.floor((y - grid['oy']) / grid['res']))
    for j in range(max(0, cj - r), min(grid['h'], cj + r + 1)):
        for i in range(max(0, ci - r), min(grid['w'], ci + r + 1)):
            if grid['data'][j * grid['w'] + i] >= lethal:
                d = math.hypot((i + .5) * grid['res'] + grid['ox'] - x, (j + .5) * grid['res'] + grid['oy'] - y)
                if best is None or d < best: best = d
    return best

def label(v):
    if v is None: return 'outside'
    return {-1: 'UNKNOWN', 0: 'free', 99: 'INSCRIBED(<=0.20m from obstacle)', 100: 'LETHAL'}.get(v, 'inflation gradient')

def verdict(name, cm, mp):
    c, m = cell(cm, *name[1]), cell(mp, *name[1])
    dc, dm = nearest_lethal(cm, *name[1]), nearest_lethal(mp, *name[1])
    return c, m, dc, dm

def snapshot(rclpy, node, qos, OccupancyGrid, Buf):
    got = {}
    def mk(key):
        def cb(msg): got[key] = dict(w=msg.info.width, h=msg.info.height, res=msg.info.resolution,
                                    ox=msg.info.origin.position.x, oy=msg.info.origin.position.y, data=list(msg.data))
        return cb
    node.create_subscription(OccupancyGrid, '/map', mk('map'), qos)
    node.create_subscription(OccupancyGrid, '/robot_01/global_costmap/costmap', mk('cost'), qos)
    end = time.time() + 12
    while time.time() < end and len(got) < 2: rclpy.spin_once(node, timeout_sec=.1)
    if len(got) < 2: raise RuntimeError('did not receive /map and the global costmap: ' + str(list(got)))
    pose = None
    try:
        tf = Buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()); pose = (tf.transform.translation.x, tf.transform.translation.y)
    except Exception as e: print('robot TF unavailable:', e)
    return got['map'], got['cost'], pose

def report(tag, mp, cm, pose):
    print('\n== %s ==' % tag)
    pts = dict(POINTS); pts['robot'] = pose
    for n, p in pts.items():
        if p is None: print('%-15s (no pose)' % n); continue
        c, m, dc, dm = verdict((n, p), cm, mp)
        print('%-15s (%.3f,%.3f) map=%-4s costmap=%-4s [%s]  nearest lethal: costmap=%s  /map=%s' % (
            n, p[0], p[1], m, c, label(c), 'none<1m' if dc is None else '%.2fm' % dc, 'none<1m' if dm is None else '%.2fm' % dm))
    print('Reading: lethal in costmap but NOT in /map near a point -> Obstacle Layer marks (LiDAR/frame/AMCL).')
    print('         lethal in both -> the static map really has an obstacle there (geometry or map/world mismatch).')

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); node = Node('costmap_probe')
    buf = Buffer(); TransformListener(buf, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    try:
        for _ in range(20): rclpy.spin_once(node, timeout_sec=.1)
        mp, cm, pose = snapshot(rclpy, node, qos, OccupancyGrid, buf)
        report('as running (obstacle layer ON)', mp, cm, pose)
        if '--experiment' in sys.argv:
            try:
                subprocess.run(['ros2', 'param', 'set', G, 'obstacle_layer.enabled', 'false'], check=True, timeout=15)
                subprocess.run(['ros2', 'service', 'call', CLEAR, 'nav2_msgs/srv/ClearEntireCostmap', '{}'], check=True, timeout=15)
                time.sleep(4)
                node2 = Node('costmap_probe2'); mp2, cm2, pose2 = snapshot(rclpy, node2, qos, OccupancyGrid, buf)
                report('obstacle layer OFF (static map + inflation only)', mp2, cm2, pose2 or pose)
            finally:
                restored = False
                for _ in range(4):
                    try:
                        subprocess.run(['ros2', 'param', 'set', G, 'obstacle_layer.enabled', 'true'], timeout=15, capture_output=True)
                        got = subprocess.run(['ros2', 'param', 'get', G, 'obstacle_layer.enabled'], timeout=15, capture_output=True, text=True).stdout
                        if 'True' in got: restored = True; break
                    except Exception: pass
                    time.sleep(1)
                try: subprocess.run(['ros2', 'service', 'call', CLEAR, 'nav2_msgs/srv/ClearEntireCostmap', '{}'], timeout=15, capture_output=True)
                except Exception: pass
                if restored: print('\nVERIFIED: obstacle layer re-enabled and costmap cleared.')
                else:
                    print('\n!!! obstacle_layer.enabled COULD NOT BE CONFIRMED TRUE. Run now:\n    ros2 param set %s obstacle_layer.enabled true' % G)
                    sys.exit(2)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
