#!/usr/bin/env python3
"""Diagnostic-only Nav2 costmap/cloud recorder for a single existing Gazebo trial."""
import argparse, gzip, hashlib, json, time
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

parser=argparse.ArgumentParser()
parser.add_argument('--robot',default='robot_01')
parser.add_argument('--output',required=True)
args=parser.parse_args()
rclpy.init()
node=Node('detour_costmap_probe')
out=gzip.open(args.output,'wt')
prefix='/'+args.robot+'/'
counts={'global':0,'local':0,'cloud':0}
latest={}
def record(kind,payload):
    row={'wall':time.time(),'kind':kind,**payload}
    out.write(json.dumps(row,separators=(',',':'))+'\n')
    out.flush()
def cost_callback(which):
    def cb(msg):
        counts[which]+=1
        raw=bytes(msg.data)
        meta=msg.metadata
        info={'stamp':[msg.header.stamp.sec,msg.header.stamp.nanosec],
              'frame':msg.header.frame_id,'layer':meta.layer,
              'resolution':meta.resolution,'size':[meta.size_x,meta.size_y],
              'origin':[meta.origin.position.x,meta.origin.position.y],
              'sha256':hashlib.sha256(raw).hexdigest(),
              'cells':list(raw)}
        latest[which]={k:v for k,v in info.items() if k!='cells'}
        record(which,info)
    return cb
def cloud(msg):
    counts['cloud']+=1
    pts=[]
    try:
        pts=[[float(p[0]),float(p[1]),float(p[2])]
             for p in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)]
    except Exception as exc:
        record('cloud_error',{'error':str(exc)})
    record('cloud',{'stamp':[msg.header.stamp.sec,msg.header.stamp.nanosec],
                    'frame':msg.header.frame_id,'points':pts,
                    'subscriber_count':node.count_subscribers(prefix+'dynamic_obstacle_cloud')})
def event(kind):
    def cb(msg):
        try: data=json.loads(msg.data)
        except Exception: data={'raw':msg.data}
        if kind=='dynamic_safety' and not data.get('events'): return
        record(kind,{'data':data,'costmaps':latest.copy(),'counts':counts.copy()})
    return cb
subs=[
 node.create_subscription(Costmap,prefix+'global_costmap/costmap_raw',cost_callback('global'),10),
 node.create_subscription(Costmap,prefix+'local_costmap/costmap_raw',cost_callback('local'),10),
 node.create_subscription(PointCloud2,prefix+'dynamic_obstacle_cloud',cloud,10),
 node.create_subscription(String,prefix+'dynamic_event',event('dynamic_event'),20),
 node.create_subscription(String,prefix+'detour_state',event('detour_state'),20),
 node.create_subscription(String,prefix+'dynamic_safety',event('dynamic_safety'),10)]
print('probe ready',args.output,flush=True)
try:
    while rclpy.ok():rclpy.spin_once(node,timeout_sec=0.5)
except KeyboardInterrupt:pass
finally:
    record('summary',{'counts':counts,'latest':latest})
    out.close();node.destroy_node();rclpy.shutdown()
