#!/usr/bin/env python3
"""ROS LAB serving manager v3: map-aware table approach selection and handover gates.
Does not modify ROS Nav2 or existing destination_server. Not Fleet task authority.
"""
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import yaml

ROOT=Path(__file__).resolve().parent
CFG=ROOT/'layout.yaml'; SNAP=ROOT/'generated/manifest.json'; WPS=ROOT/'generated/active_waypoints.yaml'

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def json_msg(obj):return String(data=json.dumps(obj,ensure_ascii=False))

class Supervisor(Node):
    def __init__(self):
        super().__init__('serving_supervisor_lab_v3')
        self.conf=yaml.safe_load(CFG.read_text());self.snap=json.loads(SNAP.read_text())
        self.robot=self.conf['robot_id'];self.revision=self.conf['map_revision']
        self.wps={v['entity_id']:v['pose'] for v in yaml.safe_load(WPS.read_text())['waypoints']}
        self.options={k:{v['side']:v for v in opts} for k,opts in self.snap['choices'].items()}
        self.serving=self.conf['serving'];self.mode=self.serving['completion_mode']
        self.loading_mode=self.serving['loading_mode']
        self.state='IDLE';self.table=None;self.side=None;self.target=None;self.reason='READY'
        self.last_state=None;self.active_request=None;self.wait_since=None
        self.map_received=False;self.map_valid=False;self.ready_error=None
        self.planning=None;self.plan_results=[];self.plan_pending=[];self.plan_started=None
        self.expected_revision=self.revision
        self.pub=self.create_publisher(String,f'/{self.robot}/destination_command',10)
        self.status_pub=self.create_publisher(String,f'/{self.robot}/serving/status',10)
        self.sub_req=self.create_subscription(String,f'/{self.robot}/serving/request',self.on_request,10)
        self.sub_complete=self.create_subscription(String,f'/{self.robot}/serving/complete',self.on_complete,10)
        self.sub_robot=self.create_subscription(String,f'/{self.robot}/robot_state',self.on_robot_state,10)
        self.sub_nav_event=self.create_subscription(String,f'/{self.robot}/navigation_event',self.on_nav_event,10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map=self.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        self.tf=Buffer();self.tfl=TransformListener(self.tf,self)
        self.planner=ActionClient(self,ComputePathToPose,f'/{self.robot}/compute_path_to_pose')
        self.create_timer(.2,self.tick)
        self.create_timer(1.,self.status)
        self.status()
    def status(self, reason=None):
        self.status_pub.publish(json_msg({'robot_id':self.robot,'state':self.state,'table':self.table,
           'side':self.side,'destination_id':self.target,'reason':reason or self.reason,
           'map_revision':self.revision,'map_valid':self.map_valid,
           'completion_mode':self.mode,'loading_mode':self.loading_mode}))
    def set_state(self,state,reason):
        self.state=state;self.reason=reason;self.status()
    def same_snapshot(self):
        try:
            if self.conf['map_revision']!=self.snap['map_revision']:return False
            if digest(CFG)!=self.snap['layout_sha256']:return False
            if digest(WPS)!=self.snap['waypoints_sha256']:return False
            for pathkey,digestkey in [('world','world_sha256'),('map','map_yaml_sha256'),('map_raster','map_raster_sha256')]:
                if digest(Path(self.snap[pathkey]))!=self.snap[digestkey]:return False
            return True
        except Exception:return False
    def on_map(self,msg):
        info=self.snap['map_info'];m=msg.info
        self.map_received=True
        meta_ok=(m.width==info['width'] and m.height==info['height']
           and abs(m.resolution-info['resolution'])<1e-5
           and abs(m.origin.position.x-info['origin'][0])<1e-4
           and abs(m.origin.position.y-info['origin'][1])<1e-4)
        if meta_ok:
            # Exact binary occupancy match to the active SDF-derived LAB map.
            # No silent success for a changed map that happens to share size/origin.
            binary=bytearray(1 if v>=65 else 0 if 0<=v<=20 else 2 for v in msg.data)
            self.map_valid=(hashlib.sha256(binary).hexdigest()==self.snap['occupancy_binary_sha256'])
        else:self.map_valid=False
        if not self.map_valid and self.state not in ('IDLE','COMPLETED','CANCELED','ERROR'):
            self.emergency('ACTIVE_MAP_METADATA_CHANGED')
    def ready(self):
        if not self.map_received:return 'MAP_NOT_RECEIVED'
        if not self.map_valid:return 'MAP_METADATA_MISMATCH'
        if not self.same_snapshot():return 'SNAPSHOT_CHANGED_REPREPARE_AND_REACTIVATE'
        if self.pub.get_subscription_count()==0:return 'DESTINATION_SERVER_NOT_CONNECTED'
        return None
    def go(self,target):
        problem=self.ready()
        if problem:self.set_state('ERROR',problem);return
        if target not in self.wps:self.set_state('ERROR','UNKNOWN_WAYPOINT:'+str(target));return
        self.target=target
        self.go_since=time.monotonic()
        self.pub.publish(json_msg({'command':'GO','destination_id':target,'map_revision':self.revision}))
    def emergency(self,reason):
        if self.state in ('IDLE','ERROR','CANCELED','COMPLETED'):return
        if self.pub.get_subscription_count()>0:self.pub.publish(json_msg({'command':'CANCEL'}))
        self.planning=None;self.set_state('ERROR',reason)
    def on_request(self,msg):
        try:cmd=json.loads(msg.data)
        except Exception:self.status('REJECT_BAD_JSON');return
        if cmd.get('command')=='CANCEL':
            if self.state not in ('IDLE','COMPLETED','CANCELED','ERROR'):
                self.pub.publish(json_msg({'command':'CANCEL'}))
            self.planning=None;self.set_state('CANCELED','CANCEL_REQUESTED');return
        if cmd.get('command')!='START':self.status('REJECT_UNKNOWN_COMMAND');return
        if self.state not in ('IDLE','COMPLETED','CANCELED','ERROR'):
            self.status('REJECT_BUSY');return
        problem=self.ready()
        if problem:self.status('REJECT_'+problem);return
        table=cmd.get('table');side=cmd.get('side','auto')
        if table not in self.options:self.status('REJECT_UNKNOWN_TABLE');return
        if side!='auto' and (side not in self.options[table] or not self.options[table][side]['enabled']):
            self.status('REJECT_APPROACH_INACTIVE_OR_UNKNOWN');return
        self.table=table;self.side=side;self.target=None
        self.set_state('GOING_PICKUP','GO_COUNTER_PICKUP');self.go('counter_pickup')
    def on_complete(self,msg):
        try:cmd=json.loads(msg.data)
        except Exception:self.status('REJECT_BAD_JSON');return
        action=cmd.get('command')
        if action=='CONFIRM_LOAD' and self.state=='WAIT_LOAD':self.choose_table();return
        if action=='COMPLETE' and self.state=='WAIT_HANDOVER':
            self.set_state('RETURNING_PICKUP','RETURN_TO_COUNTER_PICKUP');self.go('counter_pickup');return
        self.status('REJECT_UNEXPECTED_COMPLETION')
    def on_robot_state(self,msg):
        try:o=json.loads(msg.data)
        except Exception:return
        if o.get('robot_id')!=self.robot or o.get('destination_id')!=self.target:return
        if o.get('state') in ('ERROR','PAUSED') and self.state in ('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP'):
            self.emergency('NAV_'+o['state']);return
        if o.get('state')!='SERVING':return
        # SERVING comes exclusively from old destination_server's Nav2 + precise pose gate.
        if self.state=='GOING_PICKUP':
            self.wait_since=time.monotonic();self.set_state('WAIT_LOAD','WAIT_FOR_KITCHEN_LOAD_CONFIRM')
        elif self.state=='GOING_TABLE':
            self.wait_since=time.monotonic();self.set_state('WAIT_HANDOVER','WAIT_FOR_HANDOVER_CONFIRM')
        elif self.state=='RETURNING_PICKUP':self.set_state('COMPLETED','ROUND_TRIP_COMPLETED')
    def on_nav_event(self,msg):
        try:obj=json.loads(msg.data)
        except Exception:return
        if obj.get('robot_id')!=self.robot:return
        if obj.get('event') in ('REJECTED','NAV_GOAL_FAILED','RESULT_ERROR','NAV_GOAL_TERMINATED','FINAL_ACCURACY_NOT_REACHED') and self.state in ('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP'):
            self.emergency('DESTINATION_'+obj['event']+':'+str(obj.get('reason','')))
    def make_pose(self,p):
        msg=PoseStamped();msg.header.frame_id='map'
        msg.pose.position.x=float(p['x']);msg.pose.position.y=float(p['y'])
        yaw=float(p['yaw']);msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
        return msg
    def choose_table(self):
        problem=self.ready()
        if problem:self.set_state('ERROR',problem);return
        candidates=[v for v in self.options[self.table].values() if v['enabled']]
        if self.side!='auto':candidates=[v for v in candidates if v['side']==self.side]
        if not candidates:self.set_state('ERROR','NO_ACTIVE_APPROACH');return
        # Explicit approach selection still gets a path feasibility check.
        if not self.planner.server_is_ready():self.set_state('ERROR','PLANNER_NOT_READY');return
        try:
            tf=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time())
        except Exception as e:self.set_state('ERROR','ROBOT_POSE_NOT_AVAILABLE:'+str(e));return
        q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        start={'x':tf.transform.translation.x,'y':tf.transform.translation.y,'yaw':yaw}
        self.planning={'queue':list(candidates),'results':[], 'started':time.monotonic()}
        self.set_state('SELECTING_APPROACH','PLANNING_ACTIVE_WAYPOINTS')
        self.plan_next(self.planning)
    def plan_next(self,token):
        if self.planning is not token:return
        if not token['queue']:
            self.planning=None
            if not token['results']:self.set_state('ERROR','NO_REACHABLE_TABLE_APPROACH');return
            _,best=min(token['results'],key=lambda v:v[0]);self.side=best['side']
            self.set_state('GOING_TABLE','GO_SELECTED_APPROACH_'+self.side)
            self.go(best['id'])
            return
        item=token['queue'].pop(0)
        try:
            tf=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time())
            q=tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            start={'x':tf.transform.translation.x,'y':tf.transform.translation.y,'yaw':yaw}
            request=ComputePathToPose.Goal()
            request.start=self.make_pose(start);request.goal=self.make_pose(item['pose']);request.use_start=True
            fut=self.planner.send_goal_async(request)
            fut.add_done_callback(lambda f, it=item, tok=token:self.plan_accept(f,it,tok))
        except Exception:self.plan_done(token,item,None)
    def plan_accept(self,fut,item,token):
        if self.planning is not token:return
        try:
            handle=fut.result()
            if not handle.accepted:self.plan_done(token,item,None);return
            f=handle.get_result_async();f.add_done_callback(lambda v:self.plan_result(v,item,token))
        except Exception:self.plan_done(token,item,None)
    def plan_result(self,fut,item,token):
        if self.planning is not token:return
        try:
            result=fut.result();pts=result.result.path.poses
            if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0 or not pts:
                self.plan_done(token,item,None);return
            dist=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
                     for a,b in zip(pts,pts[1:]))
            self.plan_done(token,item,dist)
        except Exception:self.plan_done(token,item,None)
    def plan_done(self,token,item,distance):
        if self.planning is not token:return
        if distance is not None:token['results'].append((distance,item))
        self.plan_next(token)
    def tick(self):
        if self.state in ('IDLE','CANCELED','ERROR','COMPLETED'):return
        if not self.same_snapshot():self.emergency('SNAPSHOT_CHANGED_DURING_TASK');return
        if self.map_received and not self.map_valid:self.emergency('MAP_CHANGED_DURING_TASK');return
        if self.state in ('GOING_PICKUP','GOING_TABLE','RETURNING_PICKUP') and hasattr(self,'go_since'):
            if time.monotonic()-self.go_since>self.serving['active_goal_timeout_s']:
                self.emergency('SERVING_GOAL_TIMEOUT');return
        if self.state=='SELECTING_APPROACH' and self.planning and time.monotonic()-self.planning['started']>self.serving['planner_timeout_s']:
            self.planning=None;self.set_state('ERROR','PLANNER_TIMED_OUT');return
        if self.state=='WAIT_LOAD' and self.loading_mode=='auto' and time.monotonic()-self.wait_since>=self.serving['auto_loading_wait_s']:
            self.choose_table()
        if self.state=='WAIT_HANDOVER' and self.mode=='auto' and time.monotonic()-self.wait_since>=self.serving['auto_handover_wait_s']:
            self.set_state('RETURNING_PICKUP','LAB_AUTO_HANDOVER');self.go('counter_pickup')


def main():
    rclpy.init();n=None
    try:n=Supervisor();rclpy.spin(n)
    finally:
        if n:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
