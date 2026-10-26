"""Serving mission and constrained fine correction. Never treats Nav2 ABORT as successful arrival."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from .policy import (ACTIVE, norm_angle, distance, velocity_limit,
                     GOAL_NAV_XY_M, ARRIVAL_DETECT_M, FINAL_PHYSICAL_M, next_after_nav)
from .waypoints import read_waypoints, choose_waypoint
from .runtime_config import load_limits


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Mission(Node):
    def __init__(self):
        super().__init__('serving_mission')
        self.declare_parameter('robot_id','robot_01')
        self.declare_parameter('waypoints_file','')
        self.declare_parameter('fine_timeout_s',45.0)
        self.declare_parameter('fine_engage_max_m',0.20)
        self.declare_parameter('runtime_limits_file','')
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        self.xy_tolerance=self.limits['arrival']['xy_tolerance_m']
        self.yaw_tolerance=self.limits['arrival']['yaw_tolerance_rad']
        self.fine_engage_max=self.limits['arrival']['fine_engage_max_m']
        self.fine_timeout=self.limits['arrival']['fine_timeout_s']
        self.max_v=self.limits['navigation']['max_forward_mps']
        self.max_w=self.limits['navigation']['max_yaw_rps']
        self.robot=self.get_parameter('robot_id').value
        path=self.get_parameter('waypoints_file').value
        self.poses=read_waypoints(path)
        self.action=ActionClient(self,NavigateToPose,f'/{self.robot}/navigate_to_pose')
        self.tf=Buffer(cache_time=Duration(seconds=15))
        self.tf_listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(String,'serving/status',10)
        self.event=self.create_publisher(String,'navigation_event',10)
        self.fine_pub=self.create_publisher(Twist,'cmd_vel_fine',10)
        self.fine_active=self.create_publisher(Bool,'fine_active',10)
        self.create_subscription(String,'serving/request',self.command,10)
        self.create_subscription(String,'serving/complete',self.confirm,10)
        self.create_subscription(Bool,'traffic_hold',self.set_hold,10)
        self.create_subscription(Odometry,'odom',self.on_odom,10)
        self.hold=True;self.odom=None;self.replan_pending=False;self.replan_count=0
        self.create_subscription(String,'person_replan_event',self.on_person_replan,10)
        self.state='IDLE';self.request_id=None;self.mode=None;self.table=None;self.side='auto'
        self.target_name=None;self.target=None;self.goal_handle=None;self.goal_pending=False
        self.goal_seq=0;self.fine=False;self.fine_since=0.;self.fail_reason=None
        self.create_timer(0.05,self.tick)
        self.get_logger().info(self.robot+' mission ready')

    def emit(self,type_,**kv):
        o={'robot_id':self.robot,'event':type_,'request_id':self.request_id,**kv}
        m=String(data=json.dumps(o,ensure_ascii=False));self.event.publish(m)

    def on_odom(self,msg):self.odom=msg
    def set_hold(self,msg):self.hold=bool(msg.data)

    def robot_pose(self):
        try:
            t=self.tf.lookup_transform('map',self.robot+'/base_footprint',Time(),timeout=Duration(seconds=0.07))
            tr=t.transform.translation
            return float(tr.x),float(tr.y),yaw(t.transform.rotation)
        except Exception:return None

    def pose_error(self):
        p=self.robot_pose()
        if p is None or self.target is None:return None
        return distance(p,self.target[:2]), norm_angle(self.target[2]-p[2]),p

    def command(self,msg):
        v={}
        try:
            v=json.loads(msg.data)
            command=v.get('command')
            if command=='RESET':
                if v.get('robot_id')!=self.robot or self.state not in ('ERROR','CANCELED','COMPLETED'):
                    raise ValueError('RESET allowed only after terminal state')
                if self.goal_handle or self.goal_pending or self.fine:
                    raise ValueError('Cannot reset while goal active')
                self.request_id=None;self.mode=None;self.table=None;self.target=None;self.target_name=None
                self.fail_reason=None;self.state='IDLE';self.emit('MISSION_RESET');return
            if command=='CANCEL':
                if v.get('request_id')!=self.request_id:raise ValueError('CANCEL request mismatch')
                self.cancel();return
            if command!='START':raise ValueError('Unknown mission command')
            if self.state not in ('IDLE','COMPLETED','CANCELED'):
                raise ValueError('Mission busy: '+self.state)
            if not self.action.server_is_ready():raise ValueError('Nav2 action server not ready')
            if self.hold:raise ValueError('Traffic / sensor hold active')
            if not self.robot_pose():raise ValueError('TF map to base_footprint unavailable')
            if v.get('robot_id')!=self.robot:raise ValueError('Wrong robot')
            if v.get('mode') not in ('KITCHEN_PICKUP','DELIVER_DIRECT','TABLE_COLLECT'):
                raise ValueError('Unknown mode')
            if v.get('table') not in ('table_01','table_02','table_03','table_04'):
                raise ValueError('Unknown table')
            self.request_id=v['request_id'];self.mode=v['mode'];self.table=v['table'];self.side=v.get('side','auto')
            self.fail_reason=None;self.replan_count=0;self.replan_pending=False
            if self.mode=='KITCHEN_PICKUP':self.start_goal('kitchen','GOING_PICKUP')
            else:self.start_goal(self.table,'GOING_DIRECT')
            self.emit('MISSION_ACCEPTED',state=self.state)
        except Exception as exc:self.emit('MISSION_REJECTED',request_id=v.get('request_id'),reason=str(exc))

    def confirm(self,msg):
        try:
            v=json.loads(msg.data)
            if v.get('request_id')!=self.request_id:raise ValueError('Request mismatch')
            command=v.get('command')
            if command=='LOAD' and self.mode=='KITCHEN_PICKUP' and self.state=='WAIT_LOAD':
                self.start_goal(self.table,'GOING_TABLE')
            elif command in ('COMPLETE','COLLECTED') and self.state=='WAIT_HANDOVER':
                self.start_goal(self.robot+'_HOME','RETURNING_HOME')
            else:raise ValueError('Unexpected confirmation '+str(command)+' at '+self.state)
        except Exception as exc:self.emit('CONFIRM_REJECTED',reason=str(exc))

    def start_goal(self,name,new_state):
        if self.goal_pending or self.goal_handle or self.fine:
            raise ValueError('Previous goal still active')
        here=self.robot_pose()
        goal=choose_waypoint(self.poses,name,here[:2] if here else None,self.side if name==self.table else 'auto')
        self.target_name=name
        self.target=(goal['x'],goal['y'],goal['yaw'])
        self.state=new_state
        g=NavigateToPose.Goal();g.pose.header.frame_id='map'
        g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=goal['x'];g.pose.pose.position.y=goal['y']
        g.pose.pose.orientation.z=math.sin(goal['yaw']/2)
        g.pose.pose.orientation.w=math.cos(goal['yaw']/2)
        self.goal_seq+=1;seq=self.goal_seq;self.goal_pending=True
        self.action.send_goal_async(g).add_done_callback(lambda f,s=seq:self.goal_response(f,s))
        self.emit('NAV_GOAL_SENT',target=name,xy=[goal['x'],goal['y']],yaw=goal['yaw'])

    def goal_response(self,f,seq):
        if seq!=self.goal_seq:return
        self.goal_pending=False
        try:
            h=f.result()
            if not h or not h.accepted:raise ValueError('Nav2 did not accept goal')
            self.goal_handle=h
            h.get_result_async().add_done_callback(lambda f,s=seq:self.goal_result(f,s))
        except Exception as exc:self.fail('NAV_GOAL_REJECTED',str(exc))

    def goal_result(self,f,seq):
        if seq!=self.goal_seq:return
        self.goal_handle=None;self.goal_pending=False
        try:status=f.result().status
        except Exception as exc:self.fail('NAV_RESULT_FAILED',str(exc));return
        if status==GoalStatus.STATUS_CANCELED and self.replan_pending:
            self.replan_pending=False
            self.emit('PERSON_REPLAN_CANCEL_ACK',count=self.replan_count)
            try:self.start_goal(self.target_name,self.state)
            except Exception as exc:self.fail('REPLAN_FAILED',str(exc))
            return
        if status==GoalStatus.STATUS_CANCELED:
            self.fine=False;self.state='CANCELED';self.emit('MISSION_CANCELED');return
        err=self.pose_error()
        if err is None:
            self.fail('POSE_UNAVAILABLE','TF unavailable for final verification');return
        d,angle,_=err
        self.emit('NAV_RESULT',nav_status=int(status),map_error_m=d,yaw_error_rad=angle)
        # Nav2 ABORT must never silently turn into mission success, even if pose looks near target.
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.fail('NAV_NOT_SUCCEEDED', 'Nav2 status %d, map error %.4fm'%(status,d));return
        if d<=self.xy_tolerance and abs(angle)<=self.yaw_tolerance:
            self.arrived('MAP_ESTIMATE_10CM_AND_YAW')
            return
        if d<=self.fine_engage_max:
            self.fine=True;self.fine_since=time.monotonic()
            self.emit('FINE_APPROACH_BEGIN',nav_status=int(status),map_error_m=d)
        else:self.fail('NAV_SUCCEEDED_OUTSIDE_ACCEPTANCE','map error %.4fm'%d)

    def on_person_replan(self,msg):
        if self.state not in ('GOING_PICKUP','GOING_TABLE','GOING_DIRECT','RETURNING_HOME'):
            return
        if self.replan_pending or self.goal_pending or not self.goal_handle or self.replan_count>=3:
            return
        try:
            event=json.loads(msg.data)
            if event.get('event') not in ('STATIONARY_PERSON_REPLAN','STATIONARY_PERSON_20S') or event.get('robot_id')!=self.robot:
                return
            self.replan_pending=True;self.replan_count+=1
            self.emit('PERSON_REPLAN_REQUEST',count=self.replan_count)
            self.goal_handle.cancel_goal_async()
        except Exception as exc:self.fail('PERSON_REPLAN_ERROR',str(exc))

    def arrived(self,proof):
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        try:self.state=next_after_nav(self.state,self.mode)
        except ValueError as exc:self.fail('STATE_ERROR',str(exc));return
        self.emit('NAV_ARRIVED',state=self.state,proof=proof)

    def fail(self,event,reason):
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        self.state='ERROR';self.fail_reason=reason
        self.emit(event,reason=reason)
        self.get_logger().error(self.robot+' '+event+': '+reason)

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_seq+=1;self.goal_handle=None;self.goal_pending=False;self.replan_pending=False
        self.fine=False;self.fine_pub.publish(Twist());self.fine_active.publish(Bool(data=False))
        self.state='CANCELED';self.emit('MISSION_CANCELED')

    def tick(self):
        self.fine_active.publish(Bool(data=bool(self.fine)))
        if self.fine:self.fine_step()
        report={'robot_id':self.robot,'state':self.state,'map_valid':self.robot_pose() is not None,
                'request_id':self.request_id,'mode':self.mode,'table':self.table,
                'target':self.target_name,'reason':self.fail_reason,'fine':self.fine,'nav_ready':self.action.server_is_ready()}
        self.pub.publish(String(data=json.dumps(report,ensure_ascii=False)))

    def fine_step(self):
        # Every command still flows through semantic safety and Collision Monitor.
        if time.monotonic()-self.fine_since > self.fine_timeout:
            self.fail('PRECISION_TIMEOUT','Could not reach configured map acceptance in time');return
        p=self.robot_pose()
        if p is None:
            self.fine_pub.publish(Twist());return
        tx,ty,ta=self.target;x,y,yaw_now=p
        d=math.hypot(tx-x,ty-y);eyaw=norm_angle(ta-yaw_now)
        if d<=self.xy_tolerance and abs(eyaw)<=self.yaw_tolerance:
            self.arrived('MAP_ESTIMATE_10CM_NOT_GAZEBO_TRUTH');return
        cmd=Twist()
        if self.hold:
            self.fine_pub.publish(cmd);return
        if d>self.xy_tolerance:
            heading=math.atan2(ty-y,tx-x)
            turn=norm_angle(heading-yaw_now)
            if abs(turn)>0.12:
                cmd.angular.z=max(-0.25,min(0.25,1.0*turn))
            else:
                # no reverse; near-goal velocity decreases continuously
                cmd.linear.x=min(0.05,max(0.005,0.8*d))
                cmd.angular.z=max(-0.20,min(0.20,turn))
        elif abs(eyaw)>0.03:
            cmd.angular.z=max(-0.20,min(0.20,eyaw))
        cmd.linear.x,cmd.angular.z=velocity_limit(cmd.linear.x,cmd.angular.z,self.max_v,self.max_w)
        self.fine_pub.publish(cmd)

def main():
    rclpy.init();n=Mission()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
