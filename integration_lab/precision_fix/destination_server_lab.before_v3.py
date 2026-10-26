#!/usr/bin/env python3
"""Fleet-facing ROS bridge: destination_id -> Semantic input -> Nav2.

Input std_msgs/String JSON /robot_N/destination_command:
 {'destination_id':'table_03','map_revision':1}
 {'command':'CANCEL'}
 Never accept x/y from external requests. Never claim Fleet Task SUCCEEDED.
"""
import json
import math
import sys
import time
from pathlib import Path
import yaml

from arrival_policy import evaluate_arrival
from navigate_destination import resolve_destination


def load_semantic(path, expected_revision, destination_id):
    d=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(d,dict):raise ValueError('Semantic map unavailable')
    revision=d.get('map_revision')
    if not isinstance(revision,int):raise ValueError('Semantic map_revision missing')
    if expected_revision is not None and (type(expected_revision)!=int or expected_revision!=revision):
        raise ValueError('map_revision mismatch; reject stale goal')
    return resolve_destination(Path(path),destination_id),revision


def parse_command(text):
    obj=json.loads(text)
    if not isinstance(obj,dict):raise ValueError('Destination command must be JSON object')
    if obj.get('command')=='CANCEL':return {'command':'CANCEL'}
    if set(obj).difference({'destination_id','map_revision','command'}):
        raise ValueError('Only destination_id and optional map_revision allowed')
    name=obj.get('destination_id')
    if not isinstance(name,str) or not name.strip():raise ValueError('Invalid destination_id')
    return {'command':'GO','destination_id':name,'map_revision':obj.get('map_revision')}


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_msgs.msg import String
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    class DestinationServer(Node):
        def __init__(self):
            super().__init__('destination_server')
            self.robot=self.declare_parameter('robot_id','robot_01').value
            self.waypoints=Path(self.declare_parameter('waypoints_file','').value)
            config=Path(self.declare_parameter('arrival_file','').value)
            self.arrival=yaml.safe_load(config.read_text())['arrival']
            self.state='IDLE';self.active=None;self.pending=False;self.last_odom=None
            self.tf_buf=Buffer();self.tf_listener=TransformListener(self.tf_buf,self)
            self.cli=ActionClient(self,NavigateToPose,f'/{self.robot}/navigate_to_pose')
            self.pub_state=self.create_publisher(String,f'/{self.robot}/robot_state',10)
            self.pub_event=self.create_publisher(String,f'/{self.robot}/navigation_event',10)
            self.sub=self.create_subscription(String,f'/{self.robot}/destination_command',self.command,10)
            self.odom_sub=self.create_subscription(Odometry,f'/{self.robot}/odom',self.on_odom,10)
            self.timer=self.create_timer(.2,self.tick)
            self.goaltime=None;self.nav_succeeded=False;self.goal_handle=None
            self.cancel_after_accept=False;self.final_started=None;self.detected_announced=False
            # LAB-only bounded precision recovery. Never mark SERVING on Nav2 alone.
            self.precision_retries=0;self.max_precision_retries=2
            self.retry_pending=False;self.retry_deadline=None;self.last_pose_error=None
            self.retry_baseline_xy=None
            self.last_yaw=None;self.cumulative_turn=0.0;self.spin_limit_rad=2*math.pi
            self.spin_anchor_xy=None;self.spin_window_turn=0.0;self.spin_anchor_time=None
            self.terminal_failure=False
        def on_odom(self,msg):self.last_odom=msg
        def state_to(self,state,detail=''):
            self.state=state;self.pub_state.publish(String(data=json.dumps({
               'robot_id':self.robot,'state':state,'destination_id':
               self.active['destination_id'] if self.active else None,'detail':detail})))
        def event(self,event,**data):
            self.pub_event.publish(String(data=json.dumps({'robot_id':self.robot,
                               'event':event,**data})))
        def command(self,msg):
            try:
                cmd=parse_command(msg.data)
                if cmd['command']=='CANCEL':
                    self.retry_pending=False
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                        self.event('CANCEL_REQUESTED')
                    elif self.pending:
                        self.cancel_after_accept=True
                        self.event('CANCEL_DEFERRED',reason='Cancel after goal acceptance')
                    else:self.event('NO_ACTIVE_GOAL')
                    return
                if self.active or self.pending:
                    raise ValueError('Robot already has active goal; no preemption')
                wp,rev=load_semantic(self.waypoints,cmd['map_revision'],cmd['destination_id'])
                if not self.cli.server_is_ready():
                    raise RuntimeError('Nav2 action unavailable')
                goal=NavigateToPose.Goal()
                pose=PoseStamped();pose.header.frame_id='map'
                pose.header.stamp=self.get_clock().now().to_msg()
                pose.pose.position.x=wp.x;pose.pose.position.y=wp.y
                pose.pose.orientation.z=math.sin(wp.yaw*.5)
                pose.pose.orientation.w=math.cos(wp.yaw*.5)
                goal.pose=pose
                self.cancel_after_accept=False;self.detected_announced=False
                self.precision_retries=0;self.retry_pending=False
                self.last_pose_error=None;self.last_yaw=None;self.cumulative_turn=0.0
                self.retry_baseline_xy=None
                self.spin_anchor_xy=None;self.spin_window_turn=0.0;self.spin_anchor_time=None
                self.terminal_failure=False;self.nav_succeeded=False
                self.active={'destination_id':cmd['destination_id'],
                             'pose':(wp.x,wp.y,wp.yaw),'map_revision':rev}
                self.pending=True
                fut=self.cli.send_goal_async(goal)
                fut.add_done_callback(self.accepted)
                self.state_to('NAVIGATING')
            except Exception as e:
                self.event('REJECTED',reason=str(e))
        def accepted(self,fut):
            self.pending=False
            try:
                handle=fut.result()
                if not handle.accepted:raise RuntimeError('Goal rejected')
                self.goal_handle=handle
                if self.cancel_after_accept:
                    handle.cancel_goal_async()
                    self.event('CANCEL_REQUESTED_AFTER_ACCEPT')
                handle.get_result_async().add_done_callback(self.finished)
                self.event('NAV_GOAL_ACCEPTED',map_revision=self.active['map_revision'])
            except Exception as e:
                self.event('NAV_GOAL_FAILED',reason=str(e))
                self.active=None;self.state_to('ERROR',str(e))
        def finished(self,fut):
            try:
                status=fut.result().status
                if status==GoalStatus.STATUS_SUCCEEDED:
                    self.nav_succeeded=True
                    self.goal_handle=None  # Action finished; a new precision goal may be issued.
                    self.final_started=time.monotonic()
                    self.event('NAV2_SUCCEEDED_NOT_SERVING')
                    # Keep active until physical final-arrival verification.
                    return
                self.event('NAV_GOAL_TERMINATED',status=int(status))
                self.active=None;self.goal_handle=None;self.nav_succeeded=False
                self.retry_pending=False
                self.state_to('PAUSED' if status==GoalStatus.STATUS_CANCELED else 'ERROR')
            except Exception as e:
                self.event('RESULT_ERROR',reason=str(e));self.state_to('ERROR',str(e))
                self.active=None;self.goal_handle=None;self.nav_succeeded=False
        def _final_error(self, reason, xy=None, yaw=None):
            if self.terminal_failure:
                return
            self.terminal_failure=True
            details={'reason':reason,'precision_retries':self.precision_retries}
            if xy is not None: details['xy_error_m']=round(xy,6)
            if yaw is not None: details['yaw_error_rad']=round(yaw,6)
            self.event('FINAL_ACCURACY_NOT_REACHED',**details)
            self.active=None;self.goal_handle=None;self.nav_succeeded=False
            self.retry_pending=False
            self.state_to('ERROR',reason)

        def _reissue_goal(self):
            if self.active is None or self.retry_pending:
                return
            if not self.cli.server_is_ready():
                self._final_error('Nav2 action server unavailable during correction')
                return
            x,y,yaw=self.active['pose']
            pose=PoseStamped();pose.header.frame_id='map'
            pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.position.x=x;pose.pose.position.y=y
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
            goal=NavigateToPose.Goal();goal.pose=pose
            self.retry_baseline_xy=self.last_pose_error['xy_error_m']
            self.precision_retries+=1
            self.nav_succeeded=False;self.final_started=None;self.retry_pending=True
            self.event('PRECISION_RETRY',attempt=self.precision_retries,
                       max_attempts=self.max_precision_retries,
                       xy_error_m=round(self.last_pose_error['xy_error_m'],6),
                       yaw_error_rad=round(self.last_pose_error['yaw_error_rad'],6))
            future=self.cli.send_goal_async(goal)
            future.add_done_callback(self._retry_accepted)

        def _retry_accepted(self, future):
            if self.active is None:
                return
            self.retry_pending=False
            try:
                handle=future.result()
                if not handle.accepted: raise RuntimeError('Correction goal rejected')
                self.goal_handle=handle
                handle.get_result_async().add_done_callback(self.finished)
                self.event('PRECISION_RETRY_ACCEPTED',attempt=self.precision_retries)
            except Exception as exc:
                self._final_error('Precision retry failed: '+str(exc))

        def tick(self):
            if self.active is None or self.last_odom is None:
                return
            try:
                frame='base_footprint' if self.robot=='robot_01' else f'{self.robot}/base_footprint'
                tf=self.tf_buf.lookup_transform('map',frame,rclpy.time.Time())
                q=tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                pose=(tf.transform.translation.x,tf.transform.translation.y,yaw)
                # Oscillation guard concerns *local non-progress*, not all turns
                # accumulated across a legitimate route. Retains cancel-on-stall.
                xy_now=(pose[0],pose[1])
                now=time.monotonic()
                if self.spin_anchor_xy is None:
                    self.spin_anchor_xy=xy_now;self.spin_anchor_time=now
                if math.hypot(xy_now[0]-self.spin_anchor_xy[0],xy_now[1]-self.spin_anchor_xy[1])>=0.08:
                    self.spin_anchor_xy=xy_now;self.spin_anchor_time=now;self.spin_window_turn=0.0
                if self.last_yaw is not None:
                    turn=abs((yaw-self.last_yaw+math.pi)%(2*math.pi)-math.pi)
                    self.cumulative_turn+=turn
                    self.spin_window_turn+=turn
                self.last_yaw=yaw
                if now-self.spin_anchor_time>=20.0 and self.spin_window_turn>math.pi*1.5:
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                    self._final_error('ROTATING_WITHOUT_TRANSLATION',xy=self.last_pose_error['xy_error_m'] if self.last_pose_error else None)
                    return
                twist=self.last_odom.twist.twist
                a=self.arrival
                d=evaluate_arrival(pose,self.active['pose'],twist.linear.x,twist.angular.z,
                     a['detection_radius_m'],a['final_error_m'],a['yaw_tolerance_rad'],
                     a['stopped_linear_mps'],a['stopped_angular_rps'])
                self.last_pose_error=d
                if d['detected'] and not self.detected_announced:
                    self.detected_announced=True
                    self.event('WITHIN_5CM',xy_error_m=d['xy_error_m'])
                if d['final_arrived'] and self.nav_succeeded:
                    self.event('PHYSICAL_ARRIVAL_VERIFIED',xy_error_m=d['xy_error_m'],
                               yaw_error_rad=d['yaw_error_rad'],precision_retries=self.precision_retries)
                    self.state_to('SERVING')
                    self.active=None;self.goal_handle=None;self.nav_succeeded=False
                    self.retry_pending=False
                    return
                if self.nav_succeeded and self.final_started is not None:
                    if time.monotonic()-self.final_started >= float(a['final_verify_timeout_s']):
                        # A repeat goal without measurable XY progress is not a correction.
                        if (self.retry_baseline_xy is not None and
                            d['xy_error_m'] >= self.retry_baseline_xy - 0.001):
                            self._final_error('NO_XY_PROGRESS_AFTER_RETRY',d['xy_error_m'],d['yaw_error_rad'])
                        elif self.precision_retries<self.max_precision_retries:
                            self._reissue_goal()
                        else:
                            self._final_error('Precision correction exhausted',d['xy_error_m'],d['yaw_error_rad'])
            except Exception as exc:
                self.get_logger().debug(f'Awaiting valid localization/pose: {exc}')
    rclpy.init(args=args);node=None
    try:
        node=DestinationServer();rclpy.spin(node)
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
