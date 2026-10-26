#!/usr/bin/env python3
"""Fleet-facing ROS bridge: destination_id -> Semantic input -> Nav2.

V4 = V3 safe no-reissue + final-centimetre fine approach (fine_approach.py).

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
from fine_approach import FineApproach, fine_params, map_pose_to_odom
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
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    class DestinationServer(Node):
        def __init__(self):
            super().__init__('destination_server')
            self.robot=self.declare_parameter('robot_id','robot_01').value
            self.waypoints=Path(self.declare_parameter('waypoints_file','').value)
            config=Path(self.declare_parameter('arrival_file','').value)
            cfg_all=yaml.safe_load(config.read_text())
            self.arrival=cfg_all['arrival']
            # V4: final-centimetre approach. Never sends a Nav2 goal; publishes to the
            # velocity-smoother input so Semantic Safety + Collision Monitor stay in line.
            self.fine_enabled=bool(self.declare_parameter('fine_approach_enabled',True).value)
            self.fine_cfg=fine_params(cfg_all.get('fine_approach'))
            # A narrowly scoped handoff only after the ACTIVE Nav2 goal is
            # terminally canceled. Never let two controllers publish motion.
            handoff_cfg=cfg_all.get('near_goal_handoff', {})
            self.handoff_enabled=bool(handoff_cfg.get('enabled', False))
            self.handoff_radius=float(handoff_cfg.get('radius_m', 0.06))
            self.handoff_stall=float(handoff_cfg.get('stalled_for_s', 6.0))
            if not (0 < self.handoff_radius <= self.fine_cfg['engage_max_m']
                    and self.handoff_stall >= 2.0):
                raise ValueError('invalid near_goal_handoff radius/time')
            # Optional second trigger for a robot that keeps MOVING (orbiting /
            # dithering) near the goal without getting closer. 0 = disabled.
            self.linger_radius=float(handoff_cfg.get('linger_radius_m', 0.0))
            self.linger_for=float(handoff_cfg.get('linger_for_s', 8.0))
            self.linger_min_progress=float(handoff_cfg.get('linger_min_progress_m', 0.01))
            if self.linger_radius != 0.0 and not (
                    0 < self.linger_radius <= self.fine_cfg['engage_max_m']
                    and self.linger_for >= 3.0 and 0 < self.linger_min_progress <= 0.05):
                raise ValueError('invalid near_goal_handoff linger settings')
            self.linger_best=None;self.linger_best_at=None;self.linger_since=None
            self.handoff_ready=False
            self.handoff_requested=False
            self.handoff_anchor=None
            self.handoff_since=None
            self.last_odom_at=None
            self.fine=None;self.fine_attempts=0;self.fine_idle_since=time.monotonic();self.fine_prev_t=None
            self.cmd_pub=self.create_publisher(Twist,f"/{self.robot}/{self.fine_cfg['cmd_topic']}",10)
            self.state='IDLE';self.active=None;self.pending=False;self.last_odom=None
            self.tf_buf=Buffer();self.tf_listener=TransformListener(self.tf_buf,self)
            self.cli=ActionClient(self,NavigateToPose,f'/{self.robot}/navigate_to_pose')
            self.pub_state=self.create_publisher(String,f'/{self.robot}/robot_state',10)
            self.pub_event=self.create_publisher(String,f'/{self.robot}/navigation_event',10)
            self.sub=self.create_subscription(String,f'/{self.robot}/destination_command',self.command,10)
            self.odom_sub=self.create_subscription(Odometry,f'/{self.robot}/odom',self.on_odom,10)
            self.timer=self.create_timer(.2,self.tick)
            self.fine_timer=self.create_timer(.05,self.fine_tick)
            self.goaltime=None;self.nav_succeeded=False;self.goal_handle=None
            self.cancel_after_accept=False;self.final_started=None;self.detected_announced=False
            # LAB-only bounded precision recovery. Never mark SERVING on Nav2 alone.
            self.precision_retries=0;self.max_precision_retries=2
            self.retry_pending=False;self.retry_deadline=None;self.last_pose_error=None
            self.last_yaw=None;self.cumulative_turn=0.0;self.spin_limit_rad=2*math.pi
            self.spin_anchor_xy=None;self.spin_anchor_time=None;self.spin_window_turn=0.0
            self.terminal_failure=False
            self.nav_timeout_s=float(self.declare_parameter('navigation_timeout_s',300.0).value)
            if self.nav_timeout_s <= 0: raise ValueError('navigation_timeout_s must be positive')
            self.goal_sent_at=None
            self.goal_generation=0
            self.cancel_requested=False
            # Do not dispatch another GO while an Action response/result is unresolved.
            self.unresolved_goal=False
        def on_odom(self,msg):
            self.last_odom=msg
            self.last_odom_at=time.monotonic()
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
                    self._stop_fine()
                    self.retry_pending=False
                    self.cancel_requested=True
                    self.cancel_after_accept=True
                    if (self.nav_succeeded or self.handoff_ready) and self.goal_handle is None:
                        self.active=None;self.nav_succeeded=False;self.handoff_ready=False
                        self.state_to('PAUSED','CANCEL_REQUESTED_DURING_FINAL_CHECK')
                        return
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                        self.event('CANCEL_REQUESTED')
                    elif self.pending:
                        self.cancel_after_accept=True
                        self.event('CANCEL_DEFERRED',reason='Cancel after goal acceptance')
                    else:self.event('NO_ACTIVE_GOAL')
                    return
                if self.active or self.pending or self.goal_handle is not None or self.unresolved_goal:
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
                self.spin_anchor_xy=None;self.spin_anchor_time=None;self.spin_window_turn=0.0
                self.terminal_failure=False;self.nav_succeeded=False
                self.handoff_ready=False;self.handoff_requested=False
                self.handoff_anchor=None;self.handoff_since=None
                self.linger_best=None;self.linger_best_at=None;self.linger_since=None
                self.cancel_requested=False
                self.goal_generation+=1
                generation=self.goal_generation
                self.goal_sent_at=time.monotonic()
                self.active={'destination_id':cmd['destination_id'],
                             'pose':(wp.x,wp.y,wp.yaw),'map_revision':rev}
                self.pending=True
                fut=self.cli.send_goal_async(goal)
                self.unresolved_goal=True
                fut.add_done_callback(lambda result, g=generation:self.accepted(result,g))
                self.state_to('NAVIGATING')
            except Exception as e:
                self.event('REJECTED',reason=str(e))
        def accepted(self,fut,generation):
            # A late acceptance must never leave an untracked active Nav2 goal.
            try:
                handle=fut.result()
            except Exception as exc:
                if generation==self.goal_generation:
                    self.unresolved_goal=False
                if generation==self.goal_generation and not self.terminal_failure:
                    self.pending=False;self.active=None
                    self.state_to('ERROR',f'NAV_GOAL_ACCEPTANCE_FAILED:{exc}')
                return
            if not handle.accepted:
                if generation==self.goal_generation:
                    self.unresolved_goal=False
                if generation==self.goal_generation and not self.terminal_failure:
                    self.pending=False;self.active=None
                    self.state_to('ERROR','NAV_GOAL_REJECTED')
                return
            stale=(generation!=self.goal_generation or self.terminal_failure
                   or self.active is None)
            if stale or self.cancel_requested or self.cancel_after_accept:
                handle.cancel_goal_async()
                self.event('CANCEL_REQUESTED_AFTER_ACCEPT',stale=bool(stale))
                if generation==self.goal_generation:
                    self.pending=False
                    self.goal_handle=handle
            else:
                self.pending=False
                self.goal_handle=handle
                self.event('NAV_GOAL_ACCEPTED',map_revision=self.active['map_revision'])
            handle.get_result_async().add_done_callback(
                lambda result,g=generation:self.finished(result,g))

        def finished(self,fut,generation):
            # Ignore late results of any previous goal, and preserve ERROR.
            if generation!=self.goal_generation:
                return
            self.unresolved_goal=False
            self.pending=False
            self.goal_handle=None
            if self.terminal_failure:
                return
            try:
                status=fut.result().status
                if self.cancel_requested or self.cancel_after_accept:
                    self.event('NAV_GOAL_TERMINATED',status=int(status),cancel_requested=True)
                    self.active=None;self.nav_succeeded=False
                    self.state_to('PAUSED','CANCEL_REQUESTED')
                    return
                if (self.handoff_requested and status==GoalStatus.STATUS_CANCELED
                        and self.handoff_enabled and self.active is not None):
                    # Nav2 has stopped and returned a terminal Action result.
                    # Only now may fine approach own cmd_vel_nav.
                    self.handoff_requested=False
                    self.handoff_ready=True
                    self.final_started=time.monotonic()
                    self.fine=None;self.fine_attempts=0
                    self.fine_idle_since=self.final_started
                    self.event('NEAR_GOAL_HANDOFF_READY')
                    return
                if status==GoalStatus.STATUS_SUCCEEDED:
                    self.nav_succeeded=True;self.handoff_requested=False
                    self.final_started=time.monotonic()
                    self.fine=None;self.fine_attempts=0;self.fine_idle_since=self.final_started
                    self.event('NAV2_SUCCEEDED_NOT_SERVING')
                    return
                self.event('NAV_GOAL_TERMINATED',status=int(status))
                self.active=None;self.nav_succeeded=False
                self.retry_pending=False
                self.state_to('PAUSED' if status==GoalStatus.STATUS_CANCELED else 'ERROR')
            except Exception as exc:
                self.event('RESULT_ERROR',reason=str(exc))
                self.active=None;self.nav_succeeded=False
                self.state_to('ERROR',str(exc))

        def _pub_cmd(self,v,w):
            c=self.fine_cfg
            m=Twist()
            m.linear.x=float(max(0.0,min(v,c['v_max'])))   # forward only
            m.angular.z=float(max(-c['w_max'],min(w,c['w_max'])))
            self.cmd_pub.publish(m)

        def _stop_fine(self):
            if self.fine is not None:
                self.fine=None
                self.cmd_pub.publish(Twist())              # explicit zero

        def _pose2d(self,tf):
            q=tf.transform.rotation
            return (tf.transform.translation.x,tf.transform.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

        def fine_tick(self):
            if not self.fine_enabled or self.terminal_failure:
                return
            if self.active is None or not (self.nav_succeeded or self.handoff_ready):
                self._stop_fine();return
            now=time.monotonic();c=self.fine_cfg
            try:
                base='base_footprint' if self.robot=='robot_01' else f'{self.robot}/base_footprint'
                odom='odom' if self.robot=='robot_01' else f'{self.robot}/odom'
                pose=self._pose2d(self.tf_buf.lookup_transform(odom,base,rclpy.time.Time()))
                if self.fine is None:
                    if self.fine_attempts>=c['max_attempts'] or self.last_odom is None:
                        return
                    if now-self.fine_idle_since<c['settle_s']:
                        return
                    tw=self.last_odom.twist.twist
                    if (abs(tw.linear.x)>self.arrival['stopped_linear_mps']
                            or abs(tw.angular.z)>self.arrival['stopped_angular_rps']):
                        return
                    om=self._pose2d(self.tf_buf.lookup_transform(odom,'map',rclpy.time.Time()))
                    target=map_pose_to_odom(om,self.active['pose'])   # frozen in odom
                    dist0=math.hypot(target[0]-pose[0],target[1]-pose[1])
                    if dist0>c['engage_max_m']:
                        self._final_error('FINE_APPROACH_TOO_FAR',dist0);return
                    self.fine=FineApproach(c,target);self.fine_attempts+=1;self.fine_prev_t=now
                    self.event('FINE_APPROACH_STARTED',attempt=self.fine_attempts,xy_error_m=round(dist0,6))
                dt=max(1e-3,min(0.2,now-self.fine_prev_t));self.fine_prev_t=now
                v,w,status,info=self.fine.step(pose,dt)
                if status=='RUN':
                    self._pub_cmd(v,w);return
                self._stop_fine();self.fine_idle_since=now
                self.event('FINE_APPROACH_'+status,attempt=self.fine_attempts,**info)
            except Exception as exc:
                self.get_logger().debug(f'Fine approach waiting for valid TF: {exc}')

        def _final_error(self, reason, xy=None, yaw=None):
            if self.terminal_failure:
                return
            self.terminal_failure=True
            self._stop_fine()
            self.cancel_after_accept=True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            details={'reason':reason,'precision_retries':self.precision_retries}
            if xy is not None: details['xy_error_m']=round(xy,6)
            if yaw is not None: details['yaw_error_rad']=round(yaw,6)
            self.event('FINAL_ACCURACY_NOT_REACHED',**details)
            self.active=None;self.nav_succeeded=False;self.handoff_ready=False
            self.retry_pending=False
            # Clear local pending only. unresolved_goal remains True until actual
            # Action rejection, exception or terminal result is observed.
            # Never infer remote completion from a local timeout.
            self.pending=False
            self.state_to('ERROR',reason)

        def _reissue_goal(self):
            # Intentionally disabled. A new NavigateToPose after SUCCEEDED
            # resets local control for the same pose and can cause oscillation.
            # Do not send a new goal as a precision correction.
            if self.active is not None:
                err=self.last_pose_error or {}
                self._final_error('PRECISION_NOT_REACHED_NO_REISSUE',
                                  err.get('xy_error_m'),err.get('yaw_error_rad'))

        def _consider_handoff(self, pose, now):
            """Cancel *only* a settled, motionless Nav2 goal very near its endpoint.

            No direct driving is performed here. A subsequent terminal canceled
            Nav2 result is required before fine_tick may issue any velocity.
            """
            if (not self.handoff_enabled or not self.fine_enabled
                    or self.nav_succeeded or self.handoff_ready or self.handoff_requested
                    or self.pending or not self.unresolved_goal or self.goal_handle is None
                    or self.cancel_requested or self.terminal_failure):
                self.handoff_since=None;self.handoff_anchor=None
                self.linger_best=None;self.linger_best_at=None;self.linger_since=None
                return
            if self.last_odom_at is None or now-self.last_odom_at>0.5:
                self.handoff_since=None;self.handoff_anchor=None
                self.linger_best=None;self.linger_best_at=None;self.linger_since=None
                return
            dx=pose[0]-self.active['pose'][0];dy=pose[1]-self.active['pose'][1]
            dist=math.hypot(dx,dy)
            tw=self.last_odom.twist.twist
            if self.linger_radius > 0.0:
                if dist > self.linger_radius:
                    self.linger_best=None;self.linger_best_at=None;self.linger_since=None
                else:
                    if self.linger_since is None: self.linger_since=now
                    if (self.linger_best is None
                            or dist < self.linger_best - self.linger_min_progress):
                        self.linger_best=dist;self.linger_best_at=now
                    if (now-self.linger_best_at >= self.linger_for
                            and now-self.linger_since >= self.linger_for):
                        try:
                            self.goal_handle.cancel_goal_async()
                            self.handoff_requested=True
                            self.event('NEAR_GOAL_HANDOFF_CANCEL_REQUESTED',
                                       xy_error_m=round(dist,6),trigger='LINGER')
                        except Exception as exc:
                            self.event('NEAR_GOAL_HANDOFF_CANCEL_FAILED',reason=str(exc))
                            self.linger_best_at=now
                        return
            if (dist>self.handoff_radius
                    or abs(tw.linear.x)>self.arrival['stopped_linear_mps']
                    or abs(tw.angular.z)>self.arrival['stopped_angular_rps']):
                self.handoff_since=None;self.handoff_anchor=None
                return
            if self.handoff_anchor is None:
                self.handoff_anchor=pose;self.handoff_since=now
                return
            if (math.hypot(pose[0]-self.handoff_anchor[0],pose[1]-self.handoff_anchor[1])>0.003
                    or abs((pose[2]-self.handoff_anchor[2]+math.pi)%(2*math.pi)-math.pi)>0.15):
                self.handoff_anchor=pose;self.handoff_since=now
                return
            if now-self.handoff_since>=self.handoff_stall:
                try:
                    self.goal_handle.cancel_goal_async()
                    self.handoff_requested=True
                    self.event('NEAR_GOAL_HANDOFF_CANCEL_REQUESTED',xy_error_m=round(dist,6))
                except Exception as exc:
                    self.event('NEAR_GOAL_HANDOFF_CANCEL_FAILED',reason=str(exc))
                    self.handoff_since=now

        def tick(self):
            if self.active is None or self.terminal_failure:
                return
            if (not self.nav_succeeded and self.goal_sent_at is not None
                    and time.monotonic()-self.goal_sent_at>self.nav_timeout_s):
                self._final_error('NAVIGATION_TIMEOUT')
                return
            if self.last_odom is None:
                return
            try:
                frame='base_footprint' if self.robot=='robot_01' else f'{self.robot}/base_footprint'
                tf=self.tf_buf.lookup_transform('map',frame,rclpy.time.Time())
                q=tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                pose=(tf.transform.translation.x,tf.transform.translation.y,yaw)
                # Detect spinning without translational progress; do not reject valid
                # winding routes merely for accumulating more than 360 degrees.
                xy_now=(pose[0],pose[1]); now=time.monotonic()
                if self.spin_anchor_xy is None:
                    self.spin_anchor_xy=xy_now;self.spin_anchor_time=now
                    self.spin_window_turn=0.0
                if math.hypot(xy_now[0]-self.spin_anchor_xy[0],xy_now[1]-self.spin_anchor_xy[1])>=0.08:
                    self.spin_anchor_xy=xy_now;self.spin_anchor_time=now
                    self.spin_window_turn=0.0
                if self.last_yaw is not None:
                    turn=abs((yaw-self.last_yaw+math.pi)%(2*math.pi)-math.pi)
                    self.cumulative_turn+=turn
                    self.spin_window_turn+=turn
                self.last_yaw=yaw
                if self.fine is not None:
                    # Fine approach legitimately U-turns in place (own timeout applies).
                    self.spin_anchor_xy=xy_now;self.spin_anchor_time=now;self.spin_window_turn=0.0
                if now-self.spin_anchor_time>=20.0 and self.spin_window_turn>1.5*math.pi:
                    self._final_error('ROTATING_WITHOUT_TRANSLATION',xy=self.last_pose_error['xy_error_m'] if self.last_pose_error else None)
                    return
                twist=self.last_odom.twist.twist
                a=self.arrival
                d=evaluate_arrival(pose,self.active['pose'],twist.linear.x,twist.angular.z,
                     a['detection_radius_m'],a['final_error_m'],a['yaw_tolerance_rad'],
                     a['stopped_linear_mps'],a['stopped_angular_rps'])
                self.last_pose_error=d
                if not self.nav_succeeded and not self.handoff_ready:
                    self._consider_handoff(pose, now)
                if d['detected'] and not self.detected_announced:
                    self.detected_announced=True
                    self.event('WITHIN_5CM',xy_error_m=d['xy_error_m'])
                if d['final_arrived'] and (self.nav_succeeded or self.handoff_ready):
                    self.event('PHYSICAL_ARRIVAL_VERIFIED',xy_error_m=d['xy_error_m'],
                               yaw_error_rad=d['yaw_error_rad'],precision_retries=self.precision_retries)
                    self._stop_fine()
                    self.state_to('SERVING')
                    self.active=None;self.goal_handle=None;self.nav_succeeded=False;self.handoff_ready=False
                    self.retry_pending=False
                    return
                if (self.nav_succeeded or self.handoff_ready) and self.final_started is not None:
                    if (self.fine_enabled and self.fine is None
                            and self.fine_attempts>=self.fine_cfg['max_attempts']
                            and time.monotonic()-self.fine_idle_since>=self.fine_cfg['settle_s']):
                        self._final_error('PRECISION_NOT_REACHED_AFTER_FINE_APPROACH',
                                          d['xy_error_m'],d['yaw_error_rad'])
                        return
                    if time.monotonic()-self.final_started >= float(a['final_verify_timeout_s']):
                        # Never restart a navigation Action merely to improve
                        # final precision. Preserve the recorded XY/yaw error.
                        self._final_error('PRECISION_NOT_REACHED_NO_REISSUE',
                                          d['xy_error_m'],d['yaw_error_rad'])
            except Exception as exc:
                self.get_logger().debug(f'Awaiting valid localization/pose: {exc}')
    rclpy.init(args=args);node=None
    try:
        node=DestinationServer();rclpy.spin(node)
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
