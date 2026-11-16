"""Nonblocking small Nav2 scan sweep, with unchanged live safety gates."""
import time,math
from .mapping_bootstrap import bootstrap_plan,bootstrap_scan_check


class ObservationSweep:
    def __init__(self,node,on_finished):
        from rclpy.action import ActionClient
        from nav2_msgs.action import Spin
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.done=on_finished;self.client=ActionClient(node,Spin,'/mapping_scan_spin')
        self.state={};self.busy=False;self.handle=None;self.result=None;self.ack=None;self.cancel_ack=None
        self.phase='IDLE';self.closed=False
        self.sub=node.create_subscription(Odometry,'/robot_01/odom',self.odom,qos_profile_sensor_data)
        self.timer=node.create_timer(.05,self.tick)

    def odom(self,m):
        self.state.update(linear=abs(m.twist.twist.linear.x),angular=abs(m.twist.twist.angular.z),wall=time.monotonic())

    def start(self):
        if self.busy or self.closed:raise RuntimeError('Observation sweep unavailable')
        self.busy=True;self.phase='SETTLE';self.index=0;self.plan=bootstrap_plan(.18)
        self.events=[];self.reason=None;self.started=time.monotonic();self.deadline=self.started+15.;self.stable=None
        self.handle=None;self.result=None;self.ack=None;self.cancel_ack=None

    def cancel(self,reason):
        if not self.busy:return
        self.reason=reason
        if self.handle is not None and self.cancel_ack is None:
            self.cancel_ack=self.handle.cancel_goal_async()
        self.phase='CANCEL';self.deadline=time.monotonic()+8.

    def finish(self,okay,reason):
        if reason=='OBSERVATION_CANCEL_UNCONFIRMED':
            self.phase='UNCERTAIN';self.done(False,reason,self.events);return
        self.busy=False;self.phase='IDLE';self.handle=None
        self.done(okay,reason,self.events)

    def tick(self):
        if not self.busy:return
        if self.phase=='UNCERTAIN':return
        now=time.monotonic()
        if self.phase=='CANCEL':
            if self.ack is not None and self.handle is None and self.ack.done():
                h=self.ack.result()
                if h.accepted:self.handle=h;self.result=h.get_result_async();self.cancel_ack=h.cancel_goal_async()
                else:self.finish(False,self.reason);return
            if self.handle is None and self.ack is None:self.finish(False,self.reason);return
            if self.result is not None and self.result.done() and self.cancel_ack is not None and self.cancel_ack.done():
                response=self.cancel_ack.result()
                if self.result.result().status==4 or (self.result.result().status==5 and response.goals_canceling):
                    self.finish(False,self.reason);return
            if now>self.deadline:self.finish(False,'OBSERVATION_CANCEL_UNCONFIRMED');return
            return
        scan_state=dict(scan=getattr(self.node,'navigation_start_scan',None),
                        scan_wall=getattr(self.node,'navigation_start_scan_wall',-100.))
        raw,raw_reason=bootstrap_scan_check(scan_state,now)
        safe=self.node.mapping_navigation_safety_check()
        if self.phase=='SETTLE':
            stopped=(now-self.state.get('wall',-100)<.5 and self.state.get('linear',1)<=.015 and self.state.get('angular',1)<=.05)
            self.stable=(now if self.stable is None else self.stable) if stopped and raw and safe else None
            if now>self.deadline:self.finish(False,'OBSERVATION_READY_TIMEOUT');return
            if self.stable is None or now-self.stable<.5 or not self.client.server_is_ready():return
            from nav2_msgs.action import Spin
            goal=Spin.Goal();goal.target_yaw=self.plan[self.index];goal.time_allowance.sec=15
            self.ack=self.client.send_goal_async(goal);self.phase='ACK';self.deadline=now+8.;return
        if not raw or not safe:self.cancel('OBSERVATION_SAFETY_NOT_CLEAR: '+raw_reason);return
        if self.phase=='ACK':
            if self.ack.done():
                self.handle=self.ack.result()
                if not self.handle.accepted:self.finish(False,'OBSERVATION_SPIN_REJECTED');return
                self.result=self.handle.get_result_async();self.phase='SPIN';self.deadline=now+18.
            elif now>self.deadline:self.cancel('OBSERVATION_ACK_TIMEOUT')
            return
        if self.phase=='SPIN':
            if now>self.deadline:self.cancel('OBSERVATION_SPIN_TIMEOUT');return
            if self.result.done():
                response=self.result.result()
                if response.status!=4 or response.result.error_code!=0:self.finish(False,'OBSERVATION_SPIN_FAILED');return
                self.events.append(dict(requested_yaw_rad=self.plan[self.index],status='SUCCEEDED'))
                self.index+=1;self.handle=None;self.result=None;self.ack=None;self.cancel_ack=None
                if self.index==len(self.plan):self.phase='FINAL_SETTLE';self.deadline=now+3.
                else:self.phase='SETTLE';self.deadline=now+15.;self.stable=None
        elif self.phase=='FINAL_SETTLE' and now>=self.deadline:self.finish(True,'OBSERVATION_SWEEP_SUCCEEDED')

    def close(self):
        self.closed=True
        self.node.destroy_timer(self.timer);self.node.destroy_subscription(self.sub);self.client.destroy()
