"""Fail-closed semantic velocity gate, then Collision Monitor, then Gazebo."""
import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from .policy import velocity_limit, person_factor
from .runtime_config import load_limits

class Safety(Node):
    def __init__(self):
        super().__init__('semantic_safety')
        self.declare_parameter('robot_id','robot_01')
        self.robot=self.get_parameter('robot_id').value
        self.declare_parameter('runtime_limits_file','')
        self.limits=load_limits(self.get_parameter('runtime_limits_file').value)
        self.safe_cfg=self.limits['safety'];self.nav_cfg=self.limits['navigation']
        self.cmd_nav=Twist();self.cmd_fine=Twist()
        self.nav_stamp=0;self.fine_stamp=0;self.scan_stamp=0
        self.fine=False;self.hold=True;self.people=[];self.xy=None
        self.person_stationary_start={}
        self.output=self.create_publisher(Twist,'cmd_vel_semantic',10)
        self.status=self.create_publisher(String,'safety_status',10)
        self.create_subscription(Twist,'cmd_vel_nav_smooth',self.nav,10)
        self.create_subscription(Twist,'cmd_vel_fine',self.fine_cmd,10)
        self.create_subscription(Bool,'fine_active',self.fine_switch,10)
        self.create_subscription(Bool,'traffic_hold',self.set_hold,10)
        self.create_subscription(LaserScan,'scan',self.on_scan,10)
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.on_amcl,10)
        self.create_subscription(PoseArray,'/cafe/people_detected',self.on_people,10)
        self.replan_pub=self.create_publisher(String,'person_replan_event',10)
        self.people_last={};self.people_stationary={};self.replan_fired=set();self.people_stamp=0.0
        self.create_timer(0.05,self.tick)

    def nav(self,msg):self.cmd_nav=msg;self.nav_stamp=time.monotonic()
    def fine_cmd(self,msg):self.cmd_fine=msg;self.fine_stamp=time.monotonic()
    def fine_switch(self,msg):self.fine=msg.data
    def set_hold(self,msg):self.hold=msg.data
    def on_scan(self,msg):
        # A finite, plausible scan proves sensor freshness; polygon-level safety is
        # handled separately by Nav2 Collision Monitor. Do not call a wall a person.
        self.scan_stamp=time.monotonic()
    def on_amcl(self,msg):
        p=msg.pose.pose.position;self.xy=(p.x,p.y)
    def on_people(self,msg):
        # The detector contract uses global map poses, never untransformed camera coordinates.
        if msg.header.frame_id!='map':
            self.get_logger().warning('Ignored people detections without map-frame coordinates')
            return
        now=time.monotonic()
        poses=[(p.position.x,p.position.y) for p in msg.poses]
        for i,p in enumerate(poses):
            prev=self.people_last.get(i)
            if prev is None or math.hypot(p[0]-prev[0],p[1]-prev[1]) > 0.08:
                self.people_stationary[i]=now
                self.replan_fired.discard(i)
            self.people_last[i]=p
        for i in list(self.people_last):
            if i>=len(poses):
                self.people_last.pop(i,None);self.people_stationary.pop(i,None)
                self.replan_fired.discard(i)
        self.people=poses
        self.people_stamp=now

    def tick(self):
        now=time.monotonic();fresh=now-self.scan_stamp<self.safe_cfg['scan_timeout_s']
        source_stamp=self.fine_stamp if self.fine else self.nav_stamp
        cmd=self.cmd_fine if self.fine else self.cmd_nav
        valid=fresh and not self.hold and now-source_stamp<self.safe_cfg['command_timeout_s']
        result=Twist();dmin=None
        if self.xy and self.people:
            dmin=min(math.hypot(self.xy[0]-p[0],self.xy[1]-p[1]) for p in self.people)
        # Detector data must also be fresh. The LiDAR collision monitor remains downstream.
        if now-self.people_stamp>1.0:
            dmin=None;self.people=[];self.people_last.clear();self.people_stationary.clear();self.replan_fired.clear()
        fac=person_factor(dmin,self.safe_cfg['person_stop_m'],self.safe_cfg['person_slow_m'])
        # Replanning is requested once per stationary blocking episode after 20 seconds.
        if dmin is not None and dmin<=self.safe_cfg['person_slow_m'] and self.xy:
            for i,point in enumerate(self.people):
                if i in self.replan_fired:continue
                if math.hypot(self.xy[0]-point[0],self.xy[1]-point[1])<=self.safe_cfg['person_slow_m'] and now-self.people_stationary.get(i,now)>=self.safe_cfg['stationary_replan_s']:
                    self.replan_pub.publish(String(data=json.dumps({'event':'STATIONARY_PERSON_REPLAN',
                      'robot_id':self.robot,'person_index':i,'distance_m':dmin})))
                    self.replan_fired.add(i)
        if valid:
            v,w=velocity_limit(cmd.linear.x,cmd.angular.z,self.nav_cfg['max_forward_mps'],self.nav_cfg['max_yaw_rps'])
            result.linear.x=v*fac
            result.angular.z=w if fac>0 else 0.0
        self.output.publish(result)
        message={'robot_id':self.robot,'sensor_fresh':fresh,'traffic_hold':self.hold,
                 'person_distance_m':dmin,'state':('STOP' if not valid or fac==0 else
                 'SLOW' if fac<1 else 'NORMAL'),'fine':self.fine}
        self.status.publish(String(data=json.dumps(message)))

def main():
    rclpy.init();n=Safety()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
