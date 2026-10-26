"""Explicit virtual-person scenarios for semantic safety testing.
No fabricated camera detections: ROS virtual people are test inputs, not YOLO detections.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

class People(Node):
    def __init__(self):
        super().__init__('people_simulator')
        self.declare_parameter('enabled',False)
        self.declare_parameter('count',0)
        self.pub=self.create_publisher(PoseArray,'/cafe/people_ground_truth',10)
        self.create_subscription(String,'/cafe/people_command',self.command,10)
        self.count=min(8,max(0,int(self.get_parameter('count').value))) if self.get_parameter('enabled').value else 0
        self.elapsed=0.;self.freeze=False;self.active=bool(self.get_parameter('enabled').value)
        self.create_timer(0.1,self.tick)
    def command(self,msg):
        parts=msg.data.strip().split()
        self.active=True
        if not parts:return
        if parts[0]=='count' and len(parts)>1:
            try:self.count=max(0,min(8,int(parts[1])))
            except ValueError:pass
        elif parts[0]=='freeze':self.freeze=True
        elif parts[0]=='move':self.freeze=False
        elif parts[0]=='clear':self.count=0
    def tick(self):
        if not self.active:return
        if not self.freeze:self.elapsed+=0.1
        p=PoseArray();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
        for i in range(self.count):
            pose=Pose()
            pose.position.x=(i%4-1.5)*1.8+0.28*math.sin(self.elapsed*0.45+i)
            pose.position.y=(i//4-0.5)*2.7+0.4*math.cos(self.elapsed*0.35+i)
            pose.orientation.w=1.
            p.poses.append(pose)
        self.pub.publish(p)

def main():
    rclpy.init();n=People()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
