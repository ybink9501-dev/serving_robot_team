"""Publish robot-specific initial map poses until AMCL starts publishing."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Initializer(Node):
    def __init__(self):
        super().__init__('pose_initializer')
        for k,v in [('x',-3.2),('y',-2.25),('yaw',0.0)]:self.declare_parameter(k,v)
        self.pub=self.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        self.received=False
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.on_amcl,10)
        self.create_timer(2.0,self.tick)
    def on_amcl(self,msg):self.received=True
    def tick(self):
        if self.received:return
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.pose.position.x=self.get_parameter('x').value
        m.pose.pose.position.y=self.get_parameter('y').value
        a=self.get_parameter('yaw').value
        m.pose.pose.orientation.z=math.sin(a/2);m.pose.pose.orientation.w=math.cos(a/2)
        m.pose.covariance[0]=0.005;m.pose.covariance[7]=0.005;m.pose.covariance[35]=0.03
        self.pub.publish(m)

def main():
    rclpy.init();n=Initializer()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
