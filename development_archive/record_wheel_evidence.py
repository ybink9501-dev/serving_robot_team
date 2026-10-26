"""Read-only evidence: wheel joints, Gazebo model pose and odometry at sim time."""
import csv
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'metrics'/('wheel_compare_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')
FIELDS=('wall_s','wheel_sim_s','left_rad','right_rad','gt_sim_s','gt_x','gt_y',
        'gt_yaw','odom_sim_s','odom_x','odom_y','odom_yaw')

def sec(stamp):
    return stamp.sec+stamp.nanosec*1e-9

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Evidence(Node):
    def __init__(self):
        super().__init__('robot02_wheel_evidence')
        OUT.parent.mkdir(parents=True,exist_ok=True)
        self.file=OUT.open('w',newline='')
        self.writer=csv.DictWriter(self.file,fieldnames=FIELDS)
        self.writer.writeheader()
        self.wheel=None;self.gt=None;self.odom=None
        self.create_subscription(JointState,'/robot_02/joint_states',self.on_wheel,10)
        self.create_subscription(TFMessage,'/cafe/gazebo_pose_truth',self.on_gt,10)
        self.create_subscription(Odometry,'/robot_02/odom',self.on_odom,10)
        self.create_timer(0.1,self.tick)
        print(OUT,flush=True)
    def on_wheel(self,msg):
        data=dict(zip(msg.name,msg.position))
        if 'left_wheel_joint' in data and 'right_wheel_joint' in data:
            self.wheel=(sec(msg.header.stamp),data['left_wheel_joint'],data['right_wheel_joint'])
    def on_gt(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='robot_02':
                p=t.transform.translation
                self.gt=(sec(t.header.stamp),p.x,p.y,yaw(t.transform.rotation))
    def on_odom(self,msg):
        p=msg.pose.pose.position
        self.odom=(sec(msg.header.stamp),p.x,p.y,yaw(msg.pose.pose.orientation))
    def tick(self):
        if self.wheel is None or self.gt is None or self.odom is None:return
        row=dict(zip(FIELDS,(time.time(),*self.wheel,*self.gt,*self.odom)))
        self.writer.writerow(row)
        self.file.flush()
    def destroy_node(self):
        self.file.close()
        return super().destroy_node()
def main():
    rclpy.init()
    node=Evidence()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
