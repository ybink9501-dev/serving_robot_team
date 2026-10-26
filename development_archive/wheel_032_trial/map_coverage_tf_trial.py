"""Isolated diagnostic: compare map-observed AMCL with odom in unknown cells."""
import argparse,math,json,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from tf2_ros import TransformBroadcaster

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def quat(a):return (math.sin(a/2),math.cos(a/2))
class Trial(Node):
 def __init__(self,a):
  super().__init__("map_coverage_tf_trial_"+a.robot)
  self.set_parameters([Parameter("use_sim_time",Parameter.Type.BOOL,True)])
  self.robot=a.robot;self.tx=a.x;self.ty=a.y;self.da=a.yaw
  self.map=None;self.odom=None;self.last_cov=None;self.last_accept=None
  self.b=TransformBroadcaster(self)
  q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.create_subscription(OccupancyGrid,"/"+a.robot+"/map",self.on_map,q)
  self.create_subscription(Odometry,"/"+a.robot+"/odom",self.on_odom,10)
  self.create_subscription(PoseWithCovarianceStamped,"/"+a.robot+"/amcl_pose",self.on_amcl,10)
  self.create_timer(.1,self.publish)
 def on_map(self,m):self.map=m
 def on_odom(self,m):self.odom=(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))
 def coverage(self,x,y):
  if self.map is None:return None
  info=self.map.info;res=info.resolution
  cx=int((x-info.origin.position.x)/res);cy=int((y-info.origin.position.y)/res)
  rad=max(1,int(.75/res));known=total=0
  for row in range(cy-rad,cy+rad+1,2):
   if row<0 or row>=info.height:continue
   for col in range(cx-rad,cx+rad+1,2):
    if col<0 or col>=info.width or (col-cx)**2+(row-cy)**2>rad*rad:continue
    total+=1
    if self.map.data[row*info.width+col]>=0:known+=1
  return known/total if total else 0.
 def on_amcl(self,m):
  if self.odom is None or self.map is None:return
  ox,oy,oa=self.odom
  px=self.tx+math.cos(self.da)*ox-math.sin(self.da)*oy
  py=self.ty+math.sin(self.da)*ox+math.cos(self.da)*oy
  cov=self.coverage(px,py);self.last_cov=cov
  if cov is None or cov<.2:return
  mx,my=m.pose.pose.position.x,m.pose.pose.position.y
  d=yaw(m.pose.pose.orientation)-oa
  self.da=d;self.tx=mx-(math.cos(d)*ox-math.sin(d)*oy);self.ty=my-(math.sin(d)*ox+math.cos(d)*oy)
  self.last_accept=time.monotonic()
 def publish(self):
  t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id="map";t.child_frame_id=self.robot+"/odom"
  t.transform.translation.x=self.tx;t.transform.translation.y=self.ty
  t.transform.rotation.z,t.transform.rotation.w=quat(self.da)
  self.b.sendTransform(t)
  if int(time.monotonic()*2)%10==0:
   self.get_logger().info(json.dumps({"coverage":self.last_cov,"tx":self.tx,"ty":self.ty,"yaw":self.da,"accepted_recently":self.last_accept is not None and time.monotonic()-self.last_accept<2}))
def main():
 p=argparse.ArgumentParser();p.add_argument("--robot",required=True);p.add_argument("--x",type=float,required=True);p.add_argument("--y",type=float,required=True);p.add_argument("--yaw",type=float,required=True)
 a=p.parse_args();rclpy.init();n=Trial(a)
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
