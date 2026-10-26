import json,math,time
from pathlib import Path
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer,TransformListener
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

ROOT=Path(__file__).resolve().parents[2]
im=Image.open(ROOT/"maps/stores/demo_store/team_lidar_initial.pgm")
w,h=im.size
OX,OY,RES=-.755,-.817,.05

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def grid(x,y):
 c=int((x-OX)/RES);r=h-1-int((y-OY)/RES)
 if c<0 or c>=w or r<0 or r>=h:return None
 return c,r
def score(pose,msg):
 if pose is None:return None
 x,y,a=pose;z={"occupied":0,"free":0,"unknown":0,"outside":0,"total":0}
 for k in range(0,len(msg.ranges),12):
  dist=msg.ranges[k]
  if not math.isfinite(dist) or dist<msg.range_min or dist>msg.range_max:continue
  angle=a+msg.angle_min+k*msg.angle_increment
  loc=grid(x+dist*math.cos(angle),y+dist*math.sin(angle))
  z["total"]+=1
  if loc is None:z["outside"]+=1;continue
  c,r=loc;v=im.getpixel((c,r))
  z["occupied" if v<100 else "free" if v>240 else "unknown"]+=1
 return z
class Record(Node):
 def __init__(self):
  super().__init__("scan_map_check")
  self.tf=Buffer(cache_time=Duration(seconds=10));self.listener=TransformListener(self.tf,self)
  self.gt=None;self.last=0
  self.path=ROOT/"_local_only/wheel_032_trial/scan_map_check.jsonl"
  self.f=self.path.open("w")
  self.create_subscription(TFMessage,"/cafe/gazebo_pose_truth",self.truth,10)
  self.create_subscription(LaserScan,"/robot_02/scan",self.scan,10)
 def truth(self,m):
  for t in m.transforms:
   if t.child_frame_id=="robot_02":
    p=t.transform.translation;a=yaw(t.transform.rotation)
    self.gt=(3.2+p.x,2.25+p.y,a,t.header.stamp.sec+t.header.stamp.nanosec*1e-9)
 def scan(self,m):
  now=time.monotonic()
  if now-self.last<1 or self.gt is None:return
  self.last=now;st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  try:
   t=self.tf.lookup_transform("map","robot_02/base_footprint",Time.from_msg(m.header.stamp),timeout=Duration(seconds=.05))
   p=t.transform.translation;tf=(p.x,p.y,yaw(t.transform.rotation))
  except Exception:tf=None
  gt=self.gt[:3];out={"scan_sim_s":st,"gt_sim_s":self.gt[3],"gt":gt,"tf":tf,"gt_score":score(gt,m),"tf_score":score(tf,m)}
  self.f.write(json.dumps(out)+"\n");self.f.flush()
def main():
 rclpy.init();n=Record()
 try:rclpy.spin(n)
 finally:n.f.close();n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
