import json,math,time,hashlib
from pathlib import Path as FilePath
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
OUT=FilePath("/home/user/team_serving_robot_clean_20261022/_local_only/wheel_0317_trial/forwardpath_verify_diag_01.jsonl")
if OUT.exists():raise SystemExit("Refusing overwrite")
class Monitor(Node):
 def __init__(self):
  super().__init__("costmap_failure_probe");self.global_map=None;self.local_map=None;self.pose=None;self.odom=None;self.path=None;self.path_hash=None;self.nav=None;self.file=OUT.open("x")
  self.create_subscription(Costmap,"/robot_02/global_costmap/costmap_raw",lambda m:setattr(self,"global_map",m),10)
  self.create_subscription(Costmap,"/robot_02/local_costmap/costmap_raw",lambda m:setattr(self,"local_map",m),10)
  self.create_subscription(PoseWithCovarianceStamped,"/robot_02/amcl_pose",lambda m:setattr(self,"pose",m),10)
  self.create_subscription(Odometry,"/robot_02/odom",lambda m:setattr(self,"odom",m),10)
  self.create_subscription(Path,"/robot_02/plan",self.on_path,10)
  self.create_subscription(Twist,"/robot_02/cmd_vel_nav",lambda m:setattr(self,"nav",m),10)
  self.create_timer(1.0,self.tick)
 def on_path(self,m):
  xy=[(round(p.pose.position.x,3),round(p.pose.position.y,3)) for p in m.poses]
  raw=json.dumps(xy).encode();self.path_hash=hashlib.sha256(raw).hexdigest()[:16]
  self.path={"hash":self.path_hash,"points":len(xy),"start":xy[:3],"end":xy[-3:],"length_m":round(sum(math.dist(a,b) for a,b in zip(xy,xy[1:])),3)}
  self.file.write(json.dumps({"event":"path_update","wall_time":time.time(),"path":self.path,"xy":xy})+"\n");self.file.flush()
 def cell(self,m,x,y):
  if m is None:return None
  z=m.metadata;o=z.origin.position;r=z.resolution
  if r<=0:return None
  ix=int(math.floor((x-o.x)/r));iy=int(math.floor((y-o.y)/r))
  if ix<0 or iy<0 or ix>=z.size_x or iy>=z.size_y:return {"outside":True}
  v=int(m.data[iy*z.size_x+ix])
  n=int(math.ceil(.5/r));vals=[]
  for yy in range(max(0,iy-n),min(z.size_y,iy+n+1)):
   for xx in range(max(0,ix-n),min(z.size_x,ix+n+1)):
    if math.hypot(xx-ix,yy-iy)*r<=.5:vals.append(int(m.data[yy*z.size_x+xx]))
  return {"cost":v,"near_lethal":sum(t>=253 for t in vals),"near_unknown":sum(t==255 for t in vals),"near_max":max(vals) if vals else None,"cell":[ix,iy]}
 def tick(self):
  if self.pose is None:return
  p=self.pose.pose.pose.position;q=self.pose.pose.pose.orientation
  yaw=2*math.atan2(q.z,q.w);d={"wall_time":time.time(),"sim_s":self.get_clock().now().nanoseconds*1e-9,"amcl":[p.x,p.y,yaw],"path":self.path,"nav":[self.nav.linear.x,self.nav.angular.z] if self.nav else None}
  g=self.global_map;l=self.local_map
  d["global"]={"robot":self.cell(g,p.x,p.y),"ahead":self.cell(g,p.x+.45*math.cos(yaw),p.y+.45*math.sin(yaw)),"goal":self.cell(g,1.6,-.15),"resolution":g.metadata.resolution if g else None}
  if self.odom:
   o=self.odom.pose.pose.position;d["odom"]=[o.x,o.y]
   d["local"]={"robot":self.cell(l,o.x,o.y),"ahead":self.cell(l,o.x+.45*math.cos(yaw),o.y+.45*math.sin(yaw)),"resolution":l.metadata.resolution if l else None}
  self.file.write(json.dumps(d)+"\n");self.file.flush()
rclpy.init();n=Monitor()
try:rclpy.spin(n)
finally:n.file.close();n.destroy_node();rclpy.shutdown()
