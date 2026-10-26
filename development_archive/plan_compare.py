"""Read-only Nav2 ComputePathToPose comparison; sends no movement goal."""
import argparse,json,math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

ROOT=Path(__file__).resolve().parent.parent
ap=argparse.ArgumentParser()
ap.add_argument('robot',choices=('robot_01','robot_02'))
ap.add_argument('destination')
args=ap.parse_args()
dest=yaml.safe_load((ROOT/'config/integration/waypoints.yaml').read_text())['destinations'][args.destination]
if 'approaches' in dest:dest=dest['approaches']['bottom']
rclpy.init()
node=Node('plan_compare_'+args.robot,namespace=args.robot)
node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
client=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
if not client.wait_for_server(timeout_sec=30):raise SystemExit('planner unavailable')
for planner in ('ForwardPath','GridBased'):
    goal=ComputePathToPose.Goal()
    goal.planner_id=planner;goal.use_start=False
    pose=PoseStamped()
    pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
    pose.pose.position.x=float(dest['x']);pose.pose.position.y=float(dest['y'])
    pose.pose.orientation.z=math.sin(float(dest['yaw'])/2)
    pose.pose.orientation.w=math.cos(float(dest['yaw'])/2)
    goal.goal=pose
    sent=client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node,sent,timeout_sec=30)
    handle=sent.result()
    if handle is None or not handle.accepted:
        print(json.dumps({'planner':planner,'accepted':False}),flush=True);continue
    done=handle.get_result_async()
    rclpy.spin_until_future_complete(node,done,timeout_sec=45)
    result=done.result()
    if result is None:
        handle.cancel_goal_async()
        print(json.dumps({'planner':planner,'timeout':True}),flush=True);continue
    path=result.result.path
    pts=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
    length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]))
    print(json.dumps({'planner':planner,'status':result.status,'error_code':result.result.error_code,
                      'length_m':length,'path':pts}),flush=True)
node.destroy_node();rclpy.shutdown()
