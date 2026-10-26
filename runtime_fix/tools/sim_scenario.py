#!/usr/bin/env python3
"""Physical Gazebo pedestrian/obstacle scenarios for simulation-owned validation.
Gazebo entities, rather than ROS reference poses, must provide LiDAR/camera evidence.
Truth topic is /cafe/people_ground_truth, NOT the detector output.
"""
import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
MODELS=ROOT/'ros_ws/src/cafe_autonomy/models'
WORLD='cafe'

SCENARIOS={
  'crossing':[(2.0,-1.7),(0.0,-1.7),(-2.0,-1.7)],
  'stationary':[(-2.1,-0.6)]*3,
  'reenter':[(1.9,1.4),(1.2,1.4),(1.9,1.4)],
  'occlusion':[(3.1,0.7),(1.75,1.65),(-0.35,1.65)],
  'block':[(0.0,-0.5)]*3,
  'release':[(0.0,-0.5),(0.0,-0.5),(4.0,2.0)],
  'dynamic_obstacle':[(2.0,0.1),(0.0,0.1),(-2.0,0.1)],
}

def gz_service(name,req,reqtype,reptype='gz.msgs.Boolean',timeout=8):
    cmd=['gz','service','-s',f'/world/{WORLD}/{name}','--reqtype',reqtype,
         '--reptype',reptype,'--timeout','5000','--req',req]
    try:
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
    except subprocess.TimeoutExpired as exc:raise RuntimeError('Gazebo '+name+' timeout') from exc
    if result.returncode or not re.search(r'\bdata\s*:\s*true\b',result.stdout,re.I):
        raise RuntimeError('Gazebo '+name+' rejected: '+(result.stdout+result.stderr)[-900:])

def spawn(name,model_file,x,y):
    req=f'sdf_filename: "{model_file}" name: "{name}" pose: {{position: {{x:{x} y:{y} z:0}} orientation: {{w:1}}}}'
    gz_service('create',req,'gz.msgs.EntityFactory')

def move(name,x,y):
    gz_service('set_pose',f'name:"{name}" position:{{x:{x} y:{y} z:0}} orientation:{{w:1}}', 'gz.msgs.Pose')

def remove(name):
    gz_service('remove',f'name:"{name}" type: MODEL','gz.msgs.Entity')

def main():
    pa=argparse.ArgumentParser(description='Gazebo PHYSICAL scenarios (not fake YOLO detections)')
    pa.add_argument('scenario',choices=tuple(SCENARIOS)+('clear','two_robots'))
    pa.add_argument('--period',type=float,default=4.0,help='seconds per path segment')
    pa.add_argument('--people',type=int,default=1,help='1..8 person entities')
    pa.add_argument('--inject-detector-test',action='store_true',help='test-only ground-truth injection into detector topic; never use for perception evaluation')
    pa.add_argument('--dry-run',action='store_true')
    a=pa.parse_args()
    if a.scenario=='two_robots':
        print('Two robot scenario uses both ROS missions; run trial.py for robot_01 and robot_02 concurrently.');return
    if a.scenario=='clear':
        for i in range(1,9):
            for pre in ('test_person_','test_obstacle_'):
                try:
                    if not a.dry_run:remove(f'{pre}{i:02d}')
                except RuntimeError:pass
        print('Scenario objects removed');return
    if not 1<=a.people<=8:pa.error('--people must be 1..8')
    if not 0.5<=a.period<=600:pa.error('--period must be in 0.5..600')
    points=SCENARIOS[a.scenario]
    is_person=a.scenario!='dynamic_obstacle'
    name_prefix='test_person_' if is_person else 'test_obstacle_'
    model=MODELS/('person.sdf' if is_person else 'movable_obstacle.sdf')
    if a.dry_run:
        print(json.dumps({'scenario':a.scenario,'entity_count':a.people,'waypoints':points,
             'model':str(model),'ground_truth_only':not a.inject_detector_test},ensure_ascii=False,indent=2));return
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Pose,PoseArray
        if a.inject_detector_test:
            print('WARNING: synthetic perception injection ON; cannot certify detector performance',file=sys.stderr)
        rclpy.init();node=Node('physical_scenario_truth')
        truth=node.create_publisher(PoseArray,'/cafe/people_ground_truth',10)
        detected=node.create_publisher(PoseArray,'/cafe/people_detected',10) if a.inject_detector_test else None
        def report(points_now):
            m=PoseArray();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
            for x,y in points_now:
                p=Pose();p.position.x=float(x);p.position.y=float(y);p.orientation.w=1.0
                m.poses.append(p)
            truth.publish(m)
            if detected:detected.publish(m)
    except ImportError as exc:raise SystemExit('ROS 2 environment not sourced: '+str(exc))
    names=[]
    try:
        for i in range(a.people):
            name=name_prefix+f'{i+1:02d}'
            pos=(points[0][0],points[0][1]+i*.60)
            spawn(name,model,*pos);names.append(name)
        # Real collision objects move by Gazebo set_pose; ROS topic is only an external reference.
        for idx,(tx,ty) in enumerate(points):
            starts=[(points[max(0,idx-1)][0],points[max(0,idx-1)][1]+i*.6) for i in range(a.people)]
            steps=max(1,int(a.period/0.2))
            for k in range(steps):
                alpha=(k+1)/steps
                loc=[(x+(tx-x)*alpha,y+(ty+i*.6-y)*alpha) for i,(x,y) in enumerate(starts)]
                for name,(x,y) in zip(names,loc):move(name,round(x,4),round(y,4))
                report(loc if is_person else [])
                time.sleep(a.period/steps)
            print(f'[{a.scenario}] waypoint {idx+1}/{len(points)}')
    finally:
        try:
            report([])
            node.destroy_node();rclpy.shutdown()
        except Exception:pass
        for name in names:
            try:remove(name)
            except RuntimeError as exc:print('WARNING cleanup:',exc,file=sys.stderr)

if __name__=='__main__':main()
