#!/usr/bin/env python3
"""Actual ROS/Gazebo readiness acceptance: requires fresh publishers, active Nav2 and simulator entities.
Returns nonzero on missing evidence. Do not use offline checks as a runtime pass.
"""
import argparse
import json
import re
import subprocess
import sys
import time

ROBOTS=('robot_01','robot_02')
NODES=('map_server','amcl','planner_server','controller_server','bt_navigator','collision_monitor','velocity_smoother')

def cmd(*args,timeout=6):
    try:
        p=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
        return p.returncode,(p.stdout or '')+(p.stderr or '')
    except (subprocess.TimeoutExpired,FileNotFoundError) as e:return 99,str(e)

def check():
    errors=[];result={}
    # One Gazebo world, both entities and no hidden second server.
    rc,proc=cmd('pgrep','-af',r'^gz sim server($|[[:space:]])',timeout=2)
    # Gazebo Harmonic starts a 'gz sim server' process and often a separate GUI.
    # The ruby launch wrapper and 'gz sim gui' are NOT additional servers.
    servers=[line for line in proc.splitlines()
             if re.match(r'^\s*\d+\s+gz sim server(?:\s|$)',line)]
    result['gazebo_server_count']=len(servers)
    if len(servers)!=1:errors.append('Expected exactly one gz sim server')
    rc,models=cmd('gz','model','--list',timeout=12)
    result['models']=[r for r in ROBOTS if re.search(r'\b'+r+r'\b',models)]
    for r in ROBOTS:
        if r not in result['models']:errors.append(r+' model missing from Gazebo')
    for r in ROBOTS:
        for node in NODES:
            rc,out=cmd('ros2','lifecycle','get',f'/{r}/{node}',timeout=5)
            result[f'{r}/{node}']=out.strip()[:80]
            if rc!=0 or 'active' not in out.lower():errors.append(f'{r}/{node} not active')
        for topic in ('scan','odom','serving/status','safety_status','cmd_vel'):
            rc,out=cmd('ros2','topic','info',f'/{r}/{topic}',timeout=5)
            match=re.search(r'Publisher count:\s*(\d+)',out)
            publishers=int(match.group(1)) if match else 0
            result[f'{r}/{topic}/publishers']=publishers
            if rc or publishers<1:errors.append(f'{r}/{topic} lacks publisher')
        # Fresh actual messages, not mere topic-list existence.
        for topic in ('scan','odom','serving/status','safety_status'):
            rc,out=cmd('ros2','topic','echo',f'/{r}/{topic}','--once',timeout=8)
            if rc or not out.strip():errors.append(f'{r}/{topic} no live sample')
        rc,out=cmd('ros2','action','info',f'/{r}/navigate_to_pose',timeout=5)
        if rc or 'Action servers: 1' not in out:errors.append(r+' Nav2 action server missing')
    rc,out=cmd('ros2','topic','echo','/fleet/status','--once',timeout=8)
    if rc or not out.strip():errors.append('Fleet no live status')
    rc,out=cmd('ros2','topic','info','/cafe/gazebo_pose_truth',timeout=5)
    match=re.search(r'Publisher count:\s*(\d+)',out)
    if not match or int(match.group(1))<1:errors.append('Gazebo truth pose bridge missing')
    result['errors']=errors
    result['pass']=not errors
    return result

if __name__=='__main__':
    r=check();print(json.dumps(r,ensure_ascii=False,indent=2))
    sys.exit(0 if r['pass'] else 1)
