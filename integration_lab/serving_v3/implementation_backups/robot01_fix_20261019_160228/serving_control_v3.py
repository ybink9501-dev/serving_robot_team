#!/usr/bin/env python3
"""Control LAB serving flow. No movement unless 'start' is explicitly called."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',default='robot_01')
    sp=p.add_subparsers(dest='cmd',required=True)
    start=sp.add_parser('start');start.add_argument('table',choices=[f'table_{i:02d}' for i in range(1,5)])
    start.add_argument('--side',choices=['auto','top','bottom','left','right'],default='auto')
    direct=sp.add_parser('direct');direct.add_argument('table',choices=[f'table_{i:02d}' for i in range(1,5)]);direct.add_argument('--side',choices=['auto','top','bottom','left','right'],default='auto')
    for item in ['load','complete','cancel','status']:sp.add_parser(item)
    a=p.parse_args();rclpy.init();node=Node('serving_control_v3_client')
    try:
        if a.cmd=='status':
            msgs=[];sub=node.create_subscription(String,f'/{a.robot}/serving/status',lambda m:msgs.append(m.data),10)
            until=time.monotonic()+5
            while not msgs and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
            if not msgs:raise RuntimeError('No active serving supervisor')
            print(json.dumps(json.loads(msgs[-1]),ensure_ascii=False,indent=2));return
        channel=f'/{a.robot}/serving/complete' if a.cmd in ('load','complete') else f'/{a.robot}/serving/request'
        msg={'command':('DIRECT' if a.cmd=='direct' else 'START'),'table':a.table,'side':a.side} if a.cmd in ('start','direct') else {
            'load':{'command':'CONFIRM_LOAD'},'complete':{'command':'COMPLETE'},'cancel':{'command':'CANCEL'}}[a.cmd]
        pub=node.create_publisher(String,channel,10)
        deadline=time.monotonic()+5
        while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not pub.get_subscription_count():raise RuntimeError('Supervisor subscriber not connected: '+channel)
        pub.publish(String(data=json.dumps(msg)));end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        print('PUBLISHED',channel,msg,'(not proof of completion)')
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    try:main()
    except RuntimeError as e:raise SystemExit('ERROR: '+str(e))
