#!/usr/bin/env python3
"""Publish a Fleet request and wait for matching /fleet/reply (not publish-only success)."""
import sys, json, uuid, time, argparse

def build(argv):
    p=argparse.ArgumentParser(description='Send cafe ROS2 Fleet command; confirmation required')
    sub=p.add_subparsers(dest='operation',required=True)
    for op in ('pickup','direct','collect'):
        z=sub.add_parser(op);z.add_argument('robot',choices=['robot_01','robot_02'])
        z.add_argument('table',choices=[f'table_{x:02}' for x in range(1,5)])
    for op in ('load','complete','collected','cancel'):
        z=sub.add_parser(op);z.add_argument('request_id')
    a=p.parse_args(argv)
    if a.operation in ('pickup','direct','collect'):
        action={'pickup':'KITCHEN_PICKUP','direct':'DELIVER_DIRECT','collect':'TABLE_COLLECT'}[a.operation]
        return {'action':'SUBMIT','request_id':'test-'+uuid.uuid4().hex[:12],
                'robot_id':a.robot,'table':a.table,'mode':action,'source':'operator'}
    return {'action':a.operation.upper(),'request_id':a.request_id,
            'source':'kitchen' if a.operation=='load' else 'table' if a.operation in ('complete','collected') else 'operator'}

def main():
    command=build(sys.argv[1:])
    import rclpy
    from std_msgs.msg import String
    from rclpy.node import Node
    rclpy.init(); n=Node('fleet_cli_'+uuid.uuid4().hex[:8]);reply=[]
    def cb(msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        if d.get('request_id')==command['request_id'] and d.get('action')==command['action']:reply.append(d)
    subscription=n.create_subscription(String,'/fleet/reply',cb,10)
    pub=n.create_publisher(String,'/fleet/request',10)
    deadline=time.monotonic()+8
    try:
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.2)
        if pub.get_subscription_count()==0:
            print('FAIL: no Fleet subscriber; no command delivered',file=sys.stderr);return 2
        msg=String(data=json.dumps(command));pub.publish(msg)
        while not reply and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.2)
        if not reply:
            print('FAIL: Fleet acknowledgment timed out. DO NOT assume success. REQUEST_ID='+command['request_id']);return 3
        print(json.dumps(reply[0],ensure_ascii=False,indent=2))
        return 0 if reply[0].get('ok') else 4
    finally:
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
