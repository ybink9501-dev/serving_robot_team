#!/usr/bin/env python3
"""Explicit Robot 01 speed request, not a movement command."""
import argparse
import json
import time
import uuid
from checks import check,HERE

def main():
    p=argparse.ArgumentParser();p.add_argument('profile',choices=['low','normal','high']);args=p.parse_args()
    outcome=check()
    if not outcome['ready']:
        print(json.dumps(outcome,ensure_ascii=False,indent=2));return 2
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    req_id=uuid.uuid4().hex
    rclpy.init();n=Node('robot01_speed_profile_cli_v6');results=[]
    pub=n.create_publisher(String,'/lab/robot01/drive_profile_request',10)
    n.create_subscription(String,'/lab/robot01/drive_profile_status',lambda m:results.append(m.data),10)
    try:
        until=time.monotonic()+9
        while pub.get_subscription_count()==0 and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
        if pub.get_subscription_count()==0:print('ERROR: Robot 01 speed manager is not running');return 3
        pub.publish(String(data=json.dumps({'command':'SET_PROFILE','robot_id':'robot_01',
            'request_id':req_id,'profile':args.profile})))
        while time.monotonic()<until:
            rclpy.spin_once(n,timeout_sec=.1)
            for raw in results:
                try:
                    obj=json.loads(raw)
                    if obj.get('request_id')==req_id:
                        print(json.dumps(obj,ensure_ascii=False,indent=2))
                        return 0 if obj['state']=='PUBLISHED' else 4
                except Exception:pass
            results.clear()
        print('TIMEOUT: no matching acknowledgement; inspect manager log');return 5
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
