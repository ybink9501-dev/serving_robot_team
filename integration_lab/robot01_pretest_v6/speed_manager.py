#!/usr/bin/env python3
"""Robot 01-only runtime profile manager; NEVER sends cmd_vel or navigation goals.
No speed limit is published on startup; an explicit CLI request is required.
"""
import json
import time
from checks import check,read_yaml,expand,HERE,speed_config_ok

def main():
    import rclpy
    from rclpy.node import Node
    from nav2_msgs.msg import SpeedLimit
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    cfg_file=HERE/'config.yaml'
    class Speed(Node):
        def __init__(self):
            super().__init__('robot01_speed_manager_v6')
            self.pub_status=self.create_publisher(String,'/lab/robot01/drive_profile_status',10)
            self.create_subscription(String,'/lab/robot01/drive_profile_request',self.request,10)
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            c=read_yaml(cfg_file)
            other=self.get_publishers_info_by_topic(c['speed']['topic'])
            if other:
                raise RuntimeError('Another speed-limit publisher already exists: '+
                                   ', '.join(x.node_name for x in other))
            self.pub_limit=self.create_publisher(SpeedLimit,c['speed']['topic'],qos)
            self.pending=[]
            self.create_timer(.1,self.tick)
        def reply(self, state,request_id=None,**kw):
            msg={'state':state,'request_id':request_id,'robot_id':'robot_01',**kw}
            self.pub_status.publish(String(data=json.dumps(msg,ensure_ascii=False)))
            self.get_logger().info(json.dumps(msg,ensure_ascii=False))
        def request(self,m):
            request_id=None
            try:
                d=json.loads(m.data);request_id=d.get('request_id')
                if d.get('robot_id')!='robot_01' or d.get('command')!='SET_PROFILE':
                    raise ValueError('Robot 01 SET_PROFILE only')
                c=read_yaml(cfg_file)
                profiles=speed_config_ok(c,read_yaml(expand(c['runtime_limits_yaml'])))
                name=d.get('profile')
                if name not in profiles:raise ValueError('Unknown speed profile')
                if not isinstance(request_id,str) or not request_id.strip():
                    raise ValueError('Missing request_id')
                self.pending.append((time.monotonic()+6,request_id,name,profiles[name]))
                self.tick()
            except Exception as e:self.reply('REJECT',request_id,reason=str(e))
        def tick(self):
            if not self.pending:return
            remaining=[]
            for deadline,req,name,value in self.pending:
                if self.pub_limit.get_subscription_count()>0:
                    out=SpeedLimit();out.percentage=False;out.speed_limit=float(value)
                    self.pub_limit.publish(out)
                    self.reply('PUBLISHED',req,profile=name,speed_limit_mps=value,
                               notice='Published to Nav2, physical odom speed not yet verified')
                elif time.monotonic()>deadline:
                    self.reply('REJECT_CONTROLLER_OFFLINE',req)
                else:remaining.append((deadline,req,name,value))
            self.pending=remaining
    rclpy.init();n=Speed()
    try:rclpy.spin(n)
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
