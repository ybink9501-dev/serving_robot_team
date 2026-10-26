#!/usr/bin/env python3
"""LAB serving flow adapter. Keeps the existing destination_server unchanged.

Input:
  /robot_01/serving/request  std_msgs/String JSON {"command":"START","table":"table_01"}
  /robot_01/serving/complete std_msgs/String JSON {"command":"COMPLETE"}
Output:
  /robot_01/serving/status   std_msgs/String JSON {"state":...,"table":...,"mode":...}
Sends existing GO/CANCEL on /robot_01/destination_command.

DOES NOT bypass destination_server's Nav2+physical-arrival (1mm) gating.
"""
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
from serving_flow import ServingFlow


class ServingSupervisor(Node):
    def __init__(self):
        super().__init__('serving_supervisor_lab')
        robot = str(self.declare_parameter('robot_id', 'robot_01').value)
        mode = str(self.declare_parameter('completion_mode', 'manual').value)
        wait = float(self.declare_parameter('auto_wait_s', 10.0).value)
        self.revision = int(self.declare_parameter('map_revision', 1).value)
        self.kitchen = str(self.declare_parameter('kitchen_destination', 'kitchen').value)
        waypoint_path = str(self.declare_parameter('waypoints_file', '').value)
        if not waypoint_path or not Path(waypoint_path).is_file():
            raise RuntimeError('Explicit, verified waypoints_file required')
        obj = yaml.safe_load(Path(waypoint_path).read_text(encoding='utf-8'))
        self.valid_destinations = {w.get('entity_id') for w in obj.get('waypoints', [])}
        if self.kitchen not in self.valid_destinations:
            raise RuntimeError(f'Kitchen waypoint {self.kitchen!r} missing; no driving allowed')
        if obj.get('map_revision') != self.revision:
            raise RuntimeError('map_revision mismatch with waypoint file')
        self.flow = ServingFlow(kitchen=self.kitchen, mode=mode, auto_wait_s=wait)
        self.robot = robot
        self.pub_cmd = self.create_publisher(String, f'/{robot}/destination_command', 10)
        self.pub_status = self.create_publisher(String, f'/{robot}/serving/status', 10)
        self.sub_req = self.create_subscription(String, f'/{robot}/serving/request', self.request_cb, 10)
        self.sub_complete = self.create_subscription(String, f'/{robot}/serving/complete', self.complete_cb, 10)
        self.sub_state = self.create_subscription(String, f'/{robot}/robot_state', self.state_cb, 10)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.heartbeat = self.create_timer(1.0, self.status)
        self.status('READY')

    def status(self, reason=None):
        self.pub_status.publish(String(data=json.dumps({
            'robot_id': self.robot, 'state': self.flow.state,
            'table': self.flow.table, 'completion_mode': self.flow.mode,
            'reason': reason if reason is not None else self.flow.reason,
            'time_monotonic_s': round(time.monotonic(), 3)
        })))

    def apply(self, actions):
        for action in actions:
            kind = action[0]
            if kind == 'STATUS':
                self.status(action[3])
            elif kind in ('GO', 'CANCEL'):
                if self.pub_cmd.get_subscription_count() == 0:
                    self.flow.state = 'ERROR'
                    self.status('DESTINATION_SERVER_NOT_CONNECTED')
                    return
                command = ({'command':'GO', 'destination_id':action[1],
                            'map_revision':self.revision} if kind == 'GO'
                           else {'command':'CANCEL'})
                self.pub_cmd.publish(String(data=json.dumps(command)))

    def request_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('command')
            if action == 'CANCEL':
                self.apply(self.flow.cancel())
            elif action == 'START':
                table = cmd.get('table')
                if table not in self.valid_destinations or table == self.kitchen:
                    self.status('REJECTED_UNKNOWN_TABLE')
                    return
                if self.pub_cmd.get_subscription_count() == 0:
                    self.status('REJECTED_DESTINATION_SERVER_OFFLINE')
                    return
                self.apply(self.flow.start(table))
            else:
                self.status('REJECTED_UNKNOWN_COMMAND')
        except Exception as exc:
            self.status(f'REJECTED_BAD_MESSAGE:{exc}')

    def complete_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            if cmd.get('command') != 'COMPLETE':
                self.status('REJECTED_EXPECT_COMPLETE')
                return
            self.apply(self.flow.complete())
        except Exception as exc:
            self.status(f'REJECTED_BAD_MESSAGE:{exc}')

    def state_cb(self, msg):
        try:
            obj = json.loads(msg.data)
            if obj.get('robot_id') != self.robot:
                return
            self.apply(self.flow.robot_state(obj.get('state'), obj.get('destination_id')))
        except Exception as exc:
            self.status(f'BAD_ROBOT_STATE:{exc}')

    def timer_cb(self):
        self.apply(self.flow.tick())


def main():
    rclpy.init()
    node = None
    try:
        node = ServingSupervisor()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
