#!/usr/bin/env python3
"""Read-only check of Robot 01, person simulator, and possible Robot 02 separation.
Does not send motion commands, spawn robots, or change node parameters.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait', type=float, default=3.0)
    args = parser.parse_args()
    rclpy.init()
    node=Node('cafe_lab_readiness', parameter_overrides=[Parameter('use_sim_time',value=True)],
              automatically_declare_parameters_from_overrides=True)
    buf=Buffer()
    listener=TransformListener(buf,node)
    try:
        end=time.monotonic()+args.wait
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.1)
        nodes=set(n for n,_ in node.get_node_names_and_namespaces())
        named=set(ns.rstrip('/')+'/'+n if ns!='/' else '/'+n for n,ns in node.get_node_names_and_namespaces())
        topics=dict(node.get_topic_names_and_types())
        print('=== Required Robot 01 ===')
        for n in ('/robot_01/planner_server','/robot_01/controller_server',
                  '/robot_01/bt_navigator','/destination_server'):
            print(('OK ' if n in named else 'MISSING ')+n)
        for t in ('/robot_01/scan','/robot_01/odom','/robot_01/cmd_vel',
                  '/robot_01/destination_command','/robot_01/robot_state',
                  '/robot_01/serving/status'):
            print(('FOUND ' if t in topics else 'NOT FOUND ')+t)
        print('=== TF (Robot 01) ===')
        for target,source in [('map','odom'),('odom','base_footprint'),
                              ('map','base_footprint')]:
            try:
                tf=buf.lookup_transform(target,source,Time())
                print('OK',target,'<-',source)
            except Exception as exc:
                print('MISSING',target,'<-',source,exc)
        print('=== Person simulator ===')
        for n in ('/gazebo_person_event_bridge','/person_simulator_manager',
                  '/person_simulator_watchdog'):
            print(('FOUND ' if n in named else 'MISSING ')+n)
        print('=== Robot 02 (NO SPAWN) ===')
        for n in ('/robot_02/controller_server','/robot_02/amcl'):
            print(('FOUND ' if n in named else 'NOT YET ')+n)
        for target,source in [('map','robot_02/odom'),
                              ('robot_02/odom','robot_02/base_footprint')]:
            try:
                buf.lookup_transform(target,source,Time())
                print('FOUND TF',target,'<-',source)
            except Exception:
                print('NOT YET TF',target,'<-',source)
        print('NOTE: Topic/TF presence does not prove actual person/robot02 motion or collision safety.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
