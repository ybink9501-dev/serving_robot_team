#!/usr/bin/env python3
"""LAB-only safe connection of the preexisting person simulator to SemanticSafety.

--inspect: read ROS graph; no publisher created.
--apply: create an event-only relay ONLY when not already connected and safe.
NEVER publishes Twist, /cmd_vel or Nav2 GO. Existing semantic_safety owns
person-based speed gating and collision_monitor remains final cmd_vel owner.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from person_chain_policy import evaluate_graph


def endpoint_names(entries):
    return [('/' + str(e.node_namespace).strip('/') if e.node_namespace not in ('', '/') else '')
            + '/' + e.node_name for e in entries]


def safety_subscribers(node, topic, expected):
    return [name for name in endpoint_names(node.get_subscriptions_info_by_topic(topic))
            if name.rsplit('/', 1)[-1] == expected]


def publishers(node, topic):
    return endpoint_names(node.get_publishers_info_by_topic(topic))


def audit(node, cfg):
    src, dest = cfg['source_topic'], cfg['fallback_safety_input_topic']
    safety_name = cfg['required_safety_node']
    src_publishers = publishers(node, src)
    direct_safety = safety_subscribers(node, src, safety_name)
    target_safety = safety_subscribers(node, dest, safety_name)
    dest_publishers = publishers(node, dest)
    semantic_publishers = publishers(node, cfg['semantic_velocity_topic'])
    final_publishers = publishers(node, cfg['final_velocity_topic'])
    verdict = evaluate_graph(src_publishers, direct_safety, target_safety,
                             dest_publishers, semantic_publishers, final_publishers)
    return {'verdict':verdict, 'source_publishers':src_publishers,
            'direct_safety_subscribers':direct_safety,
            'fallback_safety_subscribers':target_safety,
            'fallback_publishers':dest_publishers,
            'semantic_velocity_publishers':semantic_publishers,
            'final_velocity_publishers':final_publishers}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--apply', action='store_true')
    args=parser.parse_args()
    if args.inspect == args.apply:
        parser.error('choose exactly one of --inspect or --apply')
    config_path=Path(__file__).resolve().parent/'config.yaml'
    cfg=yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if os.environ.get('ROS_DOMAIN_ID') != str(cfg['expected_ros_domain_id']):
        sys.exit('REJECT_ROS_DOMAIN_ID_MISMATCH')
    src=cfg['source_topic']; dest=cfg['fallback_safety_input_topic']
    if src == dest or src == cfg['final_velocity_topic'] or dest == cfg['final_velocity_topic']:
        sys.exit('REJECT_TOPIC_CONFIGURATION')

    import rclpy
    from rclpy.node import Node
    from serving_robot_interfaces.msg import PersonSafetyEvent

    rclpy.init()
    node = Node('person_event_connection_lab_v4')
    try:
        t_end=time.monotonic()+cfg['startup_discovery_s']
        result=None
        while time.monotonic() < t_end:
            result=audit(node,cfg)
            if result['verdict'] in ('DIRECT_READY','RELAY_ALLOWED',
                                     'EXTERNAL_SOURCE_PRESENT_NO_LAB_RELAY'):
                break
            time.sleep(.2)
        if result is None:result=audit(node,cfg)
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        verdict=result['verdict']
        if args.inspect:
            if verdict.startswith('REJECT_'):sys.exit(2)
            return
        if verdict == 'DIRECT_READY':
            print('V4: ALREADY_CONNECTED. No relay or velocity publisher created.',flush=True)
            return
        if verdict != 'RELAY_ALLOWED':
            sys.exit('V4: '+verdict+'; no ROS publisher created.')

        # Reconfirm before creating a publisher, to reduce graph discovery race.
        if audit(node,cfg)['verdict'] != 'RELAY_ALLOWED':
            sys.exit('V4: graph changed; refusing relay.')
        pub = node.create_publisher(PersonSafetyEvent, dest, 10)
        def forward(event):
            if event.robot_id != cfg['robot_id']:
                return  # Existing safety owner will fail closed on stale input.
            if event.state not in ('NORMAL','SLOW','STOP','SENSOR_STALE'):
                safe=PersonSafetyEvent()
                safe.header=event.header
                safe.robot_id=cfg['robot_id']
                safe.state='SENSOR_STALE'
                safe.confidence=0.0
                safe.distance_m=0.0
                pub.publish(safe)
                return
            if (event.state in ('NORMAL','SLOW') and
                (not math.isfinite(float(event.distance_m)) or event.distance_m < 0)):
                event.state='SENSOR_STALE'
                event.confidence=0.0
                event.distance_m=0.0
            pub.publish(event)
        sub = node.create_subscription(PersonSafetyEvent,src,forward,10)
        print(f'V4: EVENT_RELAY_ACTIVE {src} -> {dest} (no Twist publisher)',flush=True)
        next_check=time.monotonic()+cfg['graph_recheck_s']
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_check:
                # The relay is the only permitted fallback publisher.
                current=audit(node,cfg)
                own=node.get_name()
                outside=[p for p in current['fallback_publishers'] if p.rsplit('/',1)[-1]!=own]
                if outside or len(current['fallback_safety_subscribers'])!=1:
                    print('V4: graph changed, relay stopping (fail-closed): '+json.dumps(current,ensure_ascii=False),flush=True)
                    break
                if not current['source_publishers'] or not current['semantic_velocity_publishers'] or not current['final_velocity_publishers']:
                    print('V4: safety chain is incomplete, relay stopping (fail-closed)',flush=True)
                    break
                next_check=time.monotonic()+cfg['graph_recheck_s']
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
