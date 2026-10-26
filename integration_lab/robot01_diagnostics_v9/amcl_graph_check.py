#!/usr/bin/env python3
"""Read-only ROS graph snapshot; no publishers, commands, or node restarts."""
import argparse
import json
import os
import time

TOPICS = [
    '/initialpose', '/robot_01/initialpose', '/map', '/robot_01/map',
    '/tf', '/tf_static', '/robot_01/scan', '/robot_01/amcl_pose', '/amcl_pose',
]

def endpoint(p):
    return {'node': p.node_namespace.rstrip('/') + '/' + p.node_name,
            'type': p.topic_type, 'qos_reliability': str(p.qos_profile.reliability),
            'qos_durability': str(p.qos_profile.durability)}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-seconds', type=float, default=10.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    import rclpy
    rclpy.init()
    n = rclpy.create_node('robot01_graph_diagnostic_v9')
    try:
        deadline = time.monotonic() + max(0.0, args.wait_seconds)
        snapshots = []
        while True:
            observed = sorted(ns.rstrip('/') + '/' + name
                              for name, ns in n.get_node_names_and_namespaces())
            snapshots.append({'elapsed_s': round(max(0.0, args.wait_seconds - max(0, deadline-time.monotonic())),1),
                              'node_count': len(observed),
                              'amcl_visible': '/robot_01/amcl' in observed})
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(n, timeout_sec=0.25)
        topics = {}
        for topic in TOPICS:
            topics[topic] = {
                'publishers': [endpoint(p) for p in n.get_publishers_info_by_topic(topic)],
                'subscribers': [endpoint(p) for p in n.get_subscriptions_info_by_topic(topic)]
            }
        result = {'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
                  'waited_seconds': args.wait_seconds,
                  'discovery_samples': snapshots[::max(1, len(snapshots)//7)],
                  'nodes': observed, 'topics': topics,
                  'note': 'Graph visibility alone cannot prove whether a lifecycle node exists.'}
        output = json.dumps(result, ensure_ascii=False, indent=2)
        print(output)
        if args.out:
            with open(args.out,'w',encoding='utf8') as f:
                f.write(output+'\n')
    finally:
        n.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
