#!/usr/bin/env python3
"""Repair one incompatible ROS 2 Jazzy V6 preflight action lookup. No ROS calls."""
from pathlib import Path
import os

OLD = '''        actions=dict(node.get_action_names_and_types())
        for action in ('/robot_01/navigate_to_pose','/robot_01/compute_path_to_pose'):
            if action not in actions:report['errors'].append({'name':'Nav2 action','error':f'Missing {action}'})
            else:report['checks'].append({'name':action,'result':'PASS'})'''
NEW = '''        # This rclpy.Node does not provide get_action_names_and_types().
        # Check readiness through Nav2 action clients; never send a goal.
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        for action, action_type in (
            ('/robot_01/navigate_to_pose', NavigateToPose),
            ('/robot_01/compute_path_to_pose', ComputePathToPose),
        ):
            client = ActionClient(node, action_type, action)
            try:
                if client.wait_for_server(timeout_sec=2.0):
                    report['checks'].append({'name': action, 'result': 'PASS'})
                else:
                    report['errors'].append({'name': 'Nav2 action',
                                             'error': f'Action server unavailable: {action}'})
            finally:
                node.destroy_action_client(client)'''

def main():
    target = Path(os.environ.get('V6_PREFLIGHT_TARGET', str(Path.home() / 'team_robot_lab/robot01_pretest_v6/preflight.py')))
    if not target.is_file():
        raise SystemExit(f'NOT FOUND: {target}')
    content = target.read_text(encoding='utf-8')
    if NEW in content:
        print('ALREADY PATCHED:', target)
        return
    if content.count(OLD) != 1:
        raise SystemExit('STOP: Unexpected preflight version; original file not modified')
    replacement = content.replace(OLD, NEW, 1)
    compile(replacement, str(target), 'exec')
    backup = target.with_name('preflight.py.before_actionfix_v6.bak')
    if not backup.exists():
        backup.write_text(content, encoding='utf-8')
    target.write_text(replacement, encoding='utf-8')
    print('PATCHED: action server inspection only')
    print('BACKUP:', backup)
    print('NO GOAL SENT; no ROS nodes touched')

if __name__ == '__main__':
    main()
