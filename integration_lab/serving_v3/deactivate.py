#!/usr/bin/env python3
"""Stop only this LAB V3 supervisor before swapping semantic/map snapshots."""
import os
from pathlib import Path
import signal
from activate import find_exact, guard_robot, wait_exit

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='99':raise RuntimeError('ROS_DOMAIN_ID must be 99')
    processes=find_exact(Path(__file__).resolve().parent/'supervisor_v3.py')
    if not processes:
        print('No serving V3 supervisor running; no action needed');return
    if len(processes)!=1:raise RuntimeError('Multiple V3 supervisors detected; stop manually')
    guard_robot()
    pid,_=processes[0]
    os.kill(pid,signal.SIGINT)
    if not wait_exit(pid):raise RuntimeError('Supervisor still running; no source/config changes permitted')
    print('Stopped V3 supervisor PID',pid,'; destination server and Nav2 stay running')

if __name__=='__main__':
    try:main()
    except Exception as e:raise SystemExit('V3 DEACTIVATION ABORTED: '+str(e))
