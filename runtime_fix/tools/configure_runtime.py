#!/usr/bin/env python3
"""Safe, atomic numeric setting changes. Only modifies copied project config files.
Usage: python3 tools/configure_runtime.py --set arrival.xy_tolerance_m 0.10
       python3 tools/configure_runtime.py --check
Restart ROS stack after configuring; no run-time magic or undocumented defaults.
"""
import argparse, copy, os, sys, tempfile
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent.parent
PKG=ROOT/'ros_ws/src/cafe_autonomy'
CF=PKG/'config/runtime_limits.yaml'
sys.path.insert(0,str(PKG))
from cafe_autonomy.runtime_config import load_limits

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self,data): return True

def dump(d): return yaml.dump(d,Dumper=NoAliasDumper,sort_keys=False,allow_unicode=True)

def staged_write(p,text):
    # no partial config; old file copied to .bak when changing it
    fd,name=tempfile.mkstemp(prefix='.'+p.name+'.',dir=p.parent)
    with os.fdopen(fd,'w',encoding='utf8') as f:f.write(text)
    os.replace(name,p)

def sync_nav2(cfg,dry_run=False):
    for rid in ('robot_01','robot_02'):
        p=PKG/'config'/('nav2_'+rid+'.yaml')
        doc=yaml.safe_load(p.read_text(encoding='utf8'))
        node=doc[f'/{rid}/controller_server']['ros__parameters']
        node['general_goal_checker']['xy_goal_tolerance']=cfg['navigation']['goal_xy_tolerance_m']
        node['general_goal_checker']['yaw_goal_tolerance']=cfg['navigation']['goal_yaw_tolerance_rad']
        node['FollowPath']['vx_max']=cfg['navigation']['max_forward_mps']
        node['FollowPath']['wz_max']=cfg['navigation']['max_yaw_rps']
        smooth=doc[f'/{rid}/velocity_smoother']['ros__parameters']
        smooth['max_velocity'][0]=cfg['navigation']['max_forward_mps']
        smooth['max_velocity'][2]=cfg['navigation']['max_yaw_rps']
        smooth['min_velocity'][2]=-cfg['navigation']['max_yaw_rps']
        smooth['velocity_timeout']=cfg['safety']['command_timeout_s']
        if not dry_run:staged_write(p,dump(doc))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--set',nargs=2,metavar=('SECTION.KEY','NUMBER'))
    p.add_argument('--check',action='store_true')
    p.add_argument('--dry-run',action='store_true')
    a=p.parse_args()
    old=load_limits(CF)
    new=copy.deepcopy(old)
    if a.set:
        path,num=a.set
        try:
            section,key=path.split('.')
            if section not in old or key not in old[section]:raise ValueError('Unknown key')
            new[section][key]=float(num)
        except (ValueError,KeyError) as exc:raise SystemExit('INVALID SETTING: '+str(exc))
        # validate before changing a single file
        import tempfile
        with tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=True) as tmp:
            tmp.write(dump(new));tmp.flush();load_limits(tmp.name)
    if a.check or a.dry_run:
        for rid in ('robot_01','robot_02'):
            nav=yaml.safe_load((PKG/'config'/('nav2_'+rid+'.yaml')).read_text())
            got=nav[f'/{rid}/controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']
            print(rid,'Nav2_xy',got,'runtime_xy',old['navigation']['goal_xy_tolerance_m'])
            if a.check and got!=old['navigation']['goal_xy_tolerance_m']:
                raise SystemExit('FAIL: Nav2 not synchronized')
    if a.set and not a.dry_run:
        staged_write(CF.with_suffix('.yaml.bak'),CF.read_text(encoding='utf8'))
        staged_write(CF,dump(new))
        sync_nav2(new)
        print('APPLIED',a.set,'Nav2 Robot 01/02 synchronized, prior runtime .yaml.bak saved')
    elif a.set:print('DRY RUN only',a.set)
    else:print('Config valid')
if __name__=='__main__':main()
