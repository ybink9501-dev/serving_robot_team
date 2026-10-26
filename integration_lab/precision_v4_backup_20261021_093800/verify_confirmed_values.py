#!/usr/bin/env python3
from pathlib import Path
import yaml
p=Path(__file__).resolve().parent
n=yaml.safe_load((p/'nav2_lab_v4.yaml').read_text())
a=yaml.safe_load((p/'arrival_v4.yaml').read_text())
f=yaml.safe_load((p/'confirmed_requirements.yaml').read_text())
c=n['controller_server']['ros__parameters']; m=c['FollowPath']; v=n['velocity_smoother']['ros__parameters']; r=a['arrival']; fine=a['fine_approach']; q=f['robot_operation']; ar=f['arrival']
tests={
 'nav2_xy':c['general_goal_checker']['xy_goal_tolerance']==ar['nav2_xy_goal_tolerance_m'],
 'nav2_yaw':c['general_goal_checker']['yaw_goal_tolerance']==ar['nav2_yaw_tolerance_rad'],
 'arrival_detection':r['detection_radius_m']==ar['detection_radius_m'],
 'arrival_final':r['final_error_m']==ar['final_xy_error_m'],
 'mppi_forward_max':m['vx_max']==q['mppi_max_forward_mps'],
 'mppi_reverse_forbidden':m['vx_min']==0,
 'mppi_turn_max':m['wz_max']==q['mppi_max_turn_radps'],
 'smoother_forward_max':v['max_velocity'][0]==q['mppi_max_forward_mps'],
 'smoother_turn_max':v['max_velocity'][2]==q['mppi_max_turn_radps'],
 'smoother_reverse_forbidden':v['min_velocity'][0]==0,
 'smoother_turn_min':v['min_velocity'][2]==-q['mppi_max_turn_radps'],
 'fine_stays_below_forward_max':fine['v_max']<=q['mppi_max_forward_mps'],
 'fine_stays_below_turn_max':fine['w_max']<=q['mppi_max_turn_radps'],
 'fine_target_inside_final':fine['xy_tol_m']<=r['final_error_m'],
 'fine_uses_safety_input':fine['cmd_topic']=='cmd_vel_nav',
 'low_normal_high_modes_preserved':(q['low_speed_mps'],q['normal_speed_mps'],q['high_speed_mps'])==(0.1,0.15,0.2),
 'in_place_turn_target_separate':q['in_place_turn_target_radps']==0.5,
}
for k,ok in tests.items():print(('PASS' if ok else 'FAIL'),k)
print('NOTE: operational speed MODES and obstacle policy are documented requirements; not falsely asserted as implemented.')
raise SystemExit(0 if all(tests.values()) else 1)
