#!/usr/bin/env python3
"""Offline analysis of independent Gazebo/odom/TF evidence.
Absent truth or time-series data is a failing result, never a PASS.
"""
import argparse,csv,json,math,statistics,sys
from pathlib import Path

def number(x):
    try:return float(x)
    except (ValueError,TypeError):return None

def analyze(path):
    rows=list(csv.DictReader(Path(path).open()))
    result={'file':str(path),'rows':len(rows),'robots':{},'pass':False}
    for r in ('robot_01','robot_02'):
        rr=[row for row in rows if row['robot']==r]
        errors=[];distances=[];tf_err=[];violations=[];stationary_spin=[]
        if not rr:errors.append('No robot rows')
        for row in rr:
            a=number(row.get('xy_error_odom_gt'))
            b=number(row.get('xy_error_tf_gt'))
            if a is not None:distances.append(a)
            if b is not None:tf_err.append(b)
            nav=number(row.get('cmd_nav_v'))
            w=number(row.get('cmd_nav_w'))
            safe=number(row.get('cmd_safe_v'))
            if nav is not None and (nav < -1e-4 or nav > 0.2501):violations.append(('nav_limit',row.get('wall_time'),nav))
            if w is not None and abs(w)>0.8001:violations.append(('angular_limit',row.get('wall_time'),w))
            if safe is not None and safe < -1e-4:violations.append(('reverse_safe_command',row.get('wall_time'),safe))
        # Velocity commands and TF existence alone cannot certify physical movement.
        positions=[(number(row.get('gt_x')),number(row.get('gt_y'))) for row in rr]
        validpos=[v for v in positions if None not in v]
        traveled=sum(math.hypot(x2-x1,y2-y1) for (x1,y1),(x2,y2) in zip(validpos,validpos[1:]))
        if traveled<0.10:errors.append('No demonstrated Gazebo movement >= 10cm')
        if not distances:errors.append('No Gazebo Ground Truth vs odom data')
        if not tf_err:errors.append('No Gazebo Ground Truth vs map TF data')
        # 20-second sliding window, less than 8 cm movement and >=270 deg turning.
        for i,start in enumerate(rr):
            t0=number(start.get('wall_time'))
            x0=number(start.get('gt_x'));y0=number(start.get('gt_y'))
            a0=number(start.get('gt_yaw'))
            if None in (t0,x0,y0,a0):continue
            rotations=0.;prior=a0;max_displacement=0.
            for row in rr[i+1:]:
                t=number(row.get('wall_time'))
                if t is None or t-t0>20:break
                x=number(row.get('gt_x'));y=number(row.get('gt_y'));a=number(row.get('gt_yaw'))
                if None in (x,y,a):continue
                max_displacement=max(max_displacement,math.hypot(x-x0,y-y0))
                rotations+=abs(math.atan2(math.sin(a-prior),math.cos(a-prior)))
                prior=a
                if rotations>=math.radians(270) and max_displacement<0.08:
                    stationary_spin.append({'wall_time':t,'turn_rad':round(rotations,4),'max_displacement_m':round(max_displacement,4)})
                    break
        if stationary_spin:errors.append('Over-rotation while stalled >=270deg/20s')
        if violations:errors.append('Velocity command limit violated')
        result['robots'][r]={'rows':len(rr),'truth_samples':len(distances),'travelled_ground_truth_m':round(traveled,5),
          'odom_error_max_m':max(distances) if distances else None,
          'odom_error_p95_m':sorted(distances)[int(.95*(len(distances)-1))] if distances else None,
          'tf_error_max_m':max(tf_err) if tf_err else None,
          'violations':violations[:10],'stationary_spin_events':stationary_spin[:10],
          'errors':errors,'pass':not errors}
    result['pass']=all(v['pass'] for v in result['robots'].values())
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('csv_file');p.add_argument('--json-file')
    a=p.parse_args();r=analyze(a.csv_file)
    payload=json.dumps(r,ensure_ascii=False,indent=2)
    print(payload)
    if a.json_file:Path(a.json_file).write_text(payload)
    sys.exit(0 if r['pass'] else 1)
