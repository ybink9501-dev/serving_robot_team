"""On-demand pedestrians: separate Gazebo models, no writes to robot cmd_vel.

Commands: /person_simulator/command (std_msgs/String containing JSON).
Built-in safeguards gate motion on recent *world-ground-truth* robot + person poses.
"""
import fcntl
import json
import math
import os
from pathlib import Path
import random
import secrets
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String, Empty
import yaml

from .gazebo_api import spawn, remove
from .pose_cli import PoseStream
from .policy import (PersonRules, Walker, apply_deadlock_overlay, decide_action, draw_encounter,
                     draw_speed, evade_velocity, next_time, plan_evade, validate,
                     validate_spawn_positions)


def xy(transform):
    return (transform.transform.translation.x, transform.transform.translation.y)


from .policy import can_escape_person_hold

class Manager(Node):
    def __init__(self):
        super().__init__('person_simulator_manager')
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise RuntimeError('config_path is required')
        with open(config_path, encoding='utf-8') as f:
            self.cfg=validate(yaml.safe_load(f))
        # Optional overlay: may ONLY set the two deadlock keys (never speeds/radii/lanes).
        self.declare_parameter('overlay_config_path', '')
        overlay_path = self.get_parameter('overlay_config_path').value
        if overlay_path:
            with open(overlay_path, encoding='utf-8') as f:
                self.cfg = apply_deadlock_overlay(self.cfg, yaml.safe_load(f))
        self.rules = PersonRules(self.cfg)
        # Robot encounters: 'yield' = original 80/20 yield (default), 'mixed' = wait/block/back_off/sidestep.
        self.declare_parameter('robot_encounter', 'yield')
        encounter_mode = self.get_parameter('robot_encounter').value
        if encounter_mode not in ('yield', 'mixed'):
            raise ValueError("robot_encounter must be 'yield' or 'mixed'")
        self.encounter_mixed = encounter_mode == 'mixed'
        self.world=self.cfg['world']
        self.prefix=self.cfg['model_prefix']
        self.robot_names=self.cfg.get('robot_models',[self.cfg['robot_model']])
        if not isinstance(self.robot_names,list) or not self.robot_names or any(
            r not in ('robot_01','robot_02') for r in self.robot_names):
            raise RuntimeError('robot_models must contain robot_01/robot_02')
        if self.prefix != 'test_person_':
            raise RuntimeError('model_prefix must remain test_person_ for ownership safety')
        self.seed = self.cfg['seed'] if self.cfg['seed'] is not None else secrets.randbits(63)
        self.rng = random.Random(self.seed)
        self.people = {}  # id -> Walker
        self.test_spawn_pending = None
        self.test_spawn_result = None
        self.cmd_pubs = {
            i: self.create_publisher(Twist, f'/model/{self.prefix}{i:02d}/cmd_vel', 10)
            for i in range(1,self.cfg['people_max']+1)
        }
        self.poses = {}
        self.pose_received_at = {}
        self.main_paused = False
        self.shutdown_requested = False
        self.last_error = None
        self.last_sim_time = None
        self.last_print_state = None
        # Raw Gazebo model pose names are preserved by the CLI; the Pose_V ->
        # TFMessage bridge in this environment drops child_frame_id names.
        self.pose_stream = PoseStream(self.world, self.prefix, self.robot_names)
        self.create_subscription(String, '/person_simulator/command', self.on_command, 10)
        self.status_pub=self.create_publisher(String, '/person_simulator/status', 10)
        self.heartbeat_pub=self.create_publisher(Empty, '/person_simulator/heartbeat', 10)
        self.create_timer(1.0 / self.cfg['control_rate_hz'],self.tick)
        self.create_timer(1.0,self.publish_status)
        folder=Path.home()/'.ros'/'person_simulator'
        folder.mkdir(parents=True, exist_ok=True)
        self.log_path=folder/f'run_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}.jsonl'
        self.log_file=self.log_path.open('w',encoding='utf-8',buffering=1)
        self.record('startup',seed=self.seed,world=self.world,deadlock_resolution=self.rules.enabled)
        self.get_logger().info(f'Ready: seed={self.seed}. No humans spawned by default. Configured lanes={len(self.cfg["lanes"])}')

    def sim_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def record(self,event,**fields):
        if hasattr(self,'log_file') and not self.log_file.closed:
            self.log_file.write(json.dumps({'sim_s':round(self.sim_now(),3),
              'event':event,**fields},ensure_ascii=False)+'\n')

    def refresh_poses(self):
        # No fallback to anonymous TF transforms or odom coordinates.
        for name, (point, received) in self.pose_stream.snapshot().items():
            if received > self.pose_received_at.get(name, -1):
                self.poses[name] = point
                self.pose_received_at[name] = received

    def fresh_pose(self,name):
        self.refresh_poses()
        received=self.pose_received_at.get(name)
        if received is None or time.monotonic()-received > self.cfg['pose_timeout_s']:
            return None
        p=self.poses.get(name)
        return p if p is not None and all(math.isfinite(v) for v in p) else None

    def command_error(self,reason):
        self.last_error = reason
        self.get_logger().error(reason)
        self.record('command_error',reason=reason)
        self.publish_status(error=reason)

    def on_command(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('command must be a JSON object')
            action=data.get('action')
            if action in ('spawn','delete','start','stop','resume','guest_cycle','sudden_stop','long_block','pause','unpause','cleanup','shutdown','recover','status'):
                self.process_command(data)
                self.last_error = None
                self.publish_status()
            else:
                raise ValueError(f'unknown action {action!r}')
        except Exception as exc:
            self.command_error(str(exc))

    def _ids(self,data,allow_empty=False):
        if 'id' not in data:
            return list(self.people)
        ident=data['id']
        if type(ident) is not int or not 1<=ident<=self.cfg['people_max']:
            raise ValueError('id must be an integer 1..8')
        if ident not in self.people and not allow_empty:
            raise ValueError(f'person {ident} is not owned/active')
        return [ident]

    def _spawn_one(self,ident,start,end,guest_mode=False,spawn_at=None,yaw=0.0):
        name=f'{self.prefix}{ident:02d}'
        if self.fresh_pose(name) is not None:
            raise ValueError(f'orphan / duplicate exists: {name}; use recover explicitly')
        origin=start if spawn_at is None else spawn_at
        spawn(self.world,name,*origin,self.cfg['person_radius_m'],self.cfg['person_height_m'],yaw=yaw)
        # A true service response means the request was accepted, NOT that the
        # world entity exists. Gazebo ground-truth confirmation is mandatory.
        deadline = time.monotonic() + 6.0
        confirmed = None
        while time.monotonic() < deadline:
            self.heartbeat_pub.publish(Empty())
            confirmed = self.fresh_pose(name)
            if confirmed is not None:
                break
            time.sleep(0.1)
        if confirmed is None or math.dist(confirmed, origin) > 0.4:
            # Clean up any late / misplaced entity; never register a ghost.
            try:
                remove(self.world, name)
            except Exception as exc:
                self.get_logger().warning(f'Spawn cleanup could not remove {name}: {exc}')
            raise RuntimeError(
                f'Gazebo spawn not confirmed for {name}; '
                f'pose={confirmed!r}, expected={origin!r}'
            )
        _, speed=draw_speed(self.rng,self.cfg)
        w=Walker(ident,name,start,end,speed_mps=speed,paused=True,
                 guest_mode=guest_mode,spawn_yaw=yaw,
                 next_decision=next_time(self.rng,self.cfg,self.sim_now()))
        self.people[ident]=w
        self.record('spawn',id=ident,name=name,x=origin[0],y=origin[1])

    def process_command(self,data):
        action=data['action']
        if action=='spawn':
            count=data.get('count',1)
            mode=data.get('mode','legacy')
            if mode not in ('legacy','normal','test'):
                raise ValueError('spawn mode must be legacy, normal or test')
            maximum=3 if mode=='normal' else self.cfg['people_max']
            if type(count) is not int or count<1 or count>maximum:
                raise ValueError(f'count must be 1..{maximum} for {mode} mode')
            if self.test_spawn_pending is not None:
                raise ValueError('test spawn already in progress')
            if mode=='test':
                if 'test_spawn' not in self.cfg:
                    raise ValueError('test_spawn is not configured')
                if self.people:
                    raise ValueError('test spawn requires no active people; cleanup first')
                self.test_spawn_pending={
                    'requested':count,'created':0,
                    'deadline':time.monotonic()+self.cfg['test_spawn']['wait_timeout_s']}
                self.test_spawn_result={'state':'pending','requested':count,'created':0}
                self.record('test_spawn_requested',requested=count,
                            x=self.cfg['test_spawn']['x'],y=self.cfg['test_spawn']['y'])
                return
            available=[i for i in range(1,9) if i not in self.people]
            if count > len(available):
                raise ValueError('count exceeds available person slots')
            # All positions preflighted before the first spawn: no partial batch
            # if one future slot collides with an existing/other future person.
            if len(self.cfg['lanes']) < max(available[:count]):
                raise ValueError('Not enough reviewed lanes in config/policy.yaml; no automatic spawn')
            plans=[]
            for ident in available[:count]:
                lane=self.cfg['lanes'][ident-1]
                # Normal-mode guest 02 starts at the lane's existing waiting end.
                if mode=='normal' and ident==2:
                    plans.append((tuple(lane['end']),tuple(lane['start'])))
                else:
                    plans.append((tuple(lane['start']),tuple(lane['end'])))
            # Validate all scheduled people against EVERY active robot, not just robot01.
            for robot_name in self.robot_names:
                robot_pose=self.fresh_pose(robot_name)
                if robot_pose is None:
                    raise RuntimeError('Gazebo robot pose unavailable: '+robot_name)
                validate_spawn_positions(
                    plans,robot_pose,
                    [self.fresh_pose(w.name) for w in self.people.values()],
                    self.cfg['robot_release_radius_m'],
                    self.cfg['person_release_radius_m'])
            created=[]
            try:
                for ident,(start,end) in zip(available[:count],plans):
                    self._spawn_one(ident,start,end,guest_mode=(mode=='normal' and ident==2))
                    created.append(ident)
            except Exception:
                for ident in reversed(created):
                    try:
                        self.delete_person(ident)
                    except Exception as exc:
                        self.get_logger().error(f'Rollback failed for {ident}: {exc}')
                raise
            return
        if action in ('sudden_stop','long_block'):
            if 'id' not in data:
                raise ValueError('manual event requires one person id')
            ident=self._ids(data)[0]
            w=self.people[ident]
            if w.paused or w.guest_stage=='counter_pause':
                raise ValueError('person is already paused')
            key='sudden_pause_s' if action=='sudden_stop' else 'long_block_pause_s'
            duration=self.rng.uniform(*self.cfg['behavior'][key])
            w.phase='sudden_pause' if action=='sudden_stop' else 'long_block'
            w.current_action=action
            w.until=self.sim_now()+duration
            self.send_zero(ident)
            self.record('manual_'+action,id=ident,duration_s=round(duration,3))
            return
        if action=='guest_cycle':
            if data.get('id') != 2 or 2 not in self.people or not self.people[2].guest_mode:
                raise ValueError('guest_cycle requires normal-mode person 2')
            w=self.people[2]
            if w.guest_stage not in (None,'waiting'):
                raise ValueError('guest cycle already active')
            w.guest_stage='to_counter';w.target_end=True;w.paused=False
            w.phase='moving';w.current_action='continue'
            w.next_decision=next_time(self.rng,self.cfg,self.sim_now())
            self.record('guest_to_counter',id=2)
            return
        if action=='recover':
            if self.people:
                raise ValueError('delete owned people before recovering orphans')
            # Never remove arbitrary models; only previously known unique owned prefix.
            for i in range(1,9):
                name=f'{self.prefix}{i:02d}'
                if self.fresh_pose(name) is not None:
                    remove(self.world,name)
                    self.record('recover_orphan',name=name)
            return
        if action in ('delete','cleanup'):
            if self.test_spawn_pending is not None:
                self.test_spawn_result={'state':'canceled',
                    'requested':self.test_spawn_pending['requested'],
                    'created':self.test_spawn_pending['created']}
                self.test_spawn_pending=None
            ids=self._ids(data) if action=='delete' else list(self.people)
            for ident in ids:
                self.delete_person(ident)
            return
        if action=='status':
            self.publish_status()
            return
        if action=='shutdown':
            self.shutdown_requested=True
            return
        if action=='pause':
            self.main_paused=True
            self.stop_all()
            self.record('global_pause')
            return
        if action=='unpause':
            self.main_paused=False
            self.record('global_unpause')
            return
        if action in ('stop','start','resume'):
            for ident in self._ids(data):
                w=self.people[ident]
                w.paused=(action=='stop')
                if action in ('start','resume'):
                    w.guest_stage=None
                w.phase='moving'
                w.current_action='continue'
                w.next_decision=next_time(self.rng,self.cfg,self.sim_now())
                if w.paused:
                    self.send_zero(ident)
                self.record(action,id=ident)

    def delete_person(self,ident):
        w=self.people[ident]
        self.send_zero(ident)
        remove(self.world,w.name)
        self.record('delete',id=ident)
        del self.people[ident]

    def stop_all(self):
        for ident in self.people:
            self.send_zero(ident)

    def send_zero(self,ident):
        if rclpy.ok():
            self.cmd_pubs[ident].publish(Twist())
        if ident in self.people:
            self.people[ident].published_v=(0,0)

    def _advance_test_spawn(self):
        pending=self.test_spawn_pending
        if pending is None:
            return
        if time.monotonic() >= pending['deadline']:
            reason=f'test spawn doorway wait exceeded 60s: {pending["created"]}/{pending["requested"]} created'
            self.last_error=reason
            self.test_spawn_result={'state':'timeout','requested':pending['requested'],
                                    'created':pending['created'],'reason':reason}
            self.test_spawn_pending=None
            self.record('test_spawn_timeout',requested=pending['requested'],
                        created=pending['created'],reason=reason)
            self.publish_status(error=reason)
            return
        entry_cfg=self.cfg['test_spawn']
        entry=(entry_cfg['x'],entry_cfg['y'])
        robots=[self.fresh_pose(name) for name in self.robot_names]
        existing=[self.fresh_pose(w.name) for w in self.people.values()]
        if any(point is None for point in robots+existing) or self.main_paused:
            return
        # Keep the existing person/robot release distances; never force overlap.
        if any(math.dist(entry,point)<self.cfg['person_release_radius_m'] for point in existing):
            return
        if any(math.dist(entry,point)<self.cfg['robot_release_radius_m'] for point in robots):
            return
        ident=pending['created']+1
        lane=self.cfg['lanes'][0]  # Existing central corridor, reached from the entry point.
        try:
            self._spawn_one(ident,tuple(lane['start']),tuple(lane['end']),
                            spawn_at=entry,yaw=entry_cfg['yaw'])
        except Exception as exc:
            reason=f'test spawn failed at person {ident}: {exc}'
            self.last_error=reason
            self.test_spawn_result={'state':'failed','requested':pending['requested'],
                                    'created':pending['created'],'reason':reason}
            self.test_spawn_pending=None
            self.record('test_spawn_failed',id=ident,reason=reason)
            self.publish_status(error=reason)
            return
        w=self.people[ident]
        w.target_end=False  # Enter from the configured point via the reviewed lane start.
        w.paused=False
        pending['created']+=1
        self.test_spawn_result={'state':'pending','requested':pending['requested'],
                                'created':pending['created']}
        self.record('test_spawn_admitted',id=ident,created=pending['created'],
                    requested=pending['requested'])
        if pending['created']==pending['requested']:
            self.test_spawn_result={'state':'complete','requested':pending['requested'],
                                    'created':pending['created']}
            self.test_spawn_pending=None
            self.record('test_spawn_complete',created=pending['created'])
        else:
            pending['deadline']=time.monotonic()+entry_cfg['wait_timeout_s']
        self.publish_status()

    def tick(self):
        self.heartbeat_pub.publish(Empty())
        now=self.sim_now()
        if now<=0:
            return
        if self.last_sim_time is None:
            self.last_sim_time=now;return
        dt=now-self.last_sim_time
        self.last_sim_time=now
        if dt<=0 or dt>0.5:
            self.stop_all();return
        self._advance_test_spawn()
        robots=[self.fresh_pose(name) for name in self.robot_names]
        actual_positions={i:self.fresh_pose(w.name) for i,w in self.people.items()}
        for ident,w in list(self.people.items()):
            person=actual_positions[ident]
            if (any(p is None for p in robots) or person is None or self.main_paused or w.paused):
                self.send_zero(ident)
                continue
            if w.guest_stage=='counter_pause':
                if now<w.until:
                    self.send_zero(ident);continue
                w.guest_stage='returning';w.target_end=False
                w.phase='moving';w.current_action='continue'
                w.next_decision=next_time(self.rng,self.cfg,now)
                self.record('guest_return',id=ident)
            elif w.guest_stage=='waiting':
                self.send_zero(ident);continue
            # Absolute emergency proximity guard is NOT probabilistic.
            robot_dist=min(math.dist(person,r) for r in robots)
            if robot_dist <= self.cfg['robot_stop_radius_m']:
                w.proximity_hold=True
            elif robot_dist >= self.cfg['robot_release_radius_m']:
                w.proximity_hold=False
                if not self.encounter_mixed or robot_dist>=1.35+self.cfg['robot_release_radius_m']-self.cfg['robot_stop_radius_m']:
                    # mixed mode: one choice per encounter; it ends only 0.20 m (= existing release-stop gap) beyond the 1.35 m trigger
                    w.yield_selected=None
                    w.encounter=None;w.evade=None
            peers=[(j,q,self.people[j].published_v,self.people[j])
                   for j,q in actual_positions.items() if j!=ident and q is not None]
            if len(peers)!=max(0,len(self.people)-1):
                self.send_zero(ident);continue  # Any unknown person is unsafe.
            self.rules.begin(w,now,person,peers)
            # Yield decision cached per encounter, not rerolled each frame.
            if robot_dist<1.35 and w.yield_selected is None:
                if self.encounter_mixed:
                    self._choose_encounter(ident,w,person,robots,now)
                else:
                    w.yield_selected=self.rng.random()*100<self.cfg['behavior']['give_way_percent']
                    self.record('give_way_choice',id=ident,choice='yield' if w.yield_selected else 'non_yield')
            should_yield=robot_dist<1.35 and w.yield_selected is True
            if w.proximity_hold or should_yield:
                self.send_zero(ident);continue
            if w.phase in ('stopped','gradual_pause','sudden_pause','long_block'):
                if now<w.until:
                    self.send_zero(ident);continue
                w.phase='moving';w.current_action='continue'
                _,w.speed_mps=draw_speed(self.rng,self.cfg)
                w.next_decision=next_time(self.rng,self.cfg,now)
                self.record('resume_auto',id=ident)
            if w.phase=='gradual_brake':
                duration=max(0.01,w.until-w.stop_start)
                remaining=max(0,(w.until-now)/duration)
                speed=w.decel_start_speed*remaining
                if now>=w.until:
                    w.phase='gradual_pause'
                    w.until=now+self.rng.uniform(*self.cfg['behavior']['gradual_pause_s'])
                    speed=0.0
                    self.record('gradual_stop_complete',id=ident)
            else:
                speed=w.speed_mps
                if now>=w.next_decision:
                    kind,duration=decide_action(self.rng,self.cfg,now,*person)
                    w.current_action=kind
                    self.record('action',id=ident,kind=kind)
                    w.next_decision=next_time(self.rng,self.cfg,now)
                    if kind=='gradual_stop':
                        w.phase='gradual_brake';w.stop_start=now
                        w.until=now+duration;w.decel_start_speed=speed
                    elif kind in ('sudden_stop','long_block'):
                        w.phase='sudden_pause' if kind=='sudden_stop' else 'long_block'
                        w.until=now+duration
                        speed=0.0
                    else:
                        _,w.speed_mps=draw_speed(self.rng,self.cfg)
                        speed=w.speed_mps
            if speed<=0:
                self.send_zero(ident);continue
            target=w.end if w.target_end else w.start
            dx,dy=target[0]-person[0],target[1]-person[1]
            length=math.hypot(dx,dy)
            if length<0.12:
                if w.guest_stage=='to_counter':
                    w.guest_stage='counter_pause'
                    w.until=now+self.rng.uniform(*self.cfg['behavior']['gradual_pause_s'])
                    self.record('guest_counter_pause',id=ident,until=round(w.until,3))
                elif w.guest_stage=='returning':
                    w.guest_stage='waiting';w.paused=True
                    self.record('guest_wait',id=ident)
                else:
                    w.target_end=not w.target_end
                    self.record('turnaround',id=ident)
                self.send_zero(ident)
                continue
            vx=speed*dx/length;vy=speed*dy/length
            if w.evade is not None:   # back_off / sidestep: same safety gates below still apply
                vx,vy=self._evade_override(ident,w,person,now,speed)
                if vx==0.0 and vy==0.0:
                    self.send_zero(ident);continue
            if self.rules.hold_blocked(w, person, (vx, vy), peers):
                self._person_block(ident, w, now, person, peers)
                self.send_zero(ident)
                continue
            projected=(person[0]+vx*max(dt,0.1),person[1]+vy*max(dt,0.1))
            if any(math.dist(projected, r) < self.cfg['robot_stop_radius_m'] for r in robots):
                self.send_zero(ident)
                continue

            # Predict the closest distance between moving people (PersonRules).
            if self.rules.future_blocked(w, person, (vx, vy), peers):
                self._person_block(ident, w, now, person, peers)
                self.send_zero(ident)
                continue
            self.rules.mark(ident, w, now, person, peers, False)
            # Gazebo VelocityControl interprets Twist in the model body frame.
            # PersonRules and published_v retain world-frame velocities.
            c=math.cos(w.spawn_yaw);s=math.sin(w.spawn_yaw)
            cmd=Twist();cmd.linear.x=c*vx+s*vy;cmd.linear.y=-s*vx+c*vy
            self.cmd_pubs[ident].publish(cmd)
            w.published_v=(vx,vy)
            w.last_position=person

    def _choose_encounter(self,ident,w,person,robots,now):
        """Mixed mode: one of wait/block/back_off/sidestep (25 % each), decided once per encounter."""
        kind=draw_encounter(self.rng)
        w.encounter=kind
        w.yield_selected=(kind=='wait')   # wait == the existing yield behaviour
        try:
            if kind=='block':             # stand still like the existing long_block pause
                w.phase='long_block';w.current_action='long_block'
                w.until=now+self.rng.uniform(*self.cfg['behavior']['long_block_pause_s'])
            elif kind in ('back_off','sidestep'):
                nearest=min(robots,key=lambda r: math.dist(person,r))
                w.evade=plan_evade(kind,person,w.start,w.end,nearest,now)
        except Exception as exc:          # never let this feature take the manager down: fall back to waiting
            w.yield_selected=True;w.evade=None
            self.record('encounter_error',id=ident,kind=kind,error=repr(exc))
            return
        self.record('encounter_choice',id=ident,kind=kind)

    def _evade_override(self,ident,w,person,now,speed):
        try:
            plan=w.evade
            velocity=evade_velocity(plan,person,w.start,w.end,speed,now)
            if velocity==(0.0,0.0):
                return velocity
            previous=plan.setdefault('progress_xy',person)
            progress_time=plan.setdefault('progress_s',now)
            # A full control tick of motion must occur within the existing
            # pose timeout. Stop pushing when a fixed obstacle blocks a move.
            along=sum((person[i]-previous[i])*plan['dir'][i] for i in (0,1))
            if along>=speed/self.cfg['control_rate_hz']:
                plan['progress_xy']=person
                plan['progress_s']=now
            elif now-progress_time>=self.cfg['pose_timeout_s']:
                self.record('encounter_blocked',id=ident,kind=w.encounter,
                            x=round(person[0],3),y=round(person[1],3))
                w.evade=None
                w.yield_selected=True
                return (0.0,0.0)
            return velocity
        except Exception as exc:
            w.evade=None
            self.record('encounter_error',id=ident,kind=w.encounter,error=repr(exc))
            return (0.0,0.0)

    def _person_block(self,ident,w,now,person,peers):
        event=self.rules.mark(ident,w,now,person,peers,True)
        if event=='grant':
            self.record('deadlock_pass_granted',id=ident,x=round(person[0],3),y=round(person[1],3))
        elif event=='retreat':
            self.record('deadlock_retreat',id=ident,x=round(person[0],3),y=round(person[1],3))

    def publish_status(self,error=None):
        people=[]
        for i,w in sorted(self.people.items()):
            people.append({'id':i,'name':w.name,'position':self.fresh_pose(w.name),
                'paused':w.paused,'phase':w.phase,'action':w.current_action,
                'guest_stage':w.guest_stage,
                'velocity_cmd':w.published_v,'target_end':w.target_end,'goal':w.end if w.target_end else w.start,'person_hold':w.person_hold,'proximity_hold':w.proximity_hold,'yield_selected':w.yield_selected,
                'person_pass':w.person_pass,'deadlock_retreats':w.person_retreats,'encounter':w.encounter})
        payload={'seed':self.seed,'world':self.world,'paused':self.main_paused,
                 'robot_pose_valid':all(self.fresh_pose(r) is not None for r in self.robot_names),
                 'configured_lanes':len(self.cfg['lanes']),'deadlock_resolution':self.rules.enabled,'robot_encounter':'mixed' if self.encounter_mixed else 'yield','people':people,'error':self.last_error if error is None else error,
                 'gazebo_pose_stream_alive':self.pose_stream.alive(),
                 'test_spawn':self.test_spawn_result}
        output=String();output.data=json.dumps(payload,ensure_ascii=False)
        self.status_pub.publish(output)

    def cleanup(self):
        self.stop_all()
        # Publish repeated zeros before removing models; no robot command is sent.
        for _ in range(4):
            self.stop_all();time.sleep(0.1)
        for ident in list(self.people):
            try:
                self.delete_person(ident)
            except Exception as exc:
                self.get_logger().error(f'Cleanup failed: person {ident}: {exc}. Run recover after relaunch.')
        self.record('shutdown')
        self.pose_stream.close()
        self.log_file.close()


def main():
    # Prevent two managers controlling /model/test_person_XX/cmd_vel simultaneously.
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    lock_path=f'/tmp/serving_robot_person_simulator_{domain}.lock'
    with open(lock_path,'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Person simulator already running in this ROS domain')
        rclpy.init()
        node=None
        try:
            node=Manager()
            while rclpy.ok() and not node.shutdown_requested:
                rclpy.spin_once(node,timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.cleanup()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__=='__main__':
    main()
