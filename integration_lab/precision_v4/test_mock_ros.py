#!/usr/bin/env python3
"""가짜 ROS(rclpy/Nav2/TF/시계)로 destination_server_lab.py 의 실제 코드를 실행하는 시나리오 테스트.

사용법:  python3 test_mock_ros.py <destination_server_lab.py 경로> [--quiet]
소스 문자열 검사가 아니라 서버 클래스를 실제로 만들고 콜백 순서를 재현한다.
"""
import importlib.util, math, sys, tempfile, time, types
from pathlib import Path
from unittest import mock

# ───────────────────────── 가짜 ROS 모듈 ─────────────────────────
class Future:
    def __init__(self): self._done = False; self._res = None; self._cbs = []
    def add_done_callback(self, cb):
        if self._done: cb(self)
        else: self._cbs.append(cb)
    def set(self, res):
        self._done = True; self._res = res
        for cb in self._cbs: cb(self)
    def result(self): return self._res

class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted; self.cancelled = False; self.result_future = Future()
    def cancel_goal_async(self): self.cancelled = True
    def get_result_async(self): return self.result_future

class Result:
    def __init__(self, status): self.status = status

class FakeActionClient:
    instances = []
    def __init__(self, node, action_type, name):
        self.sent = []; self.ready = True; FakeActionClient.instances.append(self)
    def server_is_ready(self): return self.ready
    def send_goal_async(self, goal):
        fut = Future(); self.sent.append((goal, fut)); return fut

class Pub:
    def __init__(self, log, topic): self.log = log; self.topic = topic
    def publish(self, msg): self.log.append((self.topic, msg.data))

class Param:
    def __init__(self, v): self.value = v

class FakeNode:
    overrides = {}; pubs = []
    def __init__(self, name): pass
    def declare_parameter(self, name, default=None): return Param(FakeNode.overrides.get(name, default))
    def create_publisher(self, t, topic, depth): return Pub(FakeNode.pubs, topic)
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_clock(self):
        n = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None)); return n
    def get_logger(self): return types.SimpleNamespace(debug=lambda *a, **k: None)
    def destroy_node(self): pass

class Msg:  # 중첩 속성을 자유롭게 허용하는 메시지 대역
    def __init__(self, **kw): self.__dict__.update(kw)
    def __getattr__(self, k):
        if k.startswith('__'): raise AttributeError(k)
        v = Msg(); self.__dict__[k] = v; return v

TF = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
class Buffer:
    def lookup_transform(self, a, b, t):
        r = Msg(); r.transform.translation.x = TF['x']; r.transform.translation.y = TF['y']
        r.transform.rotation.x = 0.0; r.transform.rotation.y = 0.0
        r.transform.rotation.z = math.sin(TF['yaw'] / 2); r.transform.rotation.w = math.cos(TF['yaw'] / 2)
        return r

CAPTURED = {}
def install_fakes():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    rclpy = mod('rclpy', init=lambda args=None: None, ok=lambda: False, shutdown=lambda: None,
                spin=lambda node: CAPTURED.__setitem__('node', node))
    rclpy.time = mod('rclpy.time', Time=lambda: None)
    mod('rclpy.node', Node=FakeNode)
    mod('rclpy.action', ActionClient=FakeActionClient)
    mod('std_msgs'); mod('std_msgs.msg', String=lambda data='': Msg(data=data))
    mod('nav2_msgs'); mod('nav2_msgs.action', NavigateToPose=types.SimpleNamespace(Goal=lambda: Msg()))
    mod('action_msgs'); mod('action_msgs.msg', GoalStatus=types.SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
    mod('geometry_msgs'); mod('geometry_msgs.msg', PoseStamped=lambda: Msg())
    mod('nav_msgs'); mod('nav_msgs.msg', Odometry=Msg)
    mod('tf2_ros', Buffer=Buffer, TransformListener=lambda buf, node: None)

    def evaluate_arrival(pose, goal, vx, wz, det, final, yaw_tol, st_lin, st_ang):
        xy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        dyaw = abs((pose[2] - goal[2] + math.pi) % (2 * math.pi) - math.pi)
        stopped = abs(vx) <= st_lin and abs(wz) <= st_ang
        return {'xy_error_m': xy, 'yaw_error_rad': dyaw, 'detected': xy <= det,
                'final_arrived': xy <= final and dyaw <= yaw_tol and stopped}
    mod('arrival_policy', evaluate_arrival=evaluate_arrival)
    def resolve_destination(path, dest):
        import yaml
        wp = yaml.safe_load(Path(path).read_text())['waypoints'][dest]
        return types.SimpleNamespace(**wp)
    mod('navigate_destination', resolve_destination=resolve_destination)

WAYPOINTS = {  # 테스트용 임의 좌표(실제 waypoint 파일이 아님)
    'kitchen': dict(x=-2.0, y=1.0, yaw=1.0), 'home': dict(x=0.0, y=0.0, yaw=0.0),
    'table_01': dict(x=1.0, y=2.0, yaw=0.0), 'table_02': dict(x=2.0, y=2.0, yaw=0.5),
    'table_03': dict(x=3.0, y=-1.0, yaw=-1.0), 'table_04': dict(x=4.0, y=-2.5, yaw=3.0)}
ARRIVAL = {'arrival': dict(detection_radius_m=0.05, final_error_m=0.02, yaw_tolerance_rad=0.1,
                           stopped_linear_mps=0.01, stopped_angular_rps=0.02, final_verify_timeout_s=3.0)}

# ───────────────────────── 시뮬레이터 ─────────────────────────
class Sim:
    def __init__(self, server_path, **params):
        import yaml
        d = Path(tempfile.mkdtemp())
        (d / 'wp.yaml').write_text(yaml.safe_dump({'map_revision': 1, 'waypoints': WAYPOINTS}))
        (d / 'arr.yaml').write_text(yaml.safe_dump(ARRIVAL))
        FakeNode.overrides = dict(waypoints_file=str(d / 'wp.yaml'), arrival_file=str(d / 'arr.yaml'), **params)
        FakeNode.pubs = []; FakeActionClient.instances = []; CAPTURED.clear()
        spec = importlib.util.spec_from_file_location('dest_server_under_test', server_path)
        m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); m.main([])
        self.node = CAPTURED['node']; self.cli = FakeActionClient.instances[-1]
        self.t = 1000.0; self.handles = []
        self.patch = mock.patch('time.monotonic', lambda: self.t); self.patch.start()
        self.node.on_odom(Msg(twist=Msg())); self.odom(0.0, 0.0)
        TF.update(x=0.0, y=0.0, yaw=0.0)
    def close(self): self.patch.stop()
    def odom(self, vx, wz):
        o = Msg(); o.twist.twist.linear.x = vx; o.twist.twist.angular.z = wz; self.node.on_odom(o)
    def cmd(self, text): self.node.command(Msg(data=text))
    def go(self, dest): self.cmd('{"destination_id":"%s","map_revision":1}' % dest)
    def cancel(self): self.cmd('{"command":"CANCEL"}')
    def tick(self, dt=0.2): self.t += dt; self.node.tick()
    def run(self, seconds, pose_fn=None, vx=0.0, wz=0.0):
        n = int(round(seconds / 0.2)); t0 = self.t
        for _ in range(n):
            if pose_fn: TF.update(pose_fn(self.t - t0 + 0.2))
            self.odom(vx, wz); self.tick()
    def accept(self, idx):
        goal, fut = self.cli.sent[idx]; h = Handle(True); self.handles.append(h); fut.set(h); return h
    def succeed(self, h): h.result_future.set(Result(4))
    def cancel_result(self, h): h.result_future.set(Result(5))
    @property
    def goals(self): return len(self.cli.sent)
    @property
    def states(self): return [json_load(d)['state'] for t, d in FakeNode.pubs if t.endswith('robot_state')]
    @property
    def events(self): return [json_load(d)['event'] for t, d in FakeNode.pubs if t.endswith('navigation_event')]
    def event_data(self, name):
        return [json_load(d) for t, d in FakeNode.pubs if t.endswith('navigation_event') and json_load(d)['event'] == name]
    @property
    def live(self): return [h for h in self.handles if h.accepted and not h.cancelled and not h.result_future._done]

def json_load(s):
    import json; return json.loads(s)

def at(dest, dx=0.0, dy=0.0):
    w = WAYPOINTS[dest]; return dict(x=w['x'] + dx, y=w['y'] + dy, yaw=w['yaw'])

def reach_verify_phase(sim, dest, error):
    """GO → 수락 → Nav2 SUCCEEDED → 목표에서 error(m)만큼 떨어져 정지한 상태."""
    sim.go(dest); h = sim.accept(0); sim.succeed(h); TF.update(at(dest, error)); return h

# ───────────────────────── 시나리오 ─────────────────────────
SCENARIOS = []
def scenario(name, requirement):
    def deco(fn): SCENARIOS.append((name, requirement, fn)); return fn
    return deco

@scenario('A1 테이블 1~4 목표 좌표 정확성', '테이블 1~4 각각 자기 좌표로 Goal 1회, 도착 시 SERVING')
def a1(path):
    for tb in ['table_01', 'table_02', 'table_03', 'table_04']:
        s = Sim(path); s.go(tb); g = s.cli.sent[0][0].pose.pose
        ok = abs(g.position.x - WAYPOINTS[tb]['x']) < 1e-9 and abs(g.position.y - WAYPOINTS[tb]['y']) < 1e-9
        h = s.accept(0); s.succeed(h); TF.update(at(tb, 0.005)); s.run(1.0)
        res = ok and s.goals == 1 and s.states[-1] == 'SERVING'; s.close()
        if not res: return False, f'{tb} 실패'
    return True, '4개 테이블 모두 Goal 1회로 SERVING'

@scenario('A2 부엌 경유 후 테이블 (서버 단독 2구간)', '서버가 연속된 두 목적지 요청을 순서대로 처리')
def a2(path):
    s = Sim(path); s.go('kitchen'); h = s.accept(0); s.succeed(h); TF.update(at('kitchen', 0.004)); s.run(1.0)
    first = s.states[-1]
    s.go('table_02'); g = s.cli.sent[1][0].pose.pose; h2 = s.accept(1); s.succeed(h2); TF.update(at('table_02', 0.004)); s.run(1.0)
    ok = first == 'SERVING' and g.position.x == 2.0 and s.states[-1] == 'SERVING' and s.goals == 2
    s.close(); return ok, '부엌 SERVING → 테이블 2 SERVING (경유 순서 지시는 Supervisor 몫, 여기선 미검증)'

@scenario('B1 정밀 보정 1회 성공', '3cm 오차 → 보정 Goal 1회 재전송 → 2cm 이내 도착')
def b1(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(3.4)
    if s.goals != 2: return False, f'보정 Goal 수={s.goals}'
    h2 = s.accept(1); s.succeed(h2); TF.update(at('table_03', 0.008)); s.run(1.0)
    ok = s.states[-1] == 'SERVING' and s.goals == 2; s.close(); return ok, f'총 Goal {s.goals}회, 최종 {s.states[-1]}'

@scenario('B2 보정이 개선 없으면 중단', '재전송 후 1mm 이상 개선 없음 → 세 번째 Goal 없이 ERROR')
def b2(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(3.4)
    h2 = s.accept(1); s.succeed(h2); TF.update(at('table_03', 0.03)); s.run(3.4)
    ok = s.goals == 2 and s.states[-1] == 'ERROR'
    reason = s.event_data('FINAL_ACCURACY_NOT_REACHED'); s.close()
    return ok, f'총 Goal {2 if ok else "≠2"}회, 사유={reason[-1]["reason"] if reason else None}'

@scenario('B3 재전송 횟수 상한', '계속 조금씩 개선돼도 재전송은 최대 2회(총 3 Goal)에서 끝')
def b3(path):
    s = Sim(path); reach_verify_phase(s, 'table_01', 0.08); s.run(3.4)
    h2 = s.accept(1); s.succeed(h2); TF.update(at('table_01', 0.05)); s.run(3.4)
    h3 = s.accept(2); s.succeed(h3); TF.update(at('table_01', 0.03)); s.run(3.4)
    ok = s.goals == 3 and s.states[-1] == 'ERROR'; g = s.goals; s.close(); return ok, f'총 Goal {g}회 후 ERROR'

@scenario('C1 보정 중 CANCEL 이 실제로 멈추는가', 'CANCEL 후 살아 있는 Nav2 Goal 0개, 상태 PAUSED')
def c1(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(3.4)   # 재전송 Goal 발행됨(수락 대기)
    s.cancel(); h2 = s.accept(1)                                          # 취소 후에 수락됨
    live = len(s.live)
    if h2.cancelled: s.cancel_result(h2)
    st = s.states[-1]; s.close(); return live == 0 and st == 'PAUSED', f'취소 후 살아 있는 Goal={live}, 상태={st}'

@scenario('C2 정밀도 검증 대기 중 CANCEL', 'CANCEL 후 보정 Goal 재전송이 일어나지 않음')
def c2(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(1.0); s.cancel(); s.run(8.0)
    ok = s.goals == 1 and s.states[-1] == 'PAUSED'; g = s.goals; st = s.states[-1]; s.close()
    return ok, f'총 Goal {g}회, 상태={st}'

@scenario('D1 제자리 회전 중단 후 고아 Goal 없음', '정체 감시 작동 후 수락된 재전송 Goal 취소')
def d1(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(3.4)    # retry_pending
    s.run(25.0, pose_fn=lambda t: dict(x=3.03, y=-1.0, yaw=0.5 * t), wz=1.0)   # 제자리 회전
    if s.states[-1] != 'ERROR': return False, f'정체 감시 미작동, 상태={s.states[-1]}'
    s.accept(1); live = len(s.live); s.close(); return live == 0, f'ERROR 이후 살아 있는 Goal={live}'

@scenario('D2 취소 결과가 ERROR 를 덮어쓰지 않음', '정체 감시 → 취소 결과 도착 후에도 상태 ERROR 유지')
def d2(path):
    s = Sim(path); s.go('table_03'); h = s.accept(0)
    s.run(25.0, pose_fn=lambda t: dict(x=0.0, y=0.0, yaw=0.5 * t), wz=1.0)
    if s.states[-1] != 'ERROR': return False, f'정체 감시 미작동, 상태={s.states[-1]}'
    cancelled = h.cancelled; s.cancel_result(h); st = s.states[-1]; s.close()
    return cancelled and st == 'ERROR', f'Goal 취소 요청={cancelled}, 취소 결과 후 상태={st}'

@scenario('D3 이전 임무의 재전송 Goal 이 새 임무에 섞이지 않음', '새 임무 Goal 만 살아 있음')
def d3(path):
    s = Sim(path); reach_verify_phase(s, 'table_03', 0.03); s.run(3.4)
    s.run(25.0, pose_fn=lambda t: dict(x=3.03, y=-1.0, yaw=0.5 * t), wz=1.0)  # ERROR (이전 임무)
    s.go('table_01')                                                           # 새 임무 (Goal idx 2)
    s.accept(1)                                                                # 이전 임무의 지연된 재전송 수락
    h_new = s.accept(2)
    live = s.live; ok = live == [h_new]; n = len(live); s.close()
    return ok, f'살아 있는 Goal={n}개, 새 임무 Goal만 유지={ok}'

@scenario('E1 임무 전체 시간 제한', '진행이 있어도 한도 초과 시 취소하고 ERROR(MISSION_TIMEOUT)')
def e1(path):
    s = Sim(path, mission_timeout_s=60.0); s.go('table_03'); h = s.accept(0)
    s.run(70.0, pose_fn=lambda t: dict(x=0.05 * t, y=0.0, yaw=0.0), vx=0.05)
    ok = s.states[-1] == 'ERROR' and h.cancelled; st = s.states[-1]; s.close()
    return ok, f'70초 후 상태={st}, Goal 취소={h.cancelled}'

@scenario('F1 이동하며 회전은 정체로 오판하지 않음', '60초간 진행 + 회전 → 중단 없음')
def f1(path):
    s = Sim(path); s.go('table_03'); s.accept(0)
    s.run(60.0, pose_fn=lambda t: dict(x=0.05 * t, y=0.0, yaw=0.5 * t), vx=0.05, wz=0.5)
    ok = s.states[-1] == 'NAVIGATING'; st = s.states[-1]; s.close(); return ok, f'상태={st}'

@scenario('F2 제자리 회전은 20초 부근에서 중단', '정체 감시 작동 시 취소 요청')
def f2(path):
    s = Sim(path); s.go('table_03'); h = s.accept(0)
    s.run(25.0, pose_fn=lambda t: dict(x=0.0, y=0.0, yaw=0.5 * t), wz=1.0)
    ok = s.states[-1] == 'ERROR' and h.cancelled; r = s.event_data('FINAL_ACCURACY_NOT_REACHED'); s.close()
    return ok, f'사유={r[-1]["reason"] if r else None}, Goal 취소={h.cancelled}'

def main():
    if len(sys.argv) < 2: sys.exit(__doc__)
    path = str(Path(sys.argv[1]).resolve()); install_fakes()
    passed = 0
    print(f'대상 파일: {path}\n')
    for name, req, fn in SCENARIOS:
        try: ok, detail = fn(path)
        except Exception as e: ok, detail = False, f'예외 {type(e).__name__}: {e}'
        passed += bool(ok)
        print(f'{"PASS" if ok else "FAIL"}  {name}\n      요구: {req}\n      결과: {detail}')
    print(f'\n합계 {passed}/{len(SCENARIOS)} 통과')
    sys.exit(0 if passed == len(SCENARIOS) else 1)

if __name__ == '__main__':
    main()
