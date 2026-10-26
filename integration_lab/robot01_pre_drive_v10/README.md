# Robot 01 선행작업 V10 — 순서 분리

팀 `cafe.world.sdf` + SDF와 원점이 일치하는 LAB 지도 전용. 개인 카페 지도 미사용.
설치 대상: `~/team_robot_lab/robot01_pre_drive_v10/`; `~/team_serving_robot` 팀 원본과 V3~V9 파일을 수정하지 않음.

## 1, 2단계 — 픽업 후보·위치추정 (다음 실행)

**정적 검사** (로봇 이동 없음, ROS 필요 없음):

```bash
python3 ~/team_robot_lab/robot01_pre_drive_v10/setup_gate.py
```

현재 SDF 카운터 충돌체와 로봇 반경 0.35m을 비교해 `counter_pickup` 위치에서의 물리 여유와 카운터를 바라보는 yaw를 계산하고, V3 layout/manifest/world/map/waypoints SHA를 검증함. 현재 설치된 SDF가 소스 스냅샷과 같으면 목표(-2.3,1.45), 카운터 앞 경계 y=2.05m, 충돌체까지 0.60m, 로봇 중심 반경 제외 0.25m를 예상. **팀의 최종 픽업 위치 확정은 별도 승인 필요**. 실제 이동 검증은 나중에.

**ROS 연결 상태에서 현재 AMCL 값 조회 (읽기 전용):**

```bash
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
source ~/team_robot_lab/ros_ws/install/local_setup.bash
python3 ~/team_robot_lab/robot01_pre_drive_v10/setup_gate.py --live-readonly
```

`ready:true`면 현재 AMCL 위치가 Gazebo 실제 위치 기준 0.15m/12° 이내라는 의미. **V8이 `READ_ONLY`로 0 종료하더라도 위치가 어긋나면 V10에서는 ready:false 처리.**

**어긋난 경우에만**, 로봇이 정지하고 임무 취소된 것을 확인한 뒤 다음을 명시적으로 실행:

```bash
python3 ~/team_robot_lab/robot01_pre_drive_v10/setup_gate.py --recover
```

이는 기존 V8 안전 가드를 재사용하며 `/robot_01/initialpose`를 발행할 수 있음. ROS 목표/속도 명령 및 팀 원본 수정을 하지 않음. `ready: true`를 확인하기 전 주행 금지. V8 재초기화는 **Gazebo 시뮬레이션 한정**.

## 3단계 — 정체 감시 (1, 2단계 통과 뒤)

V7은 이미 과도한 회전을 감시함. 신규 V10은 **30초 동안 총 이동량 0.15m 미만이고 순이동 0.08m 미만**인 정체만 감시. `GOING_PICKUP`, `GOING_TABLE`, `RETURNING_PICKUP`에서만 작동하며 명령값 분포로 `SAFETY_CHAIN_BLOCKING_TRANSLATION`, `CONTROLLER_NO_TRANSLATION_REQUEST` 등을 구분해 로그 작성. 다른 속도 발행자는 추가하지 않음.

처음엔 관찰 모드로 실행하여 로그 확인:

```bash
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
LOG="/tmp/robot01_stall_guard_v10_$(date +%Y%m%d_%H%M%S).jsonl"
python3 -u ~/team_robot_lab/robot01_pre_drive_v10/stall_guard.py --out "$LOG"
```

관찰 로직을 검증한 후 기존 V7과 중복 실행하지 않는지를 확인한 다음 `--enforce`를 명시적으로 추가하면 V3 `CANCEL`만 비동기 발행할 수 있음. 미션 취소가 즉시 물리 정지를 보장하지 않음. ROS 배선과 실제 노드 동작은 아직 WSL 검증 전.

## 4단계 — 기록/판정 (3단계 구현 적용 뒤)

```bash
OUT="/tmp/robot01_evidence_v10_$(date +%Y%m%d_%H%M%S).jsonl"
python3 -u ~/team_robot_lab/robot01_pre_drive_v10/evidence_recorder.py --duration 600 --out "$OUT"
python3 ~/team_robot_lab/robot01_pre_drive_v10/report.py "$OUT"
```

보고서 분류: `NO_EVIDENCE`, `INCOMPLETE`, `PICKUP_WAIT_LOAD_OBSERVED`, `FAILED_OR_INTERRUPTED`, `SERVING_SEQUENCE_COMPLETED_REVIEW_PHYSICAL_METRICS`. `PUBLISHED`를 서빙 성공으로 보지 않으며 1mm 도착 오차도 임의 완화하지 않음. 새 시험마다 새 파일을 사용해야 이전 상태와 혼동되지 않음.

## 테스트

```bash
python3 -m unittest discover -s ~/team_robot_lab/robot01_pre_drive_v10/tests -v
```

오프라인 정책·SDF 기하 테스트만 수행. WSL에서 실제 ROS/Gazebo 효과는 **위 단계별 확인 전 미검증**.

## 고정 경계
- Robot 01의 기존 Safety 속도체인은 우회하지 않음.
- 후진 허용, 각속도 조정, Nav2 파라미터 변경 없음.
- 지도 원본 및 팀원 매핑 placeholder 수정 없음.
- `counter_pickup` 현 위치는 LAB 후보이며 최종 운영 픽업 위치로 자동 승인하지 않음.
- 로봇 주행은 V3를 통해서만 시작하며 이 묶음에는 `START` 명령 없음.
