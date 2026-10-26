# Robot 01 카페 LAB 추가 코드 (검토용)

기존 TEAM 소스, `destination_server.py`, LAB Nav2 설정, 실행 중인 ROS/Gazebo 노드는 변경하지 않음.

## 포함

- `build_cafe_map.py`: SDF에 직접 포함된 box/cylinder/sphere **충돌 형상**을 2D로 투영한 임시 지도(`.pgm/.yaml`) 및 **미승인** 정차 후보 생성. Gazebo 화면의 시각 요소만으로 정확한 지도는 만들 수 없음.
- `validate_waypoints.py`: 사용자가 검토해 작성한 waypoint YAML의 지도 충돌 및 테이블을 향한 방향 검사.
- `serving_flow.py`: 부엌→테이블→수령 완료→부엌 상태 전환. ROS와 분리된 테스트 가능한 순수 로직.
- `serving_supervisor.py`: 수동 완료용 프론트 연동 지점과 LAB 자동 완료. **기존 destination_server에서 실제 도착 성공이 먼저 발생해야 다음 단계로 감.**
- `tests/test_lab.py`: 순수 Python 기반 기본 테스트.

## 준비

```bash
export ROS_DOMAIN_ID=99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
cd ~/team_robot_lab/lab_scripts/cafe_lab_additions
python3 -m pytest -q tests/test_lab.py
python3 build_cafe_map.py
```

기본 SDF 경로: `~/team_serving_robot/ros_ws/src/serving_robot_gazebo/worlds/cafe.world.sdf`.
기본 출력 위치: `~/team_robot_lab/lab_maps/generated/`.
기존 `cafe_clean_step96.yaml`을 덮어쓰지 않음.

**주의:** SDF 외부 include/mesh, 회전 pitch/roll, 동적 사람·로봇은 자동 지도에 반영되지 않거나 근사될 수 있음. 출력 리포트의 `warnings`, 건물 벽, 부엌 및 4개 테이블, Nav2 좌표 일치 검증 전까지 이 지도를 **실제 Nav2에 적용하지 말 것**. `cafe_waypoint_candidates.json`은 추천 좌표일 뿐이고 실제 서버용 YAML로 자동 등록하지 않음. 테이블 앞 정차·접근 방향·차체 간섭은 Gazebo에서 확인해야 함.

## 수동으로 승인한 정차 위치

기존 `arrival_test_waypoints.yaml`은 건드리지 않고 **별도 승인된 YAML**을 작성해야 함. 예:

```yaml
map_revision: 2
waypoints:
  - id: wp_kitchen
    entity_id: kitchen
    pose: {x: ..., y: ..., yaw: ...}
  - id: wp_table_01
    entity_id: table_01
    pose: {x: ..., y: ..., yaw: ...}
```

확정 전에는 위 `...` 자리에 임의 좌표를 넣지 않음. 현재 TEAM 파일은 수정 금지.

지도와 목표 좌표가 검증된 이후:

```bash
python3 validate_waypoints.py \
  --map-yaml ~/team_robot_lab/lab_maps/generated/cafe_full_provisional.yaml \
  --waypoints ~/team_robot_lab/lab_configs/cafe_serving_verified.yaml \
  --layout-json ~/team_robot_lab/lab_maps/generated/cafe_layout_report.json
```

## 서빙 감독 노드 (위 준비가 끝난 이후에만 실행)

기존 destination_server도 **같은 승인 waypoint YAML, map_revision**을 사용해 별도로 실행 중이어야 함. 기존 노드가 테스트용 예전 파일을 읽고 있다면 아래 감독 노드를 실행하지 말 것.

```bash
cd ~/team_robot_lab/lab_scripts/cafe_lab_additions
python3 serving_supervisor.py --ros-args \
  -p robot_id:=robot_01 \
  -p waypoints_file:=$HOME/team_robot_lab/lab_configs/cafe_serving_verified.yaml \
  -p map_revision:=2 \
  -p kitchen_destination:=kitchen \
  -p completion_mode:=manual
```

- **서빙 시작:** `/robot_01/serving/request` `std_msgs/String` JSON `{"command":"START","table":"table_01"}`
- **프론트/수동 서빙완료:** `/robot_01/serving/complete` `std_msgs/String` JSON `{"command":"COMPLETE"}`
- **상태 표시:** `/robot_01/serving/status` `std_msgs/String` JSON. 향후 프론트에서 해당 두 토픽을 연결할 수 있음.
- **자동 테스트 모드:** `-p completion_mode:=auto -p auto_wait_s:=10.0` (도착 확인 후 10초, 기본값은 **manual**)
- **중단:** `/robot_01/serving/request` JSON `{"command":"CANCEL"}`

주의: 기존 `destination_server.py`는 `SERVING` 전환에 **Nav2 성공 + 위치 1mm + 방향/정지**를 모두 요구함. 검사 실패 시 자동완료로 우회하지 않음. `HOME`은 부엌과 별도 대기 지점으로 유지되며 이 왕복 과정에서 사용하지 않음.

## 로봇 2 / 사람 시험

이번 추가 코드는 robot_id 파라미터로 분리할 수 있도록 작성되었으나, 현재 사용 중인 실물 TF 프레임 `base_footprint`는 Robot 01 기준임. Robot 02는 개별 TF 접두어·센서·Safety·목표 Action 및 실제 모델 복제 검증 전까지 스폰·운행하지 않음. 사람 시뮬레이션 노드 실행 여부는 사람의 실제 움직임이나 회피 성능 검증이 아님.

## 읽기 전용 통합 준비 검사 및 시나리오 목록

- `integration_readiness.py` : 현재 `/robot_01/` ROS 노드·토픽·TF와 사람 시뮬레이터 노드가 보이는지 검사, Robot 02가 아직 분리되지 않았는지 검사. **어떤 이동 명령도 보내지 않음**.
- `scenario_checklist.yaml` : 부엌 왕복, 수동/자동 완료, 4개 테이블, 사람 회피, Robot 02 주행 안전 시험의 항목별 결과 기록 양식. 모두 `pass: null`에서 시작함.

```bash
python3 integration_readiness.py
```
