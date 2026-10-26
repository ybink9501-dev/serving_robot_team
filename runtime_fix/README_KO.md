# 팀 카페 자율주행 — 실행 스크립트 오류 수정 통합 소스 (2026-10-22)

## 이번 WSL 실측 로그에서 발견된 직접 원인 (15:06 기준)

- `tools/preflight.sh`, `tools/run_all.sh`, `tools/check.sh`가 ROS 설정 파일을 `set -u`(nounset) 활성 상태에서 `source`하여 `/opt/ros/jazzy/setup.bash: 줄 8: AMENT_TRACE_SETUP_FILES: 바인딩 해제한 변수`로 종료했습니다.
- `tools/save_mapping_map.sh`에도 동일한 잠재 오류가 있어 함께 수정했습니다. 모든 스크립트는 ROS 환경 로딩 중에만 nounset을 해제하고 즉시 복원합니다.
- 서버 검사에서 `gz sim gui`와 Gazebo Ruby wrapper를 서버로 잘못 계수하는 문제를 수정했습니다. 실제 `gz sim server` 프로세스 한 개만 검사합니다.
- 기존 코드/지도/TEAM·LAB/개인 프로젝트를 수정하거나 삭제하지 않습니다. 신규 폴더에 별도 실행합니다.
- **오프라인 20개 테스트 통과**(기존 19개 + ROS 시작 스크립트 재현 시험). 실제 WSL/Gazebo 가동·임무 성공은 아직 미검증입니다.

## 실행 전 확인

이전 Robot01 및 Gazebo Launch 터미널을 `Ctrl+C`로 각각 정상 종료하십시오. 임의의 광범위한 `pkill`/`rm -rf`는 사용하지 않습니다. 기존 서버가 남아 있으면 `run_all.sh`가 거부하는 것이 정상입니다.

```bash
pgrep -af '^gz sim server($|[[:space:]])'
```

기존 서버가 없다면 아래 새 버전을 압축 해제하십시오. **이전 폴더에 `unzip -n`으로 업데이트하면 수정이 반영되지 않습니다.**


**프로젝트 정체성:** 팀 MAPPING → SETUP → OPERATION MVP. 개인 카페 프로젝트는 참고기록이며 소스를 그대로 이식하지 않았습니다. `~/team_serving_robot`, `~/team_robot_lab`, `~/cafe_robot_ws`, 새 Git clone을 수정·삭제하지 않습니다. 이 ZIP은 기존에 잘못 작성됐던 TEAM_FINAL 재구성본의 **독립 수정판**입니다. 원래 10월 19일 TEAM+LAB 소스의 바이트 단위 복원본은 아닙니다.

## 이번 수정 범위

- Nav2 YAML에 들어가 ROS2 파서가 실제로 거부한 앵커/alias를 양쪽 로봇에서 제거
- 로봇 두 대 사이의 충돌 판정을 서로 다른 odom 원점이 아닌 **공통 AMCL map 좌표**로 계산. map/odom 수신이 끊기면 안전 정지
- 최신 요청대로 **XY 10cm 이내**, 테이블을 바라보는 **위·아래 접근점 ±90°**, yaw 오차 0.25rad 유지. 수치는 `config/runtime_limits.yaml`을 통해 변경 가능
- `Navigation ABORTED`를 임의로 도착 성공 처리하는 경로 제거, Nav2 성공 후 필요할 때만 미세 접근
- `ERROR` 후 REST/API 또는 Fleet 요청 `RESET`으로 명시적으로 복구 가능 (기존 실패 Task는 FAILED 기록 유지하고 새 ID로 재시도)
- Gazebo 중복 서버 확인, Fleet/Robot 01·02/시뮬레이션 데이터 독립성, 주요 정량 설정 대조 테스트

## 실제 테스트 (기존 프로젝트와 충돌 방지)

**기존 Gazebo/ROS를 실행하고 있는 터미널은 Ctrl+C로 종료한 다음** 새 WSL 터미널에서 시작합니다. 넓은 범위의 `pkill -9`, `rm -rf`, 기존 지도 복사/덮어쓰기는 하지 않습니다.

```bash
cd ~
unzip -n /mnt/c/Users/user/Downloads/cafe_autonomy_TEAM_RUNTIMEFIX_20261022.zip
cd ~/cafe_autonomy_TEAM_RUNTIMEFIX_20261022
bash tools/preflight.sh
source /opt/ros/jazzy/setup.bash
cd ros_ws && colcon build --symlink-install --base-paths src && cd ..
bash tools/run_all.sh
```

새 터미널에서:

```bash
export ROS_DOMAIN_ID=99 GZ_PARTITION=team_robot_lab_gazebo_99
cd ~/cafe_autonomy_TEAM_RUNTIMEFIX_20261022
bash tools/check.sh
```

필수 검사 통과 전 주행 명령을 보내지 않습니다. 통과 후 별도 터미널에서:

```bash
python3 tools/trial.py robot_01 table_01 --timeout 300
python3 tools/trial.py robot_02 table_02 --timeout 300
```

각각 부엌 LOAD → 테이블 COMPLETE → HOME와 Gazebo 실제 위치/방향을 시험합니다. **테이블 접근점은 위/아래에서 테이블을 바라보는 ±90° yaw.** `--mission-only`는 주행 상태만 확인하며 물리 정확도 인증에 사용하지 않습니다.

## 하나의 설정 파일로 수치 조정

`ros_ws/src/cafe_autonomy/config/runtime_limits.yaml`의 `navigation`, `arrival`, `safety`, `fleet` 값을 사용합니다. `tools/configure_runtime.py`는 값 검증·양쪽 Nav2 YAML 동기화를 수행합니다. 예:

```bash
python3 tools/configure_runtime.py --check
python3 tools/configure_runtime.py --dry-run --set arrival.xy_tolerance_m 0.10
python3 tools/configure_runtime.py --set arrival.xy_tolerance_m 0.10
```

수정 후 **재빌드·재시작**이 필요합니다. Nav2 정차 허용오차(10cm)를 따로 바꾸려면 `navigation.goal_xy_tolerance_m`도 변경해야 하며, config는 Nav2 허용오차가 수용 기준보다 클 경우 거부합니다. 안전거리 2.5m는 *별도 설계 요구*로 기록되어 있고 사람 SLOW/STOP (1.5/0.8m)과 임의로 동일시하지 않습니다.

## 지도 및 팀 계약 보호

- 팀 Git의 `maps/stores/demo_store/map.pgm`은 사용자 로그에서 **0바이트**. 사용자의 LAB SLAM 원본, TEAM 공식 지도와 동일하다고 선언하지 않습니다.
- 이 소스에 포함된 `maps/cafe.pgm`은 **재구성 당시 임시 지도**입니다. 실제 LiDAR SLAM으로 취득한 Raw Map이 아닙니다. Mapping 성과물로 제출하면 안 됩니다.
- 별도 MAPPING 모드에서는 SLAM만 map→odom을 발행하고 OPERATION 모드에서는 AMCL만 발행하도록 구분해야 합니다. 같은 ROS_DOMAIN_ID에서 두 모드를 동시에 구동하지 않습니다.
- Semantic waypoint는 별도 좌표 설정 파일이며 Gazebo Ground Truth를 실제 인지 결과로 사용하지 않습니다.
- 최종 속도 채널: Nav2 → velocity smoother → semantic safety → collision monitor → Gazebo. 직접 base 속도 명령 발행을 금지합니다.

## 검증 한계

이 파일은 **코드·설정 오류 수정과 오프라인 검사를 거친 소스 제공물**입니다. 여기에는 ROS 2 Jazzy/Gazebo 실행 환경이 없어 실제 WSL 빌드 및 로봇 01·02 주행·사람 검출·실물 10cm 도착을 이 환경에서 실행하지 못했습니다. 따라서 '실주행까지 증명된 완성본'으로 표시하지 않습니다. `tools/check.sh`, `tools/trial.py`의 실제 실행이 요구됩니다.

## MAPPING / SETUP / OPERATION 모드 구분

이 ZIP은 운영 시험을 우선하고, 별도로 **Robot 01 Assisted Mapping 실행용** `02_mapping_robot.launch.py`를 포함합니다. 세계를 먼저 `01_world.launch.py`로 시작합니다. **MAPPING과 OPERATION을 동시에 실행하면 안 됩니다.**

```bash
# MAPPING 전용: 01_world 터미널 유지, 별도 터미널에서 아래 실행
ros2 launch cafe_autonomy 02_mapping_robot.launch.py
# 가상 LiDAR 스캔을 보며 수동 보조 주행. 속도 명령은 /robot_01/cmd_vel_semantic 으로 입력하여 Collision Monitor를 통과시킴
# 별도 터미널에서 Raw Map 저장
bash tools/save_mapping_map.sh
```

SETUP 단계에서는 저장한 Raw Map을 검사하고 `config/waypoints.yaml`의 의미 목적지 좌표를 해당 Map에 맞게 설정합니다. 제공된 임시 카페 좌표를 공식 Semantic 데이터로 간주하지 않습니다. OPERATION은 별도 새 세션에서 시작하고, `02_robots.launch.py map_yaml:=/절대경로/실제맵.yaml` 인자로 저장한 지도를 명시적으로 지정할 수 있습니다. `run_all.sh`에는 기본 임시 지도가 사용되므로 실제 Mapping/Setup 검증 시에는 직접 해당 Launch 인자를 적용하는 실행 구성이 필요합니다.

도착 정확도 수용시험은 **테이블 실제 위치·방향, 부엌(적재형), HOME 실제 위치·방향**을 각각 측정하며, 이 세 단계 가운데 하나라도 수용 기준을 넘으면 실패로 종료합니다. 원본 Git/슬램 Raw Map을 강제 교체하지 않고 `export CAFE_MAP_YAML=/절대경로/저장지도.yaml`로 검증된 지도를 지정할 수 있습니다. `config/zones.yaml`은 별도 SETUP 조정 파일이며, 승인되지 않은 샘플 좌표입니다.

장기 Zone 대기 시에는 `fleet.blocked_alert_s` 기준으로 `FLEET_TRAFFIC_BLOCKED` 이벤트를 기록합니다. 실제 차체가 Zone에 남아 있는 한 안전하지 않은 예약 강제 해제를 하지 않습니다.
