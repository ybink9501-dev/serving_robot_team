# 개인 자율주행 실행 (집 컴퓨터 재검증 전)

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 환경에서 실행합니다.

```bash
cd team_project
bash scripts/build_autonomy.sh
bash scripts/fetch_models.sh
bash scripts/start_autonomy.sh --headless --robot robot_01
# 별도 터미널에서 종료
bash scripts/stop_autonomy.sh
```

두 로봇은 `--robot` 옵션 없이 실행합니다. 사람 시뮬레이터는 로봇과 별도로 선택 실행합니다. ROS Domain과 Gazebo partition을 로봇 실행과 동일하게 설정해야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID=101 GZ_PARTITION=team_integration_101
ros2 launch serving_robot_person_simulator people.launch.py config:=$PWD/ros_ws/src/serving_robot_person_simulator/config/policy_dual.yaml
# 4가지 사람 행동 혼합 시험은 명시적으로 robot_encounter:=mixed 추가
```

개인 자율주행은 기존 `team_lidar_initial` 지도와 `config/integration` 좌표를 사용합니다. 최신 팀 `demo_store/map.yaml` 및 `config/common` 좌표와 다르므로 팀 기본 Launch와 동시에 실행하지 마세요. 최신 팀 World와의 동작은 집 컴퓨터에서 다시 검증해야 합니다. 로컬 YOLO 가중치는 `scripts/fetch_models.sh`가 `_local_only/models/`에 내려받으며 Git에 포함하지 않습니다. 기동 과정에서 `_local_only/config/`에 이 컴퓨터의 절대 경로를 담은 설정이 생성됩니다.

`cafe_autonomy`의 기존 수동 사람 시험 도구 `scripts/person_scenario.py`와 독립형 `serving_robot_person_simulator`는 서로 다른 Gazebo 모델을 조작합니다. 동일 모델 이름을 동시에 사용하지 마세요.

자세한 운영 절차는 [`TEAM_RUNBOOK_KO.md`](TEAM_RUNBOOK_KO.md)를 참고하세요. 새로운 컴퓨터에서는 ROS/Gazebo와 Python 의존성을 설치해야 합니다. 이번 업로드용 클론에서는 8개 ROS 패키지 빌드와 92개 오프라인 테스트가 통과했고, Robot 01의 센서·TF·Nav2·Collision Monitor 기동을 확인했습니다. 발표 준비 요청에 따라 실제 목표 주행 시험은 중단했으므로 새 클론의 전체 서빙 성공으로 기록하지 않습니다.

## 집 컴퓨터로 가져오기

전체 Git 이력이 필요하지 않다면 `git clone --depth 1`로 내려받을 수 있습니다. 현재 Git에 포함된 사람 시각 메시(약 4.4MB)는 카메라 검출 시험에 필요하므로 유지합니다. `ros_ws/build`, `ros_ws/install`, `ros_ws/log`, `_local_only`는 Git에서 제외되며 새 컴퓨터에서 빌드·모델 다운로드 시 다시 생성됩니다.

현재 통합 실행 진입점은 `scripts/start_autonomy.sh`입니다. `serving_robot_navigation`의 옛 LAB 전용 Launch와 `check_safety_timeout.py`에는 `team_robot_lab` 경로가 남아 있어 이 진입점에서 사용하지 않습니다. 팀 원본 자료로 보존했으며 집 컴퓨터의 통합 실행 명령으로 사용하지 마세요.
