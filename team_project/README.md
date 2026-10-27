# 통합 실행

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 환경에서 실행합니다.

```bash
cd team_project
bash scripts/build.sh
bash scripts/fetch_models.sh
bash scripts/start_operation.sh --headless --robot robot_01
# 별도 터미널에서 종료
bash scripts/stop.sh
```

두 로봇은 `--robot` 옵션 없이 실행합니다. 사람 시뮬레이터는 로봇과 별도로 선택 실행합니다. ROS Domain과 Gazebo partition을 로봇 실행과 동일하게 설정해야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID=101 GZ_PARTITION=team_integration_101
ros2 launch serving_robot_person_simulator people.launch.py config:=$PWD/ros_ws/src/serving_robot_person_simulator/config/policy_dual.yaml
# 4가지 사람 행동 혼합 시험은 명시적으로 robot_encounter:=mixed 추가
```

운행 지도는 `maps/stores/demo_store/team_lidar_initial.yaml`과 `.pgm`에 포함됩니다. 기존 0바이트 `map.yaml` 파일은 팀 원본 그대로 보존했습니다. 로컬 YOLO 가중치는 `scripts/fetch_models.sh`가 `_local_only/models/`에 내려받으며 Git에 포함하지 않습니다. 기동 과정에서 `_local_only/config/`에 이 컴퓨터의 절대 경로를 담은 설정이 생성됩니다.

`cafe_autonomy`의 기존 수동 사람 시험 도구 `scripts/person_scenario.py`와 독립형 `serving_robot_person_simulator`는 서로 다른 Gazebo 모델을 조작합니다. 동일 모델 이름을 동시에 사용하지 마세요.

자세한 운영 절차는 [`TEAM_RUNBOOK_KO.md`](TEAM_RUNBOOK_KO.md)를 참고하세요. 새로운 컴퓨터에서는 ROS/Gazebo와 Python 의존성을 설치해야 합니다. 이번 업로드용 클론에서는 8개 ROS 패키지 빌드와 92개 오프라인 테스트가 통과했고, Robot 01의 센서·TF·Nav2·Collision Monitor 기동을 확인했습니다. 발표 준비 요청에 따라 실제 목표 주행 시험은 중단했으므로 새 클론의 전체 서빙 성공으로 기록하지 않습니다.
