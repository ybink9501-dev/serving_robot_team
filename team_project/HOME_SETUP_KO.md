# 집 컴퓨터 개인 자율주행 실행

저장소는 `F:\ROS2\serving_robot_team`에 있으며 WSL에서는 아래 경로로 연다.

```bash
cd /mnt/f/ROS2/serving_robot_team/team_project
source /opt/ros/jazzy/setup.bash
bash scripts/setup_perception.sh
bash scripts/build_autonomy.sh
bash scripts/fetch_models.sh
bash scripts/start_autonomy.sh --headless
```

현재 컴퓨터의 Ubuntu에는 `python3.12-venv`와 `python3-pip`를 설치했다.
다른 컴퓨터에서 가상환경 생성에 실패하면 이 두 패키지가 필요하다.
인식 라이브러리는 `_local_only/venv`에 있으며 GPU가 있으면 자동 사용한다.
실행 화면은 `http://localhost:8765`이다. 화면을 포함한 Gazebo는 `--headless`를 생략한다.

## 동작 구분

- `KITCHEN_PICKUP`: 주방 이동 → `WAIT_LOAD` 적재 확인 → 테이블 이동 →
  `WAIT_HANDOVER` 수령 확인 → 해당 임무 `COMPLETED`.
- `DELIVER_DIRECT`: 테이블 이동 → 수령 확인 → 해당 위치에서 `COMPLETED`.
- `TABLE_COLLECT`: 테이블 이동 → `WAIT_COLLECTION` → 해당 ID의 `COLLECTED` → `COMPLETED`.
- 임무 완료 후 고객 대기열을 우선한다. 대기열이 비면 Fleet가 별도 GO_HOME 요청으로 복귀시킨다.
- `GO_HOME`: 별도의 명시적 HOME 복귀. 두 로봇 검사는 `python3 scripts/trial_go_home.py`.
- 적재·수령 확인은 해당 요청 ID와 실제 대기 위치가 맞을 때만 수락한다.

전체 서빙 시험 도구는 `python3 scripts/trial_serving.py --mode KITCHEN_PICKUP --timeout 420`이다.
실제 Nav2를 사용하며 시험 도구가 적재·수령 버튼 입력만 대신한다.
최신 검증 결과는 `PERSONAL_VALIDATION_20261030.md`를 따른다. 기본 지도는 실제 SLAM 지도다.
`--people`를 시작 명령에 추가하면 사람 manager/bridge/watchdog도 같은 실행에서 관리한다.

## 독립 사람 시뮬레이터

시작 명령에서 --people을 사용하지 않았을 때만 별도 Launch를 실행한다.
--people 사용 시에는 추가 Launch를 생략하고 해당 패키지의 사람 관리 명령만 사용한다.
아래 독립 예제는 기본 데모 장소용이며 다른 장소는 해당 보행 경로 설정을 지정한다.

```bash
cd /mnt/f/ROS2/serving_robot_team/team_project
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID=101 GZ_PARTITION=team_integration_101
ros2 launch serving_robot_person_simulator people.launch.py \
  config:=$PWD/ros_ws/src/serving_robot_person_simulator/config/policy_dual.yaml
```

사람은 명령을 받기 전에는 생성되지 않는다. 기본 동작은 `yield`이며
`robot_encounter:=mixed`로 네 가지 혼합 행동을 선택할 수 있다.
사람 명령·정책은 해당 패키지의 README를 참고한다.

## 종료

독립 사람 Launch는 해당 터미널에서 Ctrl+C로 종료한다. --people 통합 실행은 아래 명령으로 함께 종료한다.

```bash
cd /mnt/f/ROS2/serving_robot_team/team_project
bash scripts/stop_autonomy.sh
```

팀 기본 Launch와 개인 Launch는 서로 다른 지도·TF·명령 규약을 사용한다.
팀 웹 backend가 현재 개인 스택에 연결됐다고 가정하지 않는다.
팀 기본 Launch와 개인 스택을 같은 Gazebo 세션에서 동시에 실행하지 않는다.
