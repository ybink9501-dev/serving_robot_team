# 사람 시뮬레이터

Robot 01·02와 독립적으로 실행하는 Gazebo 시험 도구입니다. 사람 모델을 생성·이동·정지·삭제하고, 사람 간 거리 유지와 선택적 교착 해소를 수행합니다. 로봇 Nav2, Safety, TF 또는 로봇 속도 명령을 직접 제어하지 않습니다.

## 실행

통합 프로젝트의 team_project 디렉터리에서 ROS 2 Jazzy와 Gazebo Harmonic을 준비합니다. 로봇 통합 실행과 같은 ROS_DOMAIN_ID 및 GZ_PARTITION을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID=101 GZ_PARTITION=team_integration_101
ros2 launch serving_robot_person_simulator people.launch.py \
  config:=$PWD/ros_ws/src/serving_robot_person_simulator/config/policy_dual.yaml
```

policy.yaml은 Robot 01 단독, policy_dual.yaml은 Robot 01·02 위치를 함께 확인합니다. 기본 robot_encounter:=yield는 기존 양보 정책입니다. 혼합 행동 시험은 robot_encounter:=mixed를 명시하며 기다리기·막기·차선 내 후진·측면 피하기를 각 25%로 선택합니다. 사람의 후진은 로봇 후진 금지와 별개입니다. 선택적 사람 간 교착 해소는 deadlock_resolution:=true로 켭니다. 기본값은 꺼짐입니다.

## 명령과 설정

명령은 /person_simulator/command 토픽의 std_msgs/msg/String에 JSON으로 전달합니다. 상태는 /person_simulator/status에서 확인합니다.

- spawn: normal 모드는 1~3명, test 모드는 1~8명 순차 생성, legacy는 기존 방식입니다.
- test 모드는 config/policy.yaml의 단일 스폰 위치에서 한 명씩 생성합니다. 이전 사람이 1.00m 이상 벗어날 때까지 최대 60초 기다립니다.
- start, stop, resume, delete, cleanup, status, shutdown, recover를 지원합니다.
- sudden_stop, long_block, guest_cycle은 수동 시험 명령입니다. guest_cycle은 normal 모드의 2번 사람에게 적용합니다.
- 기존 속도·정지 확률·시간은 config/policy.yaml에 보존됩니다.

## 파일 역할과 제한

- manager.py: ROS 명령·상태·사람별 제어 및 Gazebo 서비스 호출
- policy.py: 확률·거리·교착·혼합 행동 판단
- gazebo_api.py: 사람 모델 생성·삭제
- pose_cli.py: Gazebo 실제 모델 위치 수집
- watchdog.py: manager heartbeat 중단 시 사람 속도 정지
- gazebo_person_event_bridge.py: 별도 LAB 시험용 브리지이며 통합 Launch에서 실행하지 않습니다.

모델 시각 메시는 assets/standing.dae이고 원통 충돌 형상은 유지됩니다. 설정된 차선이나 스폰 위치가 가구와 겹치면 이동이 정지하거나 생성이 실패할 수 있습니다. 사람 모델 생성과 로봇의 인식·안전 반응은 각각 Gazebo 통합 시험으로 확인해야 합니다.
