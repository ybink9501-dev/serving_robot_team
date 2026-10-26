# Robot 01 Precision V4 (LAB only, TEAM files untouched)

## 최종 값
| 항목 | 이전 | V4 | 이유 |
|---|---|---|---|
| Nav2 `xy_goal_tolerance` | LAB 0.25 / TEAM·trial 0.0025 | **0.03 m** | 3 cm에서 인계, 나머지는 미세 접근 |
| Nav2 `yaw_goal_tolerance` | 0.25 | **0.30 rad** | Nav2가 빨리 끝나도록, 최종 yaw는 미세 접근 |
| progress checker | Simple, 0.5 m / 10 s | **Pose, 0.03 m·0.1 rad / 15 s** | 마지막 cm 접근 중 목표가 중단되던 원인 후보 제거 |
| `general_goal_checker.stateful` | 파일 true (런타임 false) | **false** | 파일과 런타임 일치 |
| `GoalAngleCritic.enabled` | 파일 true (런타임 false) | **false** | 파일과 런타임 일치 |
| arrival 최종 XY 판정 `final_error_m` | 0.001 | **0.005 m** | 1 mm는 AMCL/odom 분해능 이하 |
| arrival `detection_radius_m` | 0.005 | **0.010 m** | final 이상이어야 함(코드 규칙) |
| arrival `yaw_tolerance_rad` | 0.25 | **0.05 rad** | 미세 접근이 yaw를 맞춤 |
| `final_verify_timeout_s` | 10 | **120 s** | 미세 접근 3회 x 30 s 수용 |
| 미세 접근 XY 목표 | 없음 | **3 mm** | 5 mm 판정에 2 mm 여유 |
| 미세 접근 속도 | 없음 | **v 0.012~0.05 m/s, w 0.04~0.45 rad/s, 후진 0** | |

## 동작
Nav2 SUCCEEDED -> 정지 확인 -> map->odom 변환으로 목표를 odom 프레임에 고정 -> 회전/전진/yaw 보정을
`/robot_01/cmd_vel_nav`(velocity_smoother 입력)로 출력 -> Semantic Safety·Collision Monitor 그대로 통과.
Goal 재전송 없음. 실패 시 최대 3회 후 ERROR. CANCEL 즉시 정지.

## 실행 (이 순서 그대로)
```
export ROS_DOMAIN_ID=99; export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash; source ~/team_serving_robot/ros_ws/install/setup.bash
unzip -o /mnt/c/Users/user/Downloads/robot01_precision_v4.zip -d $HOME
python3 ~/team_robot_lab/precision_v4/costmap_probe.py            # 1) 읽기 전용
python3 ~/team_robot_lab/precision_v4/activate_v4.py --check-only # 2) 테스트 3종 + 실제 속도체인 확인, 변경 없음
python3 ~/team_robot_lab/precision_v4/activate_v4.py              # 3) 2)가 통과한 뒤에만
```
- `--check-only`는 오프라인 테스트 3종(미세접근, V4 서버, V3 안전 회귀)과 실행 중 토픽 연결
  (controller -> cmd_vel_nav -> velocity_smoother -> semantic_safety -> collision_monitor)을 확인한다. 하나라도 어긋나면 거부.
- `--apply-nav2-file`은 마지막에 따로 선택(백업 후 교체, Nav2 재시작 때 로드). 처음엔 쓰지 않는다.
- `costmap_probe.py --experiment`는 Obstacle Layer를 잠시 끄므로 읽기 전용 결과를 본 뒤에만. 복구는 재확인하며, 실패하면 수동 명령을 출력하고 종료코드 2.

## 검증 범위
- 통과: fine_approach 단독 시뮬레이션(`test_fine_approach.py`), 실제 v4 서버 클래스 + 가짜 ROS + 운동학 플랜트
  (`test_v4_server.py`, AMCL 점프·막힘·취소·역방향 목표 포함), V3 안전 회귀 8/8.
- 미검증: Gazebo 실주행, 실제 Safety 체인 동작, 경로 차단(208) 해소. costmap_probe 결과로 확정해야 함.
- 도착 판정은 map TF(AMCL) 기준. Gazebo 실제 위치와 AMCL 오차는 별개이며 cm 단위로 다를 수 있음.
