> 최신 통로 정책: [PERSONAL_CORRIDOR_PRIORITY_20261031.md](PERSONAL_CORRIDOR_PRIORITY_20261031.md). 선도착 우선, 동시 도착 시 주문 수신 순서, 지도 기반 자동 통로 추출을 적용했다. 아래 이전 시험 기록과 구분한다.

# 정식 사람 manager ↔ 개인 자율주행 연결 원인 분석·수정

기준일: 2026-10-31. 수정은 개인 cafe_autonomy·사람 설정·시험 도구에 한정한다.
앞선 시험용 사람 fixture의 안전 검증과 정식 사람 manager 전체 연결 검증은 구분한다.
이번 시험은 정식 /person_simulator/command로 생성·이동·정지·삭제하며 인식 결과를 주입하지 않았다.

## 실패 원인
| 원인 | 실제 증거 | 영향·수정 |
|---|---|---|
| 벽 오검출 억제용 0.65 단일 신뢰도 기준이 가까운 옆모습도 탈락시킴 | 실제 manager 사람 영상에서 YOLO 신뢰도 0.472575, R1 perception count=0, LiDAR unclassified_object는 존재 | 0.65 이상 검출 유지. 0.40~0.65는 별도 후보로 전달하고 실제 LiDAR 전경 곡면·위치 일치·정합 성공이 있을 때만 확인 |
| 첫 보완의 최소 몸통 반경 0.12m 가정이 다리 관측을 거절 | 후보 (0.9266,-0.0619), LiDAR (1.0927,-0.0228), 관측 반경 0.09158m, 표면 17점, 지도 정합 성공 | 0.08~0.40m의 충분한 곡면 관측을 확인 근거로 사용. 후보 위치 오차 <=0.5m, 5점 이상, 곡면 fit 성공 조건 유지. 최종 사람 안전 몸체 반경 0.25m 유지 |
| 다른 지도용 보행·생성 설정이 새 벽·초기 로봇 위치와 불일치 | 기존 첫 경로가 x=0의 partition_south를 가로지름. 생성 지점-로봇 간격 1.0m < 기존 release 1.05m. test_spawn도 벽 위 | 왼쪽 방 안에서만 걷도록 첫 경로를 (-2.8,-2)→(-1.5,-2)로 수정. 두 번째 시작 (4,-0.8), 시험 생성 (-3,-2.5)로 수정. 모든 경로/생성점의 장애물·초기 로봇 간격 회귀 추가 |

통신·사람 생성 실패가 첫 원인은 아니었다. 양쪽 로봇 포즈와 manager 위치 상태는 정상이며,
영상 후보/다리 관측을 사람으로 확인하는 조건 때문에 Robot 01이 사람 대신 일반 장애물로 보았다.
낮은 신뢰도 영상만으로 사람 확정을 허용하거나 안전 거리·정지를 해제하지 않았다.

## 수정 파일
- ros_ws/src/cafe_autonomy/cafe_autonomy/rgbd_people.py: 높은 신뢰도 검출과 낮은 신뢰도 후보를 분리하여 발행.
- ros_ws/src/cafe_autonomy/cafe_autonomy/dynamic_obstacles.py: 신선한 LiDAR 전경·지도 정합·곡면·위치 일치로 후보를 확인.
- ros_ws/src/cafe_autonomy/cafe_autonomy/rgbd_projection.py: 확인 조건을 순수 함수로 분리, 다리 관측 재현 검사.
- config/integration/sites/validation_people.yaml: 해당 개인 시험 장소에 맞는 보행 경로/생성 위치.
- scripts/trial_manager_chain.py: 정식 manager의 실제 생성부터 로봇 임무 완료까지 연결 시험.
- tests/integration/test_person_candidate_corroboration.py 및 test_validation_people_routes.py: 실패 원인 재현·보호 검사.

## 실제 연결
사람 manager → 사람 전용 cmd_vel/bridge → Gazebo의 시각 메시·충돌 형상
→ 로봇 RGB-D/LiDAR → 사람 검출/후보 + 동적 추적 → Semantic Safety → Collision Monitor → 로봇.
사람 manager는 양쪽 로봇 위치를 읽어 사람 양보 정책을 적용한다.
로봇은 자신의 센서 관측으로 사람에 반응하며, Gazebo truth는 운행 정책에 주입하지 않는다.

## 정식 manager 종단 시험 결과
- 성공: True, 충돌: False.
- 두 로봇 모두 manager가 생성한 사람을 카메라 후보/검출 및 동적 추적에서 확인.
- Robot 01은 진행 중인 같은 request_id에서 사람 근접 거리와 안전 출력 반영 확인.
- 실제 사람 이동: 01 0.12660m / 02 0.37752m.
- stop 명령 후 2초 실제 위치 변화: 두 사람 모두 0.0m, 발행 속도도 0 확인.
- cleanup 후 실제 사람 포즈 스트림 유실·manager 목록 비움 확인.
- Robot 01이 원래 목적지로 재개하여 WAIT_HANDOVER 및 해당 ID COMPLETE → SUCCEEDED 확인.
- request_id: manager-chain-09eca6d5a816.
- 증거: _local_only/manager_chain_20261031_215404.json.
- ROS 연결 증거: _local_only/manager_graph_20261031_215459.json. 두 로봇 /cmd_vel 발행자는 각 Collision Monitor 1개.

## 회귀 검사 및 범위
최종 개인 전체 검사 168개 통과(42.34초), 개인 cafe_autonomy·serving_robot_person_simulator 빌드 성공.
낮은 신뢰도의 벽 후보는 그 자체로 사람 확정되지 않으며, 선형 벽·미일치 위치·부족한 관측은 단위 검사에서 제외됨을 확인했다.
실제 빈 복도 조우 회귀 결과는 아래에 기록했다.
Git commit/push는 하지 않았고 팀 원본·Robot 02 제어기 설정은 수정하지 않았다.
이 결과는 Gazebo에서의 연결 검증이며 실제 하드웨어·팀 Backend 연동 검증과 구분한다.


## 회귀 중 추가 확인한 양보 조기 재개 원인
실패 증거: _local_only/corridor_trial_20261031_220033.json 및 해당 Fleet DB 영구 이벤트.
우선 Robot 02 초기 x=5.65959475는 통행 구역 확장 경계 x=5.65보다 0.00959m 바깥이었다.
양보 Robot 01이 YIELD_WAIT에 도착하자 기존 코드는 이를 ‘우선 로봇이 구역을 벗어남’으로 판단했다.
하지만 우선 로봇은 아직 시작 위치에 있었고, 통과를 시작하기 전이었다.
그 상태에서 Robot 01이 원래 목표로 재개하여 복도가 막혀 있는 동안 계획 실패를 반복하고 GOAL_RETRY_EXHAUSTED가 됐다.
이는 사람 분류 실패와 별개의 Fleet 통행 상태 판정 오류다.

수정: 양보 시작 때 우선 로봇과 원래 차단 로봇의 위치를 저장한다.
구역 비점유·목적지 간격·센서 신선도 조건에 더해, 우선 로봇이 원래 차단 위치를 접근 방향으로 지나
기존 로봇 간격 0.9m만큼 벗어났는지 확인한다. 다른 방향의 경로는 실제 목표 도착·행동 종료 확인을 대안으로 사용한다.
수정 파일: cafe_autonomy/fleet.py, yield_policy.py. 실제 실패 좌표의 전·후·측면 이동을 test_yield_passage_gate.py에서 검증했다.

수정 후 실제 복도 시험: 성공, 두 로봇 모두 실제 복도 통과·각 주문 완료, 충돌 없음.
최소 실제 중심 간격 1.33494m.
증거: _local_only/corridor_trial_20261031_220443.json.

## 최종 판정
정식 사람 manager의 생성·이동·정지·삭제 → 두 로봇 인식/추적 → Robot 01 안전 제어·동일 임무 완료 연결을 확인했다.
실제 빈 복도 양보/교대 통과도 최종 수정으로 재검증했다.
개인 전체 회귀 168개, 두 개인 패키지 빌드 통과. 시험 세션은 종료한다.
팀 원본 및 Robot 02 제어기 설정 보존, Git commit/push 미실행.
