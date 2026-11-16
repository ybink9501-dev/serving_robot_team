> 정식 사람 manager 전체 연결의 추가 원인 분석·수정·검증(2026-10-31): [PERSONAL_MANAGER_INTEGRATION_20261031.md](PERSONAL_MANAGER_INTEGRATION_20261031.md).

# 개인 자율주행·사람 시뮬레이션 검증 기록

검증 범위는 개인 cafe_autonomy, Robot 01 Nav2, 사람 시뮬레이터와 개인 실행·시험 설정이다.
Robot 02 제어기와 팀 원본 Gazebo·로봇·SLAM·Backend 패키지는 보존한다. Git commit/push는 하지 않았다.

## 확인된 수정
- Occupancy Map 점유면 기반 정합, 관측 배치 최소 비용 연결, 고유 로봇 ID 재연결.
- 좌표 기준 변경·정합 실패·재획득 시 운동 이력 재설정. 불확실 사유 명시.
- 신선한 LiDAR의 몸체 전체 광선과 실제 재관측으로만 유실 트랙 해소.
- 확인된 정지 가구의 관측 표면을 Costmap/충돌 판단에 반영하여 가상 큰 원이 통로를 막는 문제 수정.
- RGB-D 몸통 픽셀별 3차원 정합. 연결된 LiDAR의 더 가까운 사람 몸체 관측을 버리지 않도록 보수적으로 융합.
- Robot 01 Rotation Shim + MPPI 복원, Nav2 취소 확인 후 최종 위치·yaw 정렬로 인계.
- 도착 허용 10cm, 위치추정 여유 3cm, 내부 위치 목표 7cm, yaw 제어 목표 5도.
- WAIT_COLLECTION과 COLLECTED 분리, request_id/대기 상태/출처 확인, 고객 대기열 우선·빈 대기열 HOME.
- 장소별 지도·목적지·통행 구역·사람 구역·DB/임무 상태 분리, 사람 실행은 --people로 선택.
- 좁은 복도 선점권 유지, 근거리 교착 인식, Nav2 양보 경로 후보를 로봇별 순차 검증.

## 완료된 실행 증거
| 시나리오 | 결과 | 증거 |
|---|---|---|
| 기존 카페 실제 SLAM 지도 생성 | 검토한 9개 경유점, 의자 주변 포함 | maps/stores/demo_store/team_lidar_complete.provenance.json |
| 다른 구조의 지도 실제 SLAM 생성 | 두 방·좁은 연결 복도 포함 10개 경유점 | maps/stores/validation_cafe/lidar_map.provenance.json |
| 실제 지도 양쪽 적재·배달·빈 대기열 HOME | 성공, 190.3초 | _local_only/serving_trial_20261030_220112.summary.json |
| 물리 테이블 정차 | R1 7.06cm/3.02도, R2 5.74cm/3.01도 | _local_only/metrics/measure_20261030_220050.csv |
| 사람 manager 생성·이동·정지·삭제 | 실제 이동 0.539m, 정지 명령 0, 삭제 성공 | _local_only/person_integrated_smoke_20261030.json |
| 삭제한 사람의 유실 이력 | R1 배달 성공, R2 실제 5.912m 이동 후 재관측으로 잔류 해소, 충돌 없음 | _local_only/removed_person_*.json, reobserve_*.json |

후속 실제 사람 안전 시험·복도 조우·다른 지도 서빙 결과는 아래 항목에 기록했다.

## 해석 범위
실제 센서 외의 사람 검출 결과나 우회용 cmd_vel을 주입하지 않는다. Gazebo truth는 시험 계측에만 사용한다.
10cm와 방향 오차는 시험에서 측정한 결과이며 임의의 미지 지도에 대한 보증은 아니다.
새 매장의 의미 목적지 등록과 사람 보행 구역 검토는 필요하다. 현재 Gazebo world 이름은 cafe여야 한다.
실제 하드웨어·팀 Backend·적재 장치 연동은 이번 개인 범위 검증에 포함되지 않는다.
과거 76개 트랙은 실행별 ID 집계이며 76개 버그나 76개의 실제 물체가 아니다.


## 실제 사람 안전 종단 시험
증거: _local_only/human_flow_20261030_231007.json, request_id human-flow-ecb8a9ba511e.
실제 YOLO/RGB-D·LiDAR 관측으로 SLOW 3표본, STOP 및 최종 속도 0/물리 정차 8표본을 확인했다.
로봇이 실제 사람 모델 옆을 지나간 후 삭제, 재출발, WAIT_HANDOVER 도착, 해당 ID의 COMPLETE 수락까지 성공했다.
충돌 없음, 실제 로봇·사람 중심 최소 거리 0.90295m. 별도 DETOUR 이벤트 없이 Nav2의 경로 회피로 통과했다.
시험용 사람 이동은 Gazebo 직접 서비스로 실제 경과 시간에 맞춰 연속 이동한다.
이전 CLI 반복 호출은 명령 소요 시간이 누적되어 로봇이 사람 도착 전에 먼저 지나가는 시험 문제가 있었다.
이전 실패 기록도 _local_only/human_flow_*.json에 보존한다.


## 별도 구조 지도 서빙 종단 시험
validation_cafe 실제 SLAM 지도에서 KITCHEN_PICKUP 두 건과 빈 대기열 HOME이 187.3초에 완료됐다.
증거: _local_only/serving_trial_20261030_231031.summary.json.
Robot 01 trial-764c14a23d52 → table_A, Robot 02 trial-9e2480cfee0f → table_B.
두 로봇 모두 LOAD/COMPLETE를 해당 request_id에만 적용했다.
물리 정차 오차: R1 7.72cm/수직 방향 2.99도, R2 3.26cm/3.68도.
계측: _local_only/metrics/measure_20261030_230843.csv의 해당 테이블 WAIT_HANDOVER 표본.
R1 주방 경로 계획 실패 1회, R2 회전 누적 제한 후 전진 경로 회복 1회가 있었고, 안전 제한을 해제하지 않고 재시도하여 완료했다.


## 회수 임무·빌드·회귀 검사
다른 지도에서 두 TABLE_COLLECT 임무 → WAIT_COLLECTION → COLLECTED → 빈 대기열 HOME을 86.3초에 완료했다.
증거: _local_only/serving_trial_20261030_231428.summary.json.
R1 trial-5be4667c086f, R2 trial-601f58f9d2bb. COLLECTED만 수락되었고 LOAD/COMPLETE로 회수를 대체하지 않았다.
개인 전체 회귀 검사 156개 통과(38.51초), cafe_autonomy 및 serving_robot_person_simulator colcon 빌드 성공.
보호 대상 팀 패키지·Robot 02 제어기 설정의 git diff --exit-code 결과 0.


## 복도 조우에서 추가로 재현·수정한 문제
- Nav2 경로가 승인돼도 가구에 근접한 양보 목표에서는 최종 정렬을 Collision Monitor가 차단했다.
  저장 지도의 점유·미확인 셀과 경계를 포함해 반경 0.48m(로봇 0.33 + 기존 여유 0.15)의 알려진 자유 공간을 확인한다.
  Nav2 경로 승인과 실제 안전 제어는 그대로 유지한다. 가구 인접·미확인 후보 제외 회귀 검사를 추가했다.
- Fleet 대기 중 NavigateToPose를 계속 실행해 진행 제한/회복을 소모할 수 있었다.
  Fleet 교통 대기는 취소 응답 확인 후 같은 임무·목적지로 재개한다. 동적 장애물의 짧은 관측 대기는 기존 동작을 유지한다.
- 빈 복도의 영상 모서리·그림자가 YOLO person으로 검출됐다. 저장 RGB 영상의 실제 추론 신뢰도는 0.588/0.519였다.
  검출 신뢰도 기본값을 0.65로 조정했다. 실제 사람 안전 시나리오도 변경 후 재검증하여 통과했다.
  영상 증거: _local_only/corridor_front.png, corridor_front_yolo.png.
  검출 한계가 완전히 사라진다는 뜻은 아니며 LiDAR/충돌 감시와 불확실 상태의 안전 정지는 유지한다.
- 이 배치의 개인 전체 회귀 검사: 161개 통과(37.85초).


## 최종 완료 판정
- 폭 1.1m의 실제 연결 복도에서 정면 상대 임무 두 건: 양보 → 우선 통과 → 대기 로봇 재개 → 양쪽 임무 완료 성공.
  두 로봇 모두 실제 복도에 진입·통과했으며, 최소 실제 중심 간격 1.32957m, 충돌 없음.
  증거: _local_only/corridor_trial_20261030_233210.json.
  이 전용 시험은 auto_home_when_idle=false로 양보/교대 통과를 분리했다. 일반 운용 기본값은 true이며 위 두 서빙/회수 시험에서 HOME을 별도 검증했다.
- 검출 기준 0.65의 최종 실제 사람 시험: 감속 3표본, 근접 정지·물리 정차 3표본,
  실제 우회·삭제 후 재개·목적지 도착·해당 주문 수령 확인 성공. 충돌 없음, 최소 중심 거리 0.99517m.
  증거: _local_only/human_flow_20261030_233431.json, request_id human-flow-b66cc4ee5c09.
- 최종 개인 회귀 161개 통과, 개인 두 패키지 빌드 성공.
- 실제 SLAM 지도 두 구성, 기존/다른 지도 서빙 및 HOME, 별도 회수, 삭제 사람 재관측,
  실제 사람 안전 동작, 두 로봇 복도 조우까지 이번 개인 파트 수정·Gazebo 검증 범위 완료.
- Git commit/push 및 팀 Backend·실물 로봇 검증은 수행하지 않았다.

## 실행
WSL Ubuntu-24.04에서 team_project 폴더를 열고 실행한다.
```bash
bash scripts/start_autonomy.sh --headless --people
# Gazebo 화면을 열려면 --headless를 생략한다.
# 다른 등록 지도:
bash scripts/start_autonomy.sh --headless --site config/integration/sites/validation_cafe.yaml --people
# 위 실행 중 하나를 종료:
bash scripts/stop_autonomy.sh
```
사람은 --people로 함께 실행하더라도 별도 생성 명령 전에는 생성되지 않는다.
기본 운용은 실제 team_lidar_complete 지도와 별도 demo_lidar_operation DB를 사용한다.
추가 장소를 바로 자동 발견하는 기능은 아니며, 장소별 목적지·방향·통행·사람 경로 등록은 필요하다.
