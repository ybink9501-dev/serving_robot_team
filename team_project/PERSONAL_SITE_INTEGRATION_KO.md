> 최신 통로 정책: [PERSONAL_CORRIDOR_PRIORITY_20261031.md](PERSONAL_CORRIDOR_PRIORITY_20261031.md). 선도착 우선, 동시 도착 시 주문 수신 순서, 지도 기반 자동 통로 추출을 적용했다. 아래 이전 시험 기록과 구분한다.

> 정식 사람 manager 전체 연결의 추가 원인 분석·수정·검증(2026-10-31): [PERSONAL_MANAGER_INTEGRATION_20261031.md](PERSONAL_MANAGER_INTEGRATION_20261031.md).

# 개인 자율주행·사람 시뮬레이션 통합

## 범위
개인 `cafe_autonomy`, 개인 실행 스크립트, `config/integration`, 사람 시뮬레이터만 수정한다.
팀 `serving_robot_mapping/navigation/bringup/description/gazebo`, Backend, common 설정과 원본 지도를 수정하지 않는다.

## 기존 팀 매핑 재사용
팀 `serving_robot_bringup/launch/mapping_mode.launch.py`는 Gazebo → 로봇/브리지 → slam_toolbox → Nav2 → frontier 후보를 준비한다.
실제 자동 탐색은 팀 `serving_robot_mapping/tools/run_lidar_mapping_session.py`가 별도로 시작한다.
개인 `scripts/start_autonomy.sh --mode mapping`은 팀 slam.yaml을 읽어 robot_01 프레임/스캔으로 SLAM을 실행한다.
개인 mapping 모드에 팀 자동 탐색 도구까지 연결한 것은 아니다. 수동 스캔이나 기존 팀 탐색으로 생성한 Occupancy Map을 재사용한다.
운영은 저장한 지도 + AMCL 위치 추정을 사용한다. 지도만으로 부엌·테이블 의미가 자동 인식되지는 않는다.

## 장소별 설정
`scripts/start_autonomy.sh --headless --site config/integration/sites/demo_lidar_complete.yaml --people`

`--site` YAML은 system.yaml을 덮어쓰는 개인 장소 설정이다. 경로는 team_project 기준이다.
필수: site_id, map_yaml, waypoints_file, zones_file, world_file, 두 robots의 전체 좌표,
world_to_map 전체 변환, people 설정이다. 지도·목적지·로봇 초기 위치는 모두 같은 map 프레임이어야 한다.
로봇 스폰과 사람 이동 경로는 Gazebo world 좌표다. 별도 world_to_map 변환과 혼동하지 않는다.
지도 이미지의 resolution/origin을 보존한다. 저장 지도 픽셀을 테이블 위치에 맞춰 임의 변경하지 않는다.
새 장소에서는 HOME, kitchen, 테이블 접근점(x/y/yaw), 통행 구역과 사람 보행 경로를 확인하여 등록한다.
테이블 이름은 table_01~04에 고정되지 않는다. HOME과 kitchen을 제외한 목적지가 주문 목적지다.
테이블 접근 yaw는 테이블 변에 수직이며 테이블을 향하는 방향으로 설정한다.
통행 제한 구역이 없는 장소는 zones.yaml에 `zones: {}`를 명시할 수 있다.
장소별 주문 DB·Mission 복구 파일을 분리해 이전 지도의 미완료 임무가 재개되지 않도록 한다.

새 장소는 기본적으로 Occupancy Map만 사용하여 정적 장애물을 분리한다(`static_world_geometry: false`).
기존 데모의 SDF 가구 형상을 새 지도에 자동 적용하지 않는다. system.yaml 기본 실행도 검증한 실제 LiDAR 지도와 독립 DB를 사용한다.
현재 시뮬레이터 어댑터의 world 이름은 cafe가 필요하다. world 파일은 변경 가능하지만 cafe 이름을 유지해야 한다.
다른 월드 이름, 실물 로봇 브리지, 자동 테이블 인식, 미지 공간 자동 탐색의 개인 실행 통합은 후속 작업이다.

## 사람 시뮬레이션
`--people`은 기존 people.launch.py의 브리지·manager·watchdog를 같은 실행에 포함한다.
이 옵션 자체로 사람을 생성하지 않는다. 사람 생성/시작은 기존 `/person_simulator` 제어 인터페이스에서 명시적으로 요청한다.
장소별 people.config는 사람 패키지 policy.yaml 형식을 따르며 world=cafe와 검토한 lanes를 명시한다.
다른 장소에 데모 보행 경로를 그대로 사용하지 않는다. 시뮬레이터는 사람 이동을 위해 실제 위치를 사용할 수 있으나
로봇의 사람 인식·안전 제어에는 Gazebo 사람 정답 좌표를 넣지 않는다.
로봇은 실제 RGB-D/LiDAR 관측을 사용한다. 이미지·인식 확인과 안전 대응 합격은 별개다.

## 임무 규칙
request_id 중복 실행 금지, BUSY 로봇에는 새 임무 금지, Fleet 대기열과 한 로봇당 활성 임무 하나를 유지한다.
LOAD는 WAIT_LOAD, COMPLETE는 WAIT_HANDOVER, COLLECTED는 WAIT_COLLECTION에서 해당 주문에만 적용한다.
회수 확인 후의 운반 목적지는 기존 설계 증거가 추가 확인되기 전 새로 추정하여 추가하지 않는다.
위치·방향의 최종 정렬은 아래 오차 여유 및 yaw 기준을 따른다.
완료 자체는 HOME을 실행하지 않는다. Fleet가 대기열이 비었고 건강한 완료 로봇이 HOME 밖에 있을 때
별도 idle-home 요청을 저장한다. 대기 중인 실제 주문은 idle-home 요청보다 먼저 배차한다.
HOME 이동이 이미 시작된 뒤 새 주문이 들어오면 현재 HOME 이동을 마친 후 배차한다(이동 중 Goal 교체 없음).

## 현재 실행 구성과 검증
Robot 01은 Rotation Shim + MPPI다. Robot 02 제어기와 팀 원본 패키지는 수정하지 않았다.
개인 기본 operation 지도는 실제 SLAM Toolbox/LiDAR로 저장한 maps/stores/demo_store/team_lidar_complete.yaml이다.
SDF 가구 형상에 의존하는 장애물 정합은 기본 해제했다. 지도 생성 때에도 SDF를 지도 픽셀로 변환하지 않았다.
기본 DB와 임무 상태는 _local_only/sites/demo_lidar_operation 아래로 분리했다.

테이블 도착 허용 오차는 10cm다. 위치추정 오차 여유 3cm를 제외한 내부 정렬 목표는 7cm이며,
2.5mm 정차를 요구하지 않는다. 최종 yaw 제어 목표는 5도, 기존 확인 허용 각도는 0.25rad다.
테이블 목적지 yaw는 해당 변을 바라보는 수직 방향으로 장소별 등록한다.

기존 카페의 실제 지도에서 두 로봇 KITCHEN_PICKUP → LOAD → WAIT_HANDOVER → COMPLETE → 자동 HOME을 검증했다.
190.3초, 물리 정차 오차 Robot 01 7.06cm / Robot 02 5.74cm,
테이블 수직 방향 오차 각각 약 3.02도 / 3.01도였다.
기록: _local_only/serving_trial_20261030_220112.summary.json,
_local_only/metrics/measure_20261030_220050.csv.

별도 validation_cafe 구조에서도 실제 LiDAR SLAM 지도 생성·저장까지 완료했다.
이름이 cafe인 다른 SDF 월드를 지원하되 현재 Gazebo 어댑터의 world 이름 cafe 제약은 남는다.
새 장소의 테이블·주방·HOME·통행 구역·사람 보행 구역은 등록해야 한다. 자동 테이블 인식이나 자동 탐사 완료를 뜻하지 않는다.
지도 생성 시험은 검토한 경유점을 운영자 대신 따라간 것이며, 새로운 production 탐사기를 추가한 것이 아니다.

불확실 트랙 수정과 시나리오별 최신 결과는 PERSONAL_VALIDATION_20261030.md를 참조한다.
과거 로그의 모든 불확실 상태가 오류인 것은 아니다. 가림·관측 유실 때 증거가 없으면 안전 정지를 유지한다.
팀 Backend와 실제 적재/수령 장치는 수정하지 않았고, 종단 시험의 LOAD/COMPLETE/COLLECTED는 운영자 신호를 시험 도구가 대신 전송한다.
