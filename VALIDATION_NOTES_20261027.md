# 2026-10-27 통합 시험 인계

기준 브랜치: `integration/team-develop-906d3e0`. 팀 원본 441개 파일의 내용과 파일 모드는 제공 ZIP과 동일하게 보존한다. 이번 변경은 개인 자율주행 카메라 연결 및 개인 테스트의 원본 해시 갱신이다. 커밋·업로드 시 `_local_only` 로그는 제외한다.

## 확인한 결과

- ROS 2 Jazzy 빌드: 10개 패키지 완료.
- 오프라인 회귀: 315개 PASS. 팀 최신 World·Map의 실제 해시로 개인 보호 테스트를 갱신했다.
- 개인 Robot 01 단독: `table_03` 서빙 상태 COMPLETED, Gazebo 실제 XY 오차 약 5.5cm.
- 개인 Robot 02 단독: `table_04` COMPLETED, 약 3.3cm. 명시적 GO_HOME 완료, 약 2.0cm.
- 두 로봇 동시 서빙: Robot 01 `table_03` 약 6.0cm, Robot 02 `table_04` 약 3.2cm. 두 목표 모두 COMPLETED. 기록된 최종 안전 속도의 최소 선속도는 0m/s였다.
- 동시 HOME: Robot 02 COMPLETED, 약 3.5cm. Robot 01은 150초 제한 안에 완료하지 못했으며 마지막 실제 HOME 거리 약 28.5cm. Nav2 로그에 `Unable to transform goal pose into costmap frame`, 복구 3회가 있었다. 원인은 아직 확정되지 않았다.
- 독립 사람 시뮬레이터: `test_person_01` 생성, 명령 후 실제 약 0.39m 이동, 삭제 확인. Robot 01과 동시 기동 중 로봇 미션은 유지됐다. 사람 근접 감속·정지 시험은 미실행이다.
- 팀 기본 Launch: Robot 01 센서·TF·AMCL·Nav2·Collision Monitor 활성화 확인. 단, ZIP에서 추출한 팀 navigation Python 실행 파일의 실행 권한이 없어 처음 기동에 실패했다. 시험 중 임시로 권한을 복원한 뒤 기동했으나, `wait_for_ready.py`가 활성 상태인 bt_navigator 응답을 놓치고 계속 기다렸다. 한 번 보낸 `table_03` 목표는 Nav2가 추종을 시작했으나 Launch 제한시간 때문에 최종 도착은 미확인이다. 팀 파일은 변경하지 않았다.

## 집 컴퓨터 후속 순서

1. 팀 기본 Launch의 스크립트 실행 권한 및 readiness 재시도 문제를 팀 담당자와 결정한다. 자동 승인 검토가 팀 소유 `wait_for_ready.py` 수정 시도를 거부했으므로 이번 브랜치에서는 수정하지 않았다.
2. 개인 스택에서 Robot 01 동시 HOME 복귀의 TF/Costmap 오류를 같은 로그 조건으로 재현하고 원인 확인 후 최소 수정한다.
3. 전체 주방 적재→테이블→명시적 HOME, 사람 근접 안전 반응, 동적 장애물 우회, 장거리 주행, 두 로봇 교착 시험을 각각 수행한다. 위 짧은 시험만으로 이를 PASS 처리하지 않는다.
4. 팀 기본 Launch와 개인 자율주행 Launch는 지도·좌표계·ROS Domain·속도 제어 구성이 달라 동시에 실행하지 않는다.

로컬 원시 로그: `team_project/_local_only/operation_20261027_151422.log`, `operation_20261027_151928.log`, `operation_20261027_152205.log`, `serving_trial_20261027_152239.jsonl`, `compat_dual_home_20261027.jsonl`, `metrics/measure_20261027_152209.csv`, `person_compat_20261027.log`. 이 자료는 Git에서 제외된다.
