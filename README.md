# TEAM 카페 서빙로봇 — 통합 준비 브랜치

`team_project/`에는 2026-10-26 팀 `develop` 원본과 개인 자율주행·사람 시뮬레이터를 함께 보관합니다. 팀 파일 441개는 제공된 최신 ZIP과 동일하게 유지했습니다.

- 팀 기본 실행: [`team_project/README.md`](team_project/README.md)
- 개인 자율주행 실행: [`team_project/PERSONAL_AUTONOMY_README.md`](team_project/PERSONAL_AUTONOMY_README.md)
- 사람 시뮬레이터: `team_project/ros_ws/src/serving_robot_person_simulator/`

팀 기본 Launch와 개인 자율주행 Launch는 지도·좌표계·ROS Domain·속도 제어 구성이 다릅니다. 같은 Gazebo 세션에서 동시에 실행하지 마세요. 최신 팀 World/Map과 개인 자율주행의 실제 호환성은 집 컴퓨터에서 빌드·Gazebo 시험 전까지 미검증입니다.

빌드 결과와 로컬 로그·모델은 Git에서 제외합니다.
