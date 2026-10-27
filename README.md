# TEAM 카페 서빙로봇

ROS 2 Jazzy + Gazebo 통합 프로젝트는 [`team_project/`](team_project/)에 있습니다. 기존 팀 폴더 구조 안에 자율주행 코드와 선택형 사람 시뮬레이터를 연결했습니다.

- 자율주행 및 안전: `team_project/ros_ws/src/cafe_autonomy/`, `serving_robot_navigation/`, `serving_robot_perception/`, 관련 `config/integration/` 및 실행 스크립트
- 사람 시뮬레이터: `team_project/ros_ws/src/serving_robot_person_simulator/` (로봇 제어와 독립 실행)
- 팀 프로젝트 구성: `team_project/ros_ws/src/`의 나머지 ROS 패키지와 `maps/`, `web/`, `deploy/`

설치 및 실행 방법은 [`team_project/README.md`](team_project/README.md)를 참고하세요. 빌드 결과, 로컬 모델 가중치, 실행 로그 및 시험 자료는 Git에서 제외됩니다.
