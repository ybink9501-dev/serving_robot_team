> 최신 개인 파트 수정·검증 완료 결과: [PERSONAL_VALIDATION_20261030.md](team_project/PERSONAL_VALIDATION_20261030.md). 아래 내용은 10월 29일 초기 점검 기록이다.

# 집 컴퓨터 통합 점검 — 2026-10-29

작업 경로: `F:\ROS2\serving_robot_team` / WSL `/mnt/f/ROS2/serving_robot_team`.
기준: `integration/team-develop-906d3e0`, 원격 커밋 `0381940`.
`main`의 마지막 커밋 `fd23262`보다 최신 팀 소스와 카메라 연결 수정이 포함된 브랜치다.
원격 브랜치와 태그를 모두 가져왔으며 서브모듈은 없다.

## 현재 검증

- ROS Jazzy/Ubuntu 24.04, Nav2 및 Gazebo 브리지 설치 확인.
- 개인 자율주행 및 사람 시뮬레이터 기존 회귀 92개 통과.
- 전체 기능 회귀 530개 통과, 1개 건너뜀. Mapping의 flake8/pep257 두 검사는
  저장소 루트에서 빌드·설치 디렉터리까지 검사해 실패했다. 기능 성공으로 대체하지 않는다.
- ROS 패키지 10개 빌드 완료. HDD/WSL에서 0.01~0.36초 파일 시각 차이 경고가 있었으나
  빌드 실패는 없었다. 실행 검증은 별도다.
- 실제 Gazebo 주행·사람 근접 반응·전체 팀 웹 연동은 이번 컴퓨터에서 검증 진행 중이다.

## 두 자율주행 체계가 남아 있는 위치

| 항목 | 개인 스택 | 팀 기본 스택 |
|---|---|---|
| 진입점 | `scripts/start_autonomy.sh` → `cafe_autonomy/launch/team.launch.py` | `serving_robot_bringup/launch/operation_mode.launch.py` |
| 주행·임무 | `cafe_autonomy/mission.py`, `fleet.py` | `serving_robot_navigation/scripts/command/route_executor.py` |
| 명령/상태 | `/<robot>/serving/request`, `serving/status`, `/fleet/request` | `/<robot>/mission_cmd`, `mission_status`, `health`, `alarm` |
| 설정 | `config/integration/` | `config/common/` |
| 운행 지도 | `team_lidar_initial.yaml` | `demo_store/map.yaml` |
| TF | 공유 ROS Domain, `<robot>/base_footprint` | 로봇별 Domain, `base_footprint` |
| 운영 화면 | 개인 Fleet HTTP `8765` | 팀 FastAPI `8000` + React |

팀 navigation은 단순 미사용 잔재만이 아니다. 팀 backend의 ROS 어댑터가 해당 토픽과
상태 필드를 사용하므로 전체 디렉터리를 삭제하면 팀 기본 실행과 웹 연동이 끊긴다.
개인 Launch는 팀 로봇 모델·월드·일부 설정을 재사용하고 개인 Nav2·Safety·Mission을
실행한다. 두 Launch를 같은 Gazebo 세션에 동시에 실행하지 않는다.

## 확인된 연동 차이

1. 팀 backend `app/ros_adapter/ros.py`는 `mission_cmd`를 발행한다. 개인 Mission은
   이 토픽을 구독하지 않으므로 현재 상태로 팀 UI의 주문이 개인 스택에 전달되지 않는다.
2. 팀 backend는 `mission_status.mission_id`, `health` 및 `map → base_footprint`를
   기대한다. 개인 상태의 `request_id` 및 로봇 접두 TF와 직접 호환되지 않는다.
3. 팀과 개인의 주방·홈·테이블 좌표 출처가 다르다. 이름만 연결하는 방식으로 임무를
   변환하면 지도와 구역 계산이 어긋날 수 있으므로 좌표 출처를 함께 맞춰야 한다.
4. 두 Fleet가 각각 배차·주방 잠금·구역 양보를 수행한다. 최종 연동에서 배차 책임을
   정하고 하나의 로봇 속도 안전 체인을 유지해야 한다.
5. 이전 인계에 남은 Robot 01 동시 HOME TF/Costmap 오류와 사람 근접 감속·정지 시험은
   아직 해결됐다고 판정하지 않는다.

## 이번 컴퓨터 대응 변경

- Windows 자동 CRLF 변환 때문에 원본 월드 SHA 검사 실패를 확인했다. 저장소 로컬
  `core.autocrlf=false` 및 `.gitattributes`의 LF 정책을 적용했다.
- `_local_only/venv`에 ROS 시스템 패키지를 유지하는 별도 인식 환경을 준비한다.
  `scripts/setup_perception.sh`와 `requirements-perception.txt`로 재설치한다.
- 개인 `team_control.py`에서 이 환경을 ROS 실행 프로세스에 전달한다.
- 개인 `rgbd_people.py`의 CPU 고정을 해제했다. `device:=auto` 기본값에서 CUDA가
  있으면 GPU, 없으면 CPU를 선택한다. 검출 데이터나 안전 판정값은 주입하지 않는다.

원격 업로드·팀 navigation 삭제·팀 기본 Launch 변경은 하지 않았다.
미완료 항목을 완료로 간주하지 않고 실제 실행 증거에 따라 이 문서를 갱신한다.
