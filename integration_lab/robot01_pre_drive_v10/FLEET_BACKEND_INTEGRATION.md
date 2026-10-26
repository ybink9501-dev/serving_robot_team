# Fleet / 동료 Backend 통합 계약 초안 (아직 실제 연동 전)

## 역할
- **Robot 01 V3 로컬 감독자:** 이미 배정된 서빙 작업 하나를 실행. `/robot_01/serving/request`의 `START/CANCEL`, `/robot_01/serving/complete`의 `CONFIRM_LOAD/COMPLETE` 수신. `/robot_01/serving/status` 상태 발행. 로봇 안전은 Semantic Safety + Collision Monitor.
- **Fleet:** 모든 로봇의 가용성/작업/공용 Zone 점유를 관장. 한 주문을 한 로봇에만 배정, 두 로봇 중 선택, 충돌할 수 있는 통로의 사용 권한 조정, 작업 취소/실패/재배정 결정. V3는 Fleet 자체가 아님.
- **Backend + DB:** HTTP/WebSocket API로 주문 수신, 입력 검증, request_id 중복 처리, 영속 작업/이벤트 기록, Fleet 어댑터 호출, 사용자에게 상태 전달.
- **Frontend (아직 미구현):** Backend의 HTTP/WebSocket만 사용. ROS 토픽 직접 구독하지 않음.

## 통합 단계 (코드 섞기 이전)
1. 동료 Backend의 실제 Git/ZIP 코드와 schema, 실행 방법, 테스트 결과 확보. 현재 미확인; 파일 없이 완료 선언 금지.
2. `docs/INTERFACES.md`와 현재 로컬 V3 토픽/상태 간 차이 표 작성. 동료 코드 변경 없이 **한쪽 어댑터만** 추가.
3. 먼저 Mock Robot adapter로 주문 생성 → 작업 ID 발급 → 처리/취소 이벤트 → DB까지 오프라인 계약 시험.
4. 실제 Robot 01로 `START` 발행은 Fleet의 작업 배정이 **확정된 경우에만**. V3 상태를 TaskEvent로 변환, 중복 START 금지.
5. Robot 02가 정상 복제된 뒤 Zone 동시 점유 금지 및 재배정 검증.
6. Backend 계약이 안정되면 Frontend를 연결.

## 최소 공통 데이터 필드 (제안안; 팀 확정값 아님)
- `task_id`: 서버 할당 작업 고유 ID
- `client_request_id`: 중복 주문 방지용 클라이언트 요청 키
- `robot_id`: `robot_01` 또는 `robot_02`
- `table_id`: `table_01`~`table_04`
- `task_state`: `CREATED`, `ASSIGNED`, `GOING_PICKUP`, `WAIT_LOAD`, `GOING_TABLE`, `WAIT_HANDOVER`, `RETURNING_PICKUP`, `COMPLETED`, `FAILED`, `CANCELED`
- `map_revision`: V3의 운영 지도 버전
- `timestamp`: DB 기록 시각 및 상태 변경 이벤트 시각

`task_id`와 `client_request_id`는 현재 V3 명령에 없음. Fleet ↔ V3 간 매핑/중복 보호는 새 Adapter에서 구현해야 함. Robot 01의 V3 코드를 즉시 바꾸거나 팀 backend의 업무 모델을 임의 수정하지 말 것.
