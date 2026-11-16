# 장애물 불확실 판정 전수 확인

대상: 개인 dynamic 로그 22개. 파일을 순차로 읽은 시점까지의 스냅샷.
불확실 이력이 있는 실행별 트랙: 76개. 물체 개수가 아니라 실행별 ID 개수이며 동일 물체가 중복 포함될 수 있다.
uncertain=true 관측 103818건. 시간이나 물체 개수와 다르다.

원인은 원본 로그의 accepted/last_seen/time 필드와 코드 분기로 분류한 추론이다. 당시 로그에는 원인 문자열이 없다.
- 최근 관측이 있으나 정합 미승인: 21927건
- 마지막 관측 1초 초과, 해소를 확인하지 못해 유지: 81891건
- 나머지/분기 확정 불가: 0건

`motion_state=UNKNOWN` 자체는 불확실 판정과 다르다. 초기 속도 관측 시간이 부족해도 UNKNOWN이 나온다.
`unclassified_object`는 물체 종류 미분류이고 이동 판정 불확실과 다르다.
`TRACK_UNCERTAIN`은 uncertain=true인 트랙이 현재 경로까지 막을 때 나오는 상태다.
SENSOR_STALE은 센서 유효성 상태로 별개이며 동시에 발생할 수 있다.

## 실행별 전체 집계

| 로그 | 불확실 트랙 | 불확실 관측 | TRACK_UNCERTAIN 보고 |
|---|---:|---:|---:|
| robot_01_20261029_212049.jsonl | 2 | 53 | 0 |
| robot_01_20261029_212521.jsonl | 1 | 1210 | 0 |
| robot_01_20261029_213011.jsonl | 1 | 15 | 0 |
| robot_01_20261029_214434.jsonl | 14 | 13252 | 0 |
| robot_01_20261029_215736.jsonl | 3 | 4458 | 0 |
| robot_01_20261029_220620.jsonl | 1 | 29248 | 0 |
| robot_01_20261030_160627.jsonl | 5 | 3030 | 0 |
| robot_01_20261030_160828.jsonl | 7 | 3027 | 0 |
| robot_01_20261030_162209.jsonl | 5 | 6490 | 0 |
| robot_01_20261030_162639.jsonl | 8 | 1280 | 0 |
| robot_01_20261030_163103.jsonl | 4 | 90 | 0 |
| robot_02_20261029_212049.jsonl | 1 | 298 | 0 |
| robot_02_20261029_212521.jsonl | 1 | 1211 | 586 |
| robot_02_20261029_213011.jsonl | 0 | 0 | 0 |
| robot_02_20261029_214434.jsonl | 0 | 0 | 0 |
| robot_02_20261029_215736.jsonl | 5 | 186 | 0 |
| robot_02_20261029_220620.jsonl | 1 | 29248 | 0 |
| robot_02_20261030_160627.jsonl | 4 | 2432 | 0 |
| robot_02_20261030_160828.jsonl | 8 | 3014 | 0 |
| robot_02_20261030_162209.jsonl | 4 | 5204 | 0 |
| robot_02_20261030_162639.jsonl | 1 | 72 | 0 |
| robot_02_20261030_163104.jsonl | 0 | 0 | 0 |

## 10월 30일 개별 트랙

위치는 map 좌표 평균이다. 모델 후보는 현재 데모 변환(+3.2,+2.25)으로 저작된 테이블/의자 중심과 비교한 추정이며, 센서 인식 결과가 아니다.
평균 위치는 이동 트랙의 정체 판정에 사용할 수 없다. 정확한 범위·최초 원문은 CSV/JSON에 보존했다.

| 실행 로그 | 로봇/ID | 분류 | 평균 map (m) | 최근관측·정합실패 | 관측유실 | 경로 차단 | 모델 후보 |
|---|---|---|---|---:|---:|---:|---|
| robot_01_20261030_160627.jsonl | robot_01/object_1 | unclassified_object | (3.912, 0.707) | 606 | 0 | 0 | 미확정 |
| robot_01_20261030_160627.jsonl | robot_01/object_2 | unclassified_object | (2.080, 0.684) | 606 | 0 | 0 | table_set_03 (candidate only) |
| robot_01_20261030_160627.jsonl | robot_01/object_3 | unclassified_object | (0.808, 0.691) | 606 | 0 | 0 | table_set_03 (candidate only) |
| robot_01_20261030_160627.jsonl | robot_01/object_4 | unclassified_object | (1.992, 3.006) | 606 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_160627.jsonl | robot_01/object_5 | unclassified_object | (0.899, 2.700) | 606 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_160828.jsonl | robot_01/object_1 | unclassified_object | (3.165, 0.615) | 38 | 0 | 0 | 미확정 |
| robot_01_20261030_160828.jsonl | robot_01/object_2 | unclassified_object | (3.913, 0.709) | 515 | 0 | 0 | 미확정 |
| robot_01_20261030_160828.jsonl | robot_01/object_3 | unclassified_object | (2.081, 0.684) | 515 | 0 | 0 | table_set_03 (candidate only) |
| robot_01_20261030_160828.jsonl | robot_01/object_4 | unclassified_object | (0.808, 0.691) | 515 | 0 | 0 | table_set_03 (candidate only) |
| robot_01_20261030_160828.jsonl | robot_01/object_5 | unclassified_object | (1.977, 3.005) | 515 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_160828.jsonl | robot_01/object_6 | unclassified_object | (0.899, 2.700) | 515 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_160828.jsonl | robot_01/object_7 | person | (3.185, 1.595) | 22 | 392 | 0 | 미확정 |
| robot_01_20261030_162209.jsonl | robot_01/object_1 | unclassified_object | (3.912, 0.707) | 1298 | 0 | 0 | 미확정 |
| robot_01_20261030_162209.jsonl | robot_01/object_2 | unclassified_object | (2.080, 0.684) | 1298 | 0 | 0 | table_set_03 (candidate only) |
| robot_01_20261030_162209.jsonl | robot_01/object_3 | unclassified_object | (0.808, 0.691) | 1298 | 0 | 966 | table_set_03 (candidate only) |
| robot_01_20261030_162209.jsonl | robot_01/object_4 | unclassified_object | (2.006, 3.007) | 1298 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_162209.jsonl | robot_01/object_5 | unclassified_object | (0.899, 2.700) | 1298 | 0 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_162639.jsonl | robot_01/object_4 | unclassified_object | (1.935, 3.096) | 0 | 544 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_162639.jsonl | robot_01/object_5 | unclassified_object | (0.845, 2.966) | 0 | 572 | 0 | table_set_01 (candidate only) |
| robot_01_20261030_162639.jsonl | robot_01/object_6 | unclassified_object | (5.334, 0.024) | 0 | 17 | 0 | 미확정 |
| robot_01_20261030_162639.jsonl | robot_01/object_7 | unclassified_object | (0.738, 2.684) | 0 | 70 | 0 | 미확정 |
| robot_01_20261030_162639.jsonl | robot_01/object_8 | unclassified_object | (4.454, 0.004) | 0 | 17 | 0 | 미확정 |
| robot_01_20261030_162639.jsonl | robot_01/object_10 | unclassified_object | (3.839, 2.749) | 0 | 16 | 0 | 미확정 |
| robot_01_20261030_162639.jsonl | robot_01/object_11 | unclassified_object | (4.427, 0.009) | 0 | 25 | 0 | 미확정 |
| robot_01_20261030_162639.jsonl | robot_01/object_13 | unclassified_object | (2.207, 1.805) | 0 | 19 | 0 | 미확정 |
| robot_01_20261030_163103.jsonl | robot_01/object_5 | unclassified_object | (5.396, 2.423) | 0 | 22 | 0 | 미확정 |
| robot_01_20261030_163103.jsonl | robot_01/object_6 | unclassified_object | (5.683, -0.953) | 0 | 54 | 0 | 미확정 |
| robot_01_20261030_163103.jsonl | robot_01/object_9 | unclassified_object | (6.395, -0.278) | 0 | 10 | 0 | 미확정 |
| robot_01_20261030_163103.jsonl | robot_01/object_12 | unclassified_object | (2.248, 4.191) | 0 | 4 | 0 | 미확정 |
| robot_02_20261030_160627.jsonl | robot_02/object_1 | unclassified_object | (5.420, 0.575) | 608 | 0 | 0 | table_set_04 (candidate only) |
| robot_02_20261030_160627.jsonl | robot_02/object_2 | unclassified_object | (4.272, 0.730) | 608 | 0 | 0 | table_set_04 (candidate only) |
| robot_02_20261030_160627.jsonl | robot_02/object_3 | unclassified_object | (2.469, 0.768) | 608 | 0 | 0 | 미확정 |
| robot_02_20261030_160627.jsonl | robot_02/object_4 | unclassified_object | (1.071, 0.664) | 608 | 0 | 0 | table_set_03 (candidate only) |
| robot_02_20261030_160828.jsonl | robot_02/object_1 | unclassified_object | (5.420, 0.575) | 512 | 0 | 0 | table_set_04 (candidate only) |
| robot_02_20261030_160828.jsonl | robot_02/object_2 | unclassified_object | (4.272, 0.730) | 512 | 0 | 0 | table_set_04 (candidate only) |
| robot_02_20261030_160828.jsonl | robot_02/object_3 | unclassified_object | (2.473, 0.971) | 26 | 486 | 0 | 미확정 |
| robot_02_20261030_160828.jsonl | robot_02/object_4 | unclassified_object | (3.217, 0.658) | 44 | 0 | 0 | 미확정 |
| robot_02_20261030_160828.jsonl | robot_02/object_5 | unclassified_object | (1.070, 0.664) | 512 | 0 | 0 | table_set_03 (candidate only) |
| robot_02_20261030_160828.jsonl | robot_02/object_6 | unclassified_object | (3.108, 0.519) | 10 | 0 | 0 | 미확정 |
| robot_02_20261030_160828.jsonl | robot_02/object_7 | unclassified_object | (2.467, 0.763) | 498 | 0 | 0 | 미확정 |
| robot_02_20261030_160828.jsonl | robot_02/object_8 | person | (3.209, 1.576) | 22 | 392 | 0 | 미확정 |
| robot_02_20261030_162209.jsonl | robot_02/object_1 | unclassified_object | (5.420, 0.575) | 1301 | 0 | 955 | table_set_04 (candidate only) |
| robot_02_20261030_162209.jsonl | robot_02/object_2 | unclassified_object | (4.272, 0.730) | 1301 | 0 | 0 | table_set_04 (candidate only) |
| robot_02_20261030_162209.jsonl | robot_02/object_3 | unclassified_object | (2.469, 0.768) | 1301 | 0 | 0 | 미확정 |
| robot_02_20261030_162209.jsonl | robot_02/object_4 | unclassified_object | (1.072, 0.664) | 1301 | 0 | 0 | table_set_03 (candidate only) |
| robot_02_20261030_162639.jsonl | robot_02/object_5 | unclassified_object | (0.837, -0.291) | 0 | 72 | 0 | 미확정 |

전체 실행의 개별 트랙은 all_uncertain_tracks.csv, 최초 원문 예시는 evidence.json을 참조한다.
