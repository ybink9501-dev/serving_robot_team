#!/usr/bin/env bash

export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99

source /opt/ros/jazzy/setup.bash
source "$HOME/team_serving_robot/ros_ws/install/setup.bash"
source "$HOME/team_robot_lab/ros_ws/install/local_setup.bash"

REPORT="$HOME/team_robot_lab/test_logs"
mkdir -p "$REPORT"
LOG="$REPORT/preflight_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee "$LOG") 2>&1

echo "========== ROBOT 01 PREFLIGHT =========="
date

echo
echo "===== 1. 실행 Launch ====="
pgrep -af 'ros2 launch' || true

echo
echo "===== 2. Gazebo 모델 ====="
timeout 10 gz model --list || true

echo
echo "===== 3. 중복 노드 ====="
DUPLICATES="$(ros2 node list 2>/dev/null | sort | uniq -d)"
printf '%s\n' "$DUPLICATES"

echo
echo "===== 4. 지도와 센서 ====="
for TOPIC in \
  /clock \
  /map \
  /robot_01/odom \
  /robot_01/scan \
  /robot_01/amcl_pose
do
    echo "--- $TOPIC ---"
    timeout 5 ros2 topic info "$TOPIC" || true
done

echo
echo "===== 5. 시뮬레이션 시간 ====="
timeout 5 ros2 topic hz /clock || true

echo
echo "===== 6. TF 확인 ====="
TF_LOG="$(mktemp)"
timeout 8 ros2 run tf2_ros tf2_echo map base_footprint \
  > "$TF_LOG" 2>&1 || true

cat "$TF_LOG"

if grep -q 'Translation:' "$TF_LOG"; then
    TF_OK=1
else
    TF_OK=0
fi
rm -f "$TF_LOG"

echo
echo "===== 7. Nav2 활성 상태 ====="
for NODE in \
  /map_server \
  /robot_01/amcl \
  /robot_01/planner_server \
  /robot_01/controller_server \
  /robot_01/behavior_server \
  /robot_01/bt_navigator \
  /robot_01/velocity_smoother \
  /robot_01/collision_monitor
do
    echo "--- $NODE ---"
    timeout 5 ros2 lifecycle get "$NODE" || true
done

echo
echo "===== 8. Nav2 재활성화 조건 확인 ====="

if [ -n "$DUPLICATES" ]; then
    echo "BLOCKED: 중복 ROS 노드 존재"
elif [ "$TF_OK" -ne 1 ]; then
    echo "BLOCKED: map -> base_footprint TF 없음"
else
    echo "TF 확인됨. Nav2 상태 점검."

    PLANNER_STATE="$(
      timeout 5 ros2 lifecycle get \
        /robot_01/planner_server 2>/dev/null || true
    )"

    if [ "$PLANNER_STATE" = "active [3]" ]; then
        echo "Planner already active. 재시작 생략."
    elif [ "$PLANNER_STATE" = "inactive [2]" ]; then
        echo "Nav2 Lifecycle Manager STARTUP 요청"

        timeout 30 ros2 service call \
          /robot_01/lifecycle_manager_navigation_lab/manage_nodes \
          nav2_msgs/srv/ManageLifecycleNodes \
          "{command: 0}" || true
    else
        echo "BLOCKED: Planner 상태가 예상과 다름"
    fi
fi

echo
echo "===== 9. 최종 Lifecycle ====="
for NODE in \
  /robot_01/planner_server \
  /robot_01/controller_server \
  /robot_01/behavior_server \
  /robot_01/bt_navigator \
  /robot_01/velocity_smoother \
  /robot_01/collision_monitor
do
    echo "--- $NODE ---"
    timeout 5 ros2 lifecycle get "$NODE" || true
done

echo
echo "===== 10. Safety 속도 경로 ====="
for TOPIC in \
  /robot_01/cmd_vel_nav \
  /robot_01/cmd_vel_smoothed \
  /robot_01/cmd_vel_semantic \
  /robot_01/cmd_vel
do
    echo "--- $TOPIC ---"
    timeout 5 ros2 topic info "$TOPIC" || true
done

echo
echo "========== PREFLIGHT END =========="
echo "LOG: $LOG"
echo "주의: 이 검사는 주행 안전을 최종 승인하지 않습니다."
