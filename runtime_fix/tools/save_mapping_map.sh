#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export GZ_PARTITION="${GZ_PARTITION:-team_robot_lab_gazebo_99}"
# ROS / colcon setup scripts must run with nounset disabled.
# Re-enable strict mode immediately after the environment has been loaded.
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros_ws/install/setup.bash"
set -u
OUT="$ROOT/results/maps/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$OUT")"
ros2 topic info /robot_01/map
ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p map_topic:=/robot_01/map -p save_map_timeout:=10.0
if [[ ! -s "$OUT.pgm" || ! -s "$OUT.yaml" ]]; then echo 'FAIL: expected SLAM map not saved' >&2;exit 3;fi
sha256sum "$OUT.pgm" "$OUT.yaml" > "$OUT.sha256"
echo "SLAM map saved to $OUT.yaml (does not overwrite TEAM/LAB official maps)"
