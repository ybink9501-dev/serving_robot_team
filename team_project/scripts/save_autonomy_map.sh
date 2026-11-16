#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name="${1:-lidar_$(date +%Y%m%d_%H%M%S)}"
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple map name'; exit 2; }
mkdir -p "$ROOT/_local_only/maps"
[[ ! -e "$ROOT/_local_only/maps/$name.yaml" && ! -e "$ROOT/_local_only/maps/$name.pgm" ]] || { echo 'Map exists; refusing overwrite'; exit 2; }
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"
ros2 run nav2_map_server map_saver_cli -f "$ROOT/_local_only/maps/$name" --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
