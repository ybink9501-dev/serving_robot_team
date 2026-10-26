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
python3 "$ROOT/tools/acceptance.py"
