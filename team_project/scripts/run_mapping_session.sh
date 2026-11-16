#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"
export GZ_PARTITION="${GZ_PARTITION:-team_integration_${ROS_DOMAIN_ID}}"
exec python3 "$ROOT/scripts/mapping_session.py" "$@"
