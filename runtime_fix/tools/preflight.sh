#!/usr/bin/env bash
set -u
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'FAIL: ROS Jazzy not found under /opt/ros/jazzy'; exit 2
fi
# ROS Jazzy setup scripts may use unset variables; protect them from nounset.
set +u
source /opt/ros/jazzy/setup.bash
set -u
packages=(ros_gz_sim ros_gz_bridge nav2_map_server nav2_amcl nav2_planner nav2_controller nav2_bt_navigator nav2_behaviors nav2_velocity_smoother nav2_collision_monitor nav2_lifecycle_manager tf2_ros)
errors=0
for pkg in "${packages[@]}";do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1;then echo "FOUND $pkg"
  else echo "MISSING $pkg";errors=$((errors+1));fi
done
if ! command -v gz >/dev/null;then echo 'MISSING Gazebo gz executable';errors=$((errors+1));fi
if ! command -v colcon >/dev/null;then echo 'MISSING colcon executable';errors=$((errors+1));fi
if [[ "$errors" -gt 0 ]];then echo "PRE-FLIGHT FAIL ($errors required dependencies absent)";exit 3;fi
echo 'PRE-FLIGHT DEPENDENCIES FOUND. This does NOT verify Nav2/Gazebo runtime.'
