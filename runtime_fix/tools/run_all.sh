#!/usr/bin/env bash
# One-session orchestrator; does not modify source or create another Gazebo if one already runs.
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
actual="$(ros2 pkg prefix cafe_autonomy 2>/dev/null || true)"
if [[ "$actual" != "$ROOT/ros_ws/install/cafe_autonomy" ]]; then
  echo "ERROR: wrong overlay selected: $actual" >&2;exit 6
fi
# Count actual simulation SERVER processes, not the GUI or the ruby wrapper.
if pgrep -af '^gz sim server($|[[:space:]])' | grep -Eq '^[[:space:]]*[0-9]+[[:space:]]+gz sim server($|[[:space:]])'; then
  echo 'ERROR: Gazebo server is already running. Stop the previous session before starting another.' >&2
  exit 2
fi
world_pid='';stack_pid=''
cleanup(){
  trap - EXIT INT TERM
  if [[ -n "$stack_pid" ]]; then kill -INT "$stack_pid" 2>/dev/null || true;wait "$stack_pid" 2>/dev/null || true;fi
  if [[ -n "$world_pid" ]]; then kill -INT "$world_pid" 2>/dev/null || true;wait "$world_pid" 2>/dev/null || true;fi
  echo 'Launch processes received shutdown. Check for any lingering gz sim server before restarting.'
}
trap cleanup EXIT INT TERM
ros2 launch cafe_autonomy 01_world.launch.py &world_pid=$!
ready='false'
for _ in $(seq 1 40);do
  if ! kill -0 "$world_pid" 2>/dev/null;then echo 'World launch exited';exit 3;fi
  if timeout 3 gz model --list 2>/dev/null | grep -q 'cafe_structure\|cafe_counter';then ready='true';break;fi
  sleep 2
done
if [[ "$ready" != 'true' ]];then echo 'No working cafe world; refusing to start robots';exit 4;fi
MAP_ARG=()
if [[ -n "${CAFE_MAP_YAML:-}" ]]; then
    if [[ ! -s "$CAFE_MAP_YAML" ]]; then echo "Map missing or empty: $CAFE_MAP_YAML" >&2;exit 5;fi
    MAP_ARG=("map_yaml:=$CAFE_MAP_YAML")
fi
ros2 launch cafe_autonomy 02_robots.launch.py "${MAP_ARG[@]}" &stack_pid=$!
wait "$stack_pid"
