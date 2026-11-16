#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/_local_only"
set +u
source /opt/ros/jazzy/setup.bash
cd "$ROOT/ros_ws"
colcon --log-base "$ROOT/_local_only/colcon" build --symlink-install 2>&1 | tee "$ROOT/_local_only/build.log"
