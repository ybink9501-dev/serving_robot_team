#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m compileall -q "$ROOT/ros_ws/src/cafe_autonomy/cafe_autonomy" "$ROOT/ros_ws/src/cafe_autonomy/launch" "$ROOT/tools"
PYTHONPATH="$ROOT/ros_ws/src/cafe_autonomy${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -v -s "$ROOT/ros_ws/src/cafe_autonomy/tests"
python3 "$ROOT/tools/source_audit.py" "$ROOT"
