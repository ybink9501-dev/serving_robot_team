# V12 — Robot 01 screenshot stasis triage & actual Gazebo footage

This package is **read-only** to Gazebo/ROS. It does **not** publish `/cmd_vel`, `/initialpose`, goals, or cancel commands. It does **not** simulate movements or reanimate old logs. Capturing the GUI requires Gazebo to visibly run on Windows desktop and Windows `ffmpeg.exe`. Existing V10 60-second recorder reported `NO_NEW_MISSION_OBSERVED` and zero velocity — a static frame is expected; it is not a valid moving navigation video.

## 1. Check the robot BEFORE attempting motion (WSL CHECK)

```
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
source ~/team_robot_lab/ros_ws/install/local_setup.bash
python3 ~/team_robot_lab/robot01_gazebo_capture_v12/check_robot01.py --seconds 12
```

Send entire JSON report to assistant. No new `START` until confirmed no active goal and no unsafe contact with wall/counter; the screenshot suggests the robot is close to the counter's western end / wall, not proof of collision. Start only after STOP/PAUSE/active mission understood.

## 2. Capture REAL Gazebo window (not data visualizations)

Windows PowerShell:
```
winget install --id Gyan.FFmpeg -e
# Open a fresh PowerShell after install
& "$env:USERPROFILE\...\record_gazebo.ps1" -Seconds 180
```

Or from WSL after Windows ffmpeg is available:
```
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w "$HOME/team_robot_lab/robot01_gazebo_capture_v12/record_gazebo.ps1")" -Seconds 180
```

Record `title=Gazebo Sim` by default. If the window title is inaccessible via WSLg, add `-Desktop` and keep Gazebo visible. Saved to `%USERPROFILE%\Videos\Robot01_Gazebo_REAL_*.mp4`.

RECORD BEFORE the explicit real motion test, while status/pose are safe; a recording without an active mission will show a stationary robot. Past unseen footage cannot be recovered retroactively from V10 JSONL or V11 MP4.
