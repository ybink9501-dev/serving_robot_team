# Robot 01 pre-drive foundation V6 — TEAM café only

## Scope / safety

- This bundle does **not** create Robot 02, modify `~/team_serving_robot`, restart Nav2, Gazebo, Safety or V3, publish motion, or install new map files.
- Uses existing `~/team_robot_lab/serving_v3/` and `~/team_serving_robot/.../cafe.world.sdf` by reference; no individual café map.
- Do not run `prepare.py` or `activate.py` on an active robot just to make a preflight pass. A changed map requires an authorized stopped deployment and new `map_revision`.
- Gazebo robot movements and ROS 2 live validation are possible ONLY in the user's WSL. Offline unit tests here do **not** prove actual driving.

## One-time install

```bash
unzip -n /mnt/c/Users/user/Downloads/Robot01_pretest_v6.zip -d "$HOME"
```

## Run before starting an actual serving test

```bash
export ROS_DOMAIN_ID=99 GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
python3 ~/team_robot_lab/robot01_pretest_v6/preflight.py --live --out /tmp/robot01_pretest_v6.json
```

Expected `ready: true` if files/ROS graph match and Robot 01 is stationary. **Warnings are NOT passed functional tests**. On `ready: false`, do not run `serving_control_v3.py start`.

## Optional Robot 01-only speed manager (no Robot 02 started)

```bash
nohup python3 -u ~/team_robot_lab/robot01_pretest_v6/speed_manager.py \
  >/tmp/robot01_speed_manager_v6.log 2>&1 </dev/null &
python3 ~/team_robot_lab/robot01_pretest_v6/speed_control.py normal
```

Manager publishes nothing on startup; only explicit `low|normal|high` commands publish a `nav2_msgs/SpeedLimit` to `/robot_01/speed_limit`, never `/cmd_vel`. `PUBLISHED` means the controller has a subscription and a speed limit was sent; **actual odometry verification is still required**. Runtime changes of `config.yaml` are checked on each request and need no restart, but cannot override existing Safety. Choose the profile before driving and record actual speed later.

## Passive evidence collector (start in WORK terminal before real test)

```bash
python3 ~/team_robot_lab/robot01_pretest_v6/recorder.py --duration 420
```

Records ROS Robot 01 odom, `cmd_vel`, serving states, Nav2 events and person events to a new `/tmp/robot01_v6_*.jsonl`. Flags >180 degrees in a 12s window without 8cm progress; this is a **warning, not a speed limiter or automatic cancel**. Do not conflate total route heading changes with needless spin. No moving robot or control takeover.

## Offline unit tests

```bash
python3 -m unittest discover -s ~/team_robot_lab/robot01_pretest_v6/tests -v
```

## What still must be tested on the actual robot

- GO pickup -> WAIT_LOAD -> GO table -> WAIT_HANDOVER -> GO pickup -> COMPLETED.
- Reliable exact arrival; the current arrival YAML may have an extremely tight final threshold (warned, not changed).
- Real braking and no-reverse behavior, correct forward-facing table approach, no unnecessary spin, cancel / Nav2 failure recovery.
- Person detection to actual slowdown/stop/replan and speed changes through actual odom (later scenarios).

Robot 02, Fleet, Backend, Frontend and full map editor are not prerequisites for **direct CLI LAB** Robot 01 tests; they are needed for project-wide end-to-end service tests.
