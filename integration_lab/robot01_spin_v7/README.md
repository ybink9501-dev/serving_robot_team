# Robot 01 LAB V7 — excessive turn protection and map-based arrival evidence

Scope: Existing TEAM cafe.world.sdf + V3 serving_v3 + V6 recorder only. Does **not** use the private café project.

Existing V6 log independently establishes 197° turning with 0.04 m net progress over 12 s, and `NAV2_SUCCEEDED_NOT_SERVING → FINAL_ACCURACY_NOT_REACHED`. Original V6 odom x/y are in `odom` frame; never compare those with the `map`-frame destination x/y.

- `analyze_v6.py` reads previously recorded commands and statuses without ROS.
- `monitor.py` records **map-frame** xy/yaw residuals to each V3 waypoint via TF, and monitors odometry for excessive turning without progress.
- Default `monitor.py` is observe-only; `--enforce` sends an existing V3 serving CANCEL only on verified excessive spin. It **does not publish `cmd_vel`, change Nav2 configuration or thresholds, or create goals**.
- The no-progress thresholds are read from the already installed `robot01_pretest_v6/config.yaml` (12 seconds, 0.08m, π rad); no new magic numbers.
- A cancellation command is asynchronous and is not a hardware emergency stop. **Operator must watch Gazebo and pause simulation if stopping fails**. No claim of guaranteed hard 180° cap.
- Only start `--enforce` after confirming V3 `IDLE` and no active goals, and before the next deliberately scheduled stationary-area test.
- Log target examples: `/tmp/robot01_spin_arrival_v7_20261018.jsonl` (must not overwrite an existing file).
- This patch does not change arrival threshold 0.001m nor stop yaw tolerance; they remain an explicit design decision pending actual map-frame measured errors.

Run tests: `python3 -m unittest discover -s ~/team_robot_lab/robot01_spin_v7/tests -v`
Analyze existing log: `python3 ~/team_robot_lab/robot01_spin_v7/analyze_v6.py /tmp/robot01_v6_20261018_022139.jsonl`
Run monitor in future test only: `python3 ~/team_robot_lab/robot01_spin_v7/monitor.py --enforce --out /tmp/robot01_spin_arrival_v7_YYYYMMDD.jsonl`
