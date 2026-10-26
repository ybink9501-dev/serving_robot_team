# Robot 01 V4 — near-goal stuck handoff (LAB trial patch)

This patch is limited to LAB `precision_v4` and **does not** change TEAM code, Nav2
2.5mm XY tolerance, detection 5mm, final physical judgment 1mm, max velocities,
MPPI, Nav2/safety costmaps or footprints.

## Why

The user trial on 2026-10-20: robot near Table01 Bottom had map TF xy error
around 0.0516m. Nav2 cmd_vx practically zero while Goal ACTIVE; V4 fine approach
was gated on Nav2 SUCCEEDED, thus never started. After that trial the costmap probe
showed goal value 35 and 0.53m to nearest lethal obstacle (robot 41, 0.50m).
This rules out a lethal GOAL CELL at this snapshot, **not** all collision risks.

## New *experimental* handoff parameters (not user confirmed speed/tolerance changes)

`near_goal_handoff.enabled=true`, `radius_m=0.06m`, `stalled_for_s=6.0s`.
Handoff is requested ONLY if there is a Nav2 active goal, robot has been inside 6cm
and near-stationary for six seconds with fresh odometry and limited pose drift.
The server requests Nav2 CANCEL and **does not emit fine approach velocity until
Nav2 reports the terminal STATUS_CANCELED**. External CANCEL always wins;
ABORTED/failed cancels will never authorize fine motion. No Nav2 goal reissue.
The existing V4 fine approach and original Safety chain are unchanged.

**This is a controlled LAB experiment only.** Safety chain continuing to run does
not prove the path is clear. Do not run unattended; abort if collision risk.
The action server returning STATUS_CANCELED is NOT interpreted as a completed
navigation task; only physical XY/Yaw/stopped checks may mark SERVING.

## Offline tests

`test_fine_approach.py`, `test_v4_server.py`,
`test_v3_regression_on_v4.py` (8/8), `test_near_goal_handoff.py` (8/8).
`activate_v4.py` now runs all four and aborts if any fails.

## WSL (provided latest team-first V4 is already installed)

Keep Gazebo, robot_01, Nav2, Safety & Supervisor as-is. Do NOT run multiple Gazebo,
robot spawn or nav stacks. Confirm the existing mission is CANCELED and robot stopped.

```
cp -a ~/team_robot_lab/precision_v4 ~/team_robot_lab/precision_v4_before_neargoal
unzip -o /mnt/c/Users/user/Downloads/robot01_v4_near_goal_handoff_LAB.zip -d "$HOME"
python3 ~/team_robot_lab/precision_v4/activate_v4.py --check-only
```

If CHECK-ONLY PASSED, and robot stationary with no active goal:

```
python3 ~/team_robot_lab/precision_v4/activate_v4.py
```

No `--apply-nav2-file` is needed (and must not be used here).
After this confirms V4 ACTIVATED, run **one** monitored trial with existing
`~/robot01_one_trial/run_trial.py --run`. Do not infer 1mm success until
`PHYSICAL_ARRIVAL_VERIFIED` and independent Gazebo pose are observed.
