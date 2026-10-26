# V4 Near-goal handoff — Nav2 settings preservation patch

- Team/consensus values and all Nav2/Velocity Smoother/Safety values remain unchanged.
- `activate_v4.py --check-only`: runs offline tests, ROS topic chain checks, **read-only** six Nav2 parameter checks and robot-stopped/no-active-goal check. Does not restart a server.
- `activate_v4.py`: does the above, then restarts **only** Destination Server V4 (original backup/rollback-on-start-failure logic retained).
- `--apply-nav2-file`: deliberately rejected. Nav2 YAML files are never overwritten and `ros2 param set` is never executed.
- Runtime values expected: PoseProgressChecker, 0.03 m progress radius, **10.0 s progress allowance**, XY 0.0025 m, yaw 0.25 rad, stateful false. If mismatched, abort instead of changing anything.
- `near_goal_handoff` stays experimental at 0.06 m/6 s, not confirmed final mm accuracy.
- Requirement: existing LAB V4 includes Safety pipeline and one Destination Server; 1mm Gazebo ground-truth accuracy not yet shown.

## Usage (existing CHECK tab, after robot stopped)

Copy ZIP into Windows Downloads, extract into `$HOME` after backing up `~/team_robot_lab/precision_v4`, then run:

```
python3 ~/team_robot_lab/precision_v4/verify_confirmed_values.py
python3 ~/team_robot_lab/precision_v4/activate_v4.py --check-only
python3 ~/team_robot_lab/precision_v4/activate_v4.py
```

Never use `--apply-nav2-file`; it is disabled.
