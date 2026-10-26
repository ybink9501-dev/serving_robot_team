# Team-serving-robot LAB safety connection V4

This is a narrow add-on to **V3**. It does not modify any existing ROS source,
YAML, Nav2 config, V3 supervisor or final velocity chain. Not for production.

## Why
The existing Gazebo bridge publishes a `PersonSafetyEvent` to
`/robot_01/person_safety_event_lab`. The original `semantic_safety` can be
configured to consume either that exact source or the standard
`/robot_01/person_safety_event` source. V4 **never adds another safety velocity
publisher**. It checks the live graph and bridges events only if safe.

- Existing `/robot_01/cmd_vel_smoothed` → SemanticSafety →
  `/robot_01/cmd_vel_semantic` → Collision Monitor → `/robot_01/cmd_vel` remains
  unchanged. Collision Monitor stays the one final velocity owner.
- If `semantic_safety` already subscribes to the LAB event topic: no relay.
- If it subscribes to the fallback and nobody publishes there: event-only relay.
- If a production camera pipeline already publishes there: V4 does nothing.
- If subscribers/publishers are not as expected: fail with diagnostic, no motion.
- An invalid event is converted to SENSOR_STALE. If the bridge stops or V4 exits,
  **the existing semantic_safety sensor timeout must fail closed**.
- No navigation GO, robot spawn, gazebo world edits, map edits, or motion tests.

## Install after downloading ZIP to Windows Downloads

```bash
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
ZIP=/mnt/c/Users/user/Downloads/Robot01_person_safety_v4.zip
[ -f "$ZIP" ] || { echo "ZIP not found: $ZIP"; exit 1; }
unzip -n "$ZIP" -d "$HOME"
python3 "$HOME/team_robot_lab/safety_v4/person_chain_lab.py" --inspect
```

Only if `--inspect` reports `RELAY_ALLOWED`, activate with

```bash
nohup python3 -u "$HOME/team_robot_lab/safety_v4/person_chain_lab.py" --apply \
  > /tmp/robot01_person_safety_v4.log 2>&1 < /dev/null &
```

If it reports `DIRECT_READY`, no new process is required. If anything else,
stop and inspect the ROS graph; **do not** run a second safety publisher.

Run offline tests without ROS:

```bash
cd ~/team_robot_lab/safety_v4
python3 -m pytest -q tests/test_person_chain_policy.py
```

## Scope warning
`PersonSafetyEvent` from Gazebo ground-truth is a **LAB simulation-only** signal;
production must use an authorized perception source. This V4 does not implement
actual path replanning nor robot-robot safety. Robot02 and Zone/Fleet safety
remain separate tasks.
