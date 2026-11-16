# Same robot, separate mapping role

Robot 01 keeps the existing TEAM chassis, sensors and URDF. The personal entry point reuses the original `serving_robot_bringup/mapping_mode.launch.py`: SLAM Toolbox, mapping Nav2, Collision Monitor, soft stop, robot health and frontier extraction. The original TEAM packages were not edited. Robot 02, serving Fleet, AMCL and people simulation are not launched in this role.

Mapping and operation are mutually exclusive under the existing runtime lock. Mapping is a software role, not a third robot. The mapping Nav2 configuration belongs to TEAM; personal serving still uses its own MPPI/Rotation Shim configuration.

From WSL, in `team_project`:

```bash
# Terminal 1: same Robot 01, mapping system
bash scripts/start_mapping.sh --headless --site config/integration/sites/demo_lidar_complete.yaml
# Terminal 2: bounded autonomous exploration and draft map save
bash scripts/run_mapping_session.sh --max-seconds 1200
# Stop the owned runtime after exploration/finalization finishes
bash scripts/stop_autonomy.sh
```

The site overlay supplies the world and starting pose. Without a world override the mapping launch defaults to TEAM's closed cafe world. The previous `start_autonomy.sh --mode mapping` SLAM-only scan lab remains available; it does not start the autonomous TEAM explorer.

The new session wrapper checks lifecycle activation, fresh map and odometry, then invokes the original TEAM session/finalizer. It retries unanswered lifecycle requests. Its child-local ROS CLI adapter adds `--no-daemon --spin-time 3` only to `ros2 node list`, because the daemon graph intermittently omitted existing safety nodes. The original required/forbidden node checks and safety checks remain in force.

Outputs go to unique `maps/drafts/<site>_<timestamp>` directories. The operation map YAML and image hashes are recorded and compared. Nothing promotes a partial map into operation automatically. A new store still needs reviewed map coverage, map-frame starting poses, destinations/table orientations, and corridor/site configuration. Table semantics are not inferred by SLAM.

## Validation

- Personal integration suite: 154 passed; personal launch build succeeded; changed Python files compile.
- Gazebo run `maps/drafts/demo_lidar_complete_final_20261031_234957`: all six required lifecycle nodes active; map and odometry received; original TEAM Explorer dispatched three Nav2 goals, with the first two successful.
- Maximum odometry displacement from session start: 2.817 m; 43 observed map updates. Saved live SLAM map: 159 x 121 at 0.05 m/cell.
- Bounded 60-second test ended `TIME_LIMIT_REACHED`; raw map saved successfully. Strict coverage check reported `FREE_SPACE_REACHES_MAP_EDGE`, handoff `PARTIAL_MAP_CHECKPOINT`, `map_complete=false`. This verifies the mapping chain, not completion of the entire cafe map.
- Initial preparation failures were a pending lifecycle-response wait and incomplete CLI daemon discovery; recorded in the earlier draft session directories. Neither was bypassed by disabling robot safety.
- Protected TEAM mapping/bringup/navigation/description/Gazebo files and Robot 02 Nav2 config: git diff clean.

No hardware test, automatic full-map approval, commit or push was performed.
