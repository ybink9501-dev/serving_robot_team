# Robot01 current team café — Serving V3 (LAB only)

## What this changes
- No changes to `team_serving_robot` source files, Gazebo, Nav2, sensors, AMCL or Safety.
- Reads **the existing team project** `serving_robot_gazebo/worlds/cafe.world.sdf`, **not** the individual café world.
- Reuses existing `build_cafe_map.py` geometry parser from the installed Oct 16 LAB additions.
- Builds one immutable semantic waypoint snapshot per `map_revision` (table ID, top/bottom options, counter pickup).
- When a table or kitchen is moved/rotated in SDF, regenerate waypoints from model coordinates and local approach offsets.
- The map may expand or change origin. Current `frame_alignment.mode=sdf_aligned_lab_only` is valid **only for the SDF-derived LAB map**. Do not use on a newly scanned SLAM map without its measured world→map transform and a new revision.
- Validates waypoint footprint against map PGM and SDF obstacles, map geometry, AND binary obstacle occupancy against the ROS `/map` contents (not merely metadata). Rejects stale snapshots when input files change.
- Table left/right approaches are represented as inactive candidates because chairs block those directions. Top/bottom are active. A physical chair relocation and new validation is required before enabling.
- Resolves `table_01` -> a reachable `table_01_top/bottom` via Nav2 ComputePathToPose, selecting shortest planned path (no motion during planning).
- Kitchen arrival -> **WAIT_LOAD** until staff confirms food loading (`load`). Table physical arrival -> **WAIT_HANDOVER** until customer confirms (`complete`) -> counter pickup return. LAB automatic gates configurable but manual is default.
- Existing destination_server continues to gate `SERVING` on Nav2 success + physical position/orientation/stationarity.
- `HOME` standby/charging remains distinct. `counter_pickup` and `kitchen` share a provisional physical point in current LAB, not a team-approved food pickup pose.

## Installation
Download `Robot01_dynamic_serving_v3.zip` from the chat and place in Windows Downloads.

```bash
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
unzip -n /mnt/c/Users/user/Downloads/Robot01_dynamic_serving_v3.zip -d "$HOME"
python3 ~/team_robot_lab/serving_v3/prepare.py
python3 ~/team_robot_lab/serving_v3/activate.py
```

Only activate.py restarts existing destination_server. No Nav2/Gazebo restart. It refuses when robot is moving, Nav2 goal active, or ROS_DOMAIN_ID wrong. Before activation, preparation modifies no live ROS nodes.

## After activation (CHECK terminal)

```bash
python3 ~/team_robot_lab/serving_v3/serving_control_v3.py status
# The following starts REAL motion, not a test:
python3 ~/team_robot_lab/serving_v3/serving_control_v3.py start table_01
# At kitchen, manually confirm the food is loaded:
python3 ~/team_robot_lab/serving_v3/serving_control_v3.py load
# After confirmed table arrival:
python3 ~/team_robot_lab/serving_v3/serving_control_v3.py complete
# To cancel:
python3 ~/team_robot_lab/serving_v3/serving_control_v3.py cancel
```

Explicit approach: `start table_03 --side bottom` (active top/bottom only).
The explicit `start` command is **not included** in the setup commands. Setup never moves the robot.

## When the map or furniture changes
1. Update actual team SDF and/or authorized Mapping-produced map (not shared empty placeholders). Do not change these on a live mission.
2. When V3 is already running, finish/cancel its current mission, then run `python3 ~/team_robot_lab/serving_v3/deactivate.py` to stop only the V3 supervisor. Change `layout.yaml` (`map_file`, `map_revision`, map alignment if applicable, approach offsets).
3. Run `prepare.py` and inspect list of enabled/disabled candidates. If all candidates blocked, it fails without activation. Deploy the updated ROS `/map` through the authorized Mapping/Operation process before activating.
4. Stop old missions and activate new snapshot via `activate.py`; do not hot-edit active YAML while driving.
5. If map content changes while metadata remains identical, destination_server and Nav2 may not detect it. Only deploy new map with its own revision and restart localized operation safely.

## Current limitations / not implemented
- Real ROS/Gazebo end-to-end serving trip not tested here.
- Current destination_server's final precision threshold (1 mm) may block a successful `SERVING` state even when navigation succeeds.
- Fleet-level exclusive zone locks, robot_02 coordination and robot-robot collision prevention are **not implemented by V3**.
- Static chairs cannot be passed through by this module. Left/right candidates are intentionally inactive.
- No automatic SLAM object detection or semantic auto-tagging; when a new SLAM map is produced, geometric alignment is a Mapping/Setup task.
- V3 compares full binary occupied/free occupancy to generated PGM. Unknown cells or shifted occupancy block new requests; it does not prove physical LiDAR alignment.
- The existing team uses Task/Robot contract names that differ from this LAB `/robot_01/serving/*` adapter. Frontend/Fleet integration remains necessary.

Logs: `/tmp/robot01_destination_v3.log`, `/tmp/robot01_supervisor_v3.log`.
