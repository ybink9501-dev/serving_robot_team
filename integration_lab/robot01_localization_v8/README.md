# Robot01 AMCL/Gazebo pose recovery V8

This is LAB-only and expects the reviewed V3 map `frame_alignment.mode: sdf_aligned_lab_only`, `world_to_map: {x:0,y:0,yaw:0}`. It never changes V3 YAML, Nav2, Safety, Gazebo, robot command velocity, or goal actions. Requires cancellation, no active Nav2 action, zero odometry velocity, live map matching V3 manifest, and collision-free actual Gazebo pose. Resets AMCL initial pose only when `--apply` is explicit. Read-only by default.

Commands (CHECK):
```
export ROS_DOMAIN_ID=99 GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_serving_robot/ros_ws/install/setup.bash
python3 ~/team_robot_lab/robot01_localization_v8/relocalize.py --apply
```

Success is `PASS` and error <=0.15 m / 12 degrees after `/initialpose`. On `NO_CHANGE` or `CHECK`, leave robot stationary and share output. This tool uses Gazebo pose as known simulation truth for a **known identity-aligned map**; never use on physical robot or an unreviewed SLAM map. A successful pose reset is only a localization recovery; subsequent drift or safety blocking must be addressed separately before resuming navigation.
