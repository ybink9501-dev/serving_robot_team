# Robot 01 diagnostics V9

- `amcl_graph_check.py`: waits 10 seconds for ROS graph discovery then prints publisher/subscriber nodes for `/initialpose`, `/robot_01/initialpose`, `/map`, `/tf`, `/robot_01/scan` and AMCL. **Read only**. No action goals, pose publication or changes to existing code.
- `nightly_offline.py`: Runs already-installed unit tests and randomized **ROS-free** V7 anti-spin policy cases (400 per round); summarizes existing logs. No motion, no ROS nodes, no code edits. It does **not** prove live Gazebo/AMCL correctness or optimize code without a follow-up analysis.

Use in isolated LAB only. Output logs go to `~/team_robot_lab/robot01_diagnostics_v9/reports/`. Stop the overnight job with `kill <pid>`; Windows sleep can suspend WSL work.
