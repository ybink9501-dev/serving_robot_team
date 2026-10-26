Gazebo and RViz2 LIVE SIMULATION GIF (Windows PowerShell + ffmpeg.exe)

Records actual GUI frames, not a synthetic simulation. Does not publish ROS topics, change
Gazebo settings, move robots, or generate previously unseen driving sequences.

From WSL (replace folder with location where you extracted this archive):
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w "$HOME/team_robot_lab/robot01_gazebo_capture_v12/record_real_sim_gif.ps1")" -Target Gazebo -Seconds 30

RViz2:
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w "$HOME/team_robot_lab/robot01_gazebo_capture_v12/record_real_sim_gif.ps1")" -Target RViz -Seconds 30

Window title may differ (especially WSLg). Use -WindowTitle 'Exact visible title' or
-Target Desktop to capture the visible desktop instead. Windows ffmpeg.exe must be on PATH.
Output: %USERPROFILE%\Videos\Robot01\Robot01_Gazebo_*.gif etc.

Run while actual driving is in progress; otherwise it will capture a stationary robot.
GIF stores animation only (no audio).
