"""Build the exact-version, mapping-only endpoint-extent patch. No apt changes."""
import argparse,os,subprocess,shlex
from pathlib import Path

root=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--workspace',default=str(root/'_local_only/mapping_slam_285'));args=ap.parse_args()
workspace=Path(args.workspace).resolve();source=workspace/'src/slam_toolbox'
workspace.mkdir(parents=True,exist_ok=True);source.parent.mkdir(exist_ok=True)
if not source.exists():subprocess.run(['git','clone','--depth','1','--branch','2.8.5','https://github.com/SteveMacenski/slam_toolbox.git',str(source)],check=True)
commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
if commit!='ec8f7635dea317b531c419f798f87d90a336f32e':raise SystemExit('Unexpected SLAM source revision')
patch=root/'ros_ws/src/serving_robot_mapping/patches/karto_endpoint_extent.patch'
applied=subprocess.run(['git','apply','--reverse','--check',str(patch)],cwd=source,capture_output=True).returncode==0
if not applied:
    subprocess.run(['git','apply','--check',str(patch)],cwd=source,check=True)
    subprocess.run(['git','apply',str(patch)],cwd=source,check=True)
env=dict(os.environ,CMAKE_BUILD_PARALLEL_LEVEL='2')
command='source /opt/ros/jazzy/setup.bash && colcon build --packages-select slam_toolbox --allow-overriding slam_toolbox --cmake-args -DBUILD_TESTING=OFF'
subprocess.run(['bash','-lc',command],cwd=workspace,env=env,check=True)
print('Mapping-only setup:',workspace/'install/setup.bash')
