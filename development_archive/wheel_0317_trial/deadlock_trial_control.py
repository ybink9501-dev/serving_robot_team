from pathlib import Path
import sys,types,runpy
p=Path("/home/user/team_serving_robot_clean_20261022")
m=types.ModuleType("configure_runtime")
m.configure=lambda root: p/"_local_only/config/deadlock_facing_trial.yaml"
sys.modules["configure_runtime"]=m
sys.argv=["team_control.py","start","--headless"]
runpy.run_path(str(p/"scripts/team_control.py"),run_name="__main__")
