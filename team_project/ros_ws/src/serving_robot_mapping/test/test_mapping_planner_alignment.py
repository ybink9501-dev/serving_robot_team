import copy,importlib.util,json,math
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('mapping_alignment_launch',Path(__file__).resolve().parents[2]/'cafe_autonomy/launch/mapping_robot.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

@pytest.mark.parametrize('resolution',[.025,.05])
def test_mapping_global_footprint_matches_stricter_path_and_goal_envelopes(resolution):
    slam={'slam_toolbox':{'ros__parameters':{'resolution':.05}}}
    nav={'global_costmap':{'global_costmap':{'ros__parameters':{'resolution':.05,'footprint':'old','footprint_padding':.02,'inflation_layer':{'inflation_radius':.45,'cost_scaling_factor':3.}}}},
         'controller_server':{'ros__parameters':{'general_goal_checker':{'xy_goal_tolerance':.15},'max_speed':.25}},
         'local_costmap':{'unchanged':'body'},'planner_server':{'ros__parameters':{'GridBased':{'allow_unknown':True,'plugin':'nav2_navfn_planner::NavfnPlanner'}}}}
    before=copy.deepcopy(nav)
    _,changed=module.mapping_precision_parameters(slam,nav,resolution,.01,align_clearance=True)
    c=changed['global_costmap']['global_costmap']['ros__parameters'];p=json.loads(c['footprint'])
    radius=max(.48+(resolution+.05)*math.sqrt(2)/2,.52+resolution*math.sqrt(2)/2)
    for a,b in zip(p,p[1:]+p[:1]):
        distance=abs(a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0],b[1]-a[1])
        assert distance>=radius-1e-12
    assert c['footprint_padding']==0
    assert c['inflation_layer']['inflation_radius']>=radius+.1
    assert c['inflation_layer']['cost_scaling_factor']==3.
    assert changed['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    assert changed['controller_server']['ros__parameters']['max_speed']==.25
    assert changed['local_costmap']==before['local_costmap']
    assert nav==before
