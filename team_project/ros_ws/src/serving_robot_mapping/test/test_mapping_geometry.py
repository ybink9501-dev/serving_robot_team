import copy,json,math
import pytest
from serving_robot_mapping.mapping_geometry import MappingGeometry


def parameters(budget):
    radius=budget.planner_radius/math.cos(math.pi/16)+1e-9
    vertices=[[radius*math.cos(i*math.pi/8),radius*math.sin(i*math.pi/8)] for i in range(16)]
    slam={'slam_toolbox':{'ros__parameters':dict(use_sim_time=True,transform_publish_period=.05,transform_timeout=.2)}}
    nav={n:{'ros__parameters':dict(use_sim_time=True)} for n in ['controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']}
    nav['global_costmap']={'global_costmap':{'ros__parameters':dict(footprint=json.dumps(vertices))}}
    return slam,nav


def test_shared_budget_validates_without_modifying_clock_or_safety_settings():
    budget=MappingGeometry();slam,nav=parameters(budget);before=copy.deepcopy((slam,nav))
    result=budget.validate_generated(slam,nav)
    assert result['perimeter_center_clearance_m']>result['planner_radius_m']
    assert result['tf_settings_modified'] is False
    assert (slam,nav)==before


def test_clock_mismatch_and_undersized_footprint_fail_before_motion():
    budget=MappingGeometry();slam,nav=parameters(budget)
    nav['controller_server']['ros__parameters']['use_sim_time']=False
    with pytest.raises(ValueError,match='simulation time'):budget.validate_generated(slam,nav)
    slam,nav=parameters(budget)
    nav['global_costmap']['global_costmap']['ros__parameters']['footprint']='[[.2,.2],[.2,-.2],[-.2,-.2],[-.2,.2]]'
    # JSON numeric literals cannot start with a decimal point.
    nav['global_costmap']['global_costmap']['ros__parameters']['footprint']=json.dumps([[.2,.2],[.2,-.2],[-.2,-.2],[-.2,.2]])
    with pytest.raises(ValueError,match='smaller'):budget.validate_generated(slam,nav)


def test_physical_body_and_hardstop_must_fit_clearance_budget():
    budget=MappingGeometry()
    body=[[.26,.2],[.26,-.2],[-.26,-.2],[-.26,.2]]
    assert budget.validate_body(body,.1)['path_clearance_slack_m']>0
    result=budget.validate_body(body,.1,.33)
    assert result['minimum_body_clearance_budget_m']==pytest.approx(math.hypot(.36,.3))
    assert result['occupied_cell_center_required_m']<result['guard_occupied_cell_center_minimum_m']
    with pytest.raises(ValueError,match='Safety radius'):
        budget.validate_body(body,.1,.2)
    with pytest.raises(ValueError,match='physical body'):
        budget.validate_body([[x*2,y*2] for x,y in body],.1)
