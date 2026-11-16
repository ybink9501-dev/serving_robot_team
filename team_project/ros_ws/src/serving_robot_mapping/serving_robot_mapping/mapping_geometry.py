"""One mapping geometry budget; never changes serving or TF tolerances."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MappingGeometry:
    resolution: float = .05
    costmap_resolution: float = .05
    path_clearance: float = .48
    goal_clearance: float = .52
    arrival_error: float = .01

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.resolution,self.costmap_resolution,self.path_clearance,self.goal_clearance,self.arrival_error)):
            raise ValueError('Invalid mapping geometry budget')
        if self.arrival_error > min(self.resolution/2,self.goal_clearance-self.path_clearance):
            raise ValueError('Arrival uncertainty exceeds mapping clearance budget')

    @property
    def planner_radius(self):
        return max(self.path_clearance+(self.resolution+self.costmap_resolution)/math.sqrt(2),
                   self.goal_clearance+self.resolution/math.sqrt(2))

    @property
    def perimeter_radius(self):
        return self.planner_radius+self.costmap_resolution/math.sqrt(2)+self.arrival_error

    def validate_body(self, vertices, hard_stop_clearance, safety_radius=None):
        radius=max(math.hypot(float(x),float(y)) for x,y in vertices)
        expanded=math.hypot(max(abs(float(x)) for x,y in vertices)+hard_stop_clearance,
                            max(abs(float(y)) for x,y in vertices)+hard_stop_clearance)
        if safety_radius is not None and (not math.isfinite(safety_radius) or safety_radius<radius):
            raise ValueError('Safety radius does not contain physical body')
        # The path guard already inflates occupied cells by their half diagonal.
        # Compare edge clearances, then expose the center-distance equivalent.
        required=max(expanded,(safety_radius or radius)+float(hard_stop_clearance))
        if not math.isfinite(required) or hard_stop_clearance<0 or required>self.path_clearance:
            raise ValueError('Mapping path clearance does not contain physical body and HardStop')
        return dict(body_corner_radius_m=radius,hard_stop_clearance_m=hard_stop_clearance,
                    configured_safety_radius_m=safety_radius,rectangular_hardstop_corner_radius_m=expanded,
                    minimum_body_clearance_budget_m=required,path_clearance_m=self.path_clearance,
                    path_clearance_slack_m=self.path_clearance-required,
                    occupied_cell_center_required_m=required+self.resolution/math.sqrt(2),
                    guard_occupied_cell_center_minimum_m=self.path_clearance+self.resolution/math.sqrt(2))

    def validate_generated(self, slam, nav):
        """Fail before motion on incompatible geometry/clocks; do not tune clocks."""
        sp=slam['slam_toolbox']['ros__parameters']
        costmap=nav['global_costmap']['global_costmap']['ros__parameters']
        import json
        vertices=json.loads(costmap['footprint'])
        radius=min(abs(a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0],b[1]-a[1])
                   for a,b in zip(vertices,vertices[1:]+vertices[:1]))
        if radius+1e-10 < self.planner_radius:
            raise ValueError('Global planner footprint is smaller than mapping safety envelope')
        nodes=['controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother']
        if not sp.get('use_sim_time') or any(not nav[n]['ros__parameters'].get('use_sim_time') for n in nodes):
            raise ValueError('Mapping nodes must share simulation time')
        period=float(sp['transform_publish_period'])
        if not 0 < period < .5:
            raise ValueError('TF publication period does not fit existing .5s freshness gate')
        return dict(planner_radius_m=radius,required_planner_radius_m=self.planner_radius,
                    perimeter_center_clearance_m=self.perimeter_radius,arrival_error_m=self.arrival_error,
                    use_sim_time=True,tf_publish_period_s=period,tf_freshness_limit_s=.5,
                    transform_timeout_s=float(sp['transform_timeout']),tf_settings_modified=False)
