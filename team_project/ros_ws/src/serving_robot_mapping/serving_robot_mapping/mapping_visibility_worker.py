"""One immutable observation job; no ROS, goals, velocity or map writes."""
from concurrent.futures import ThreadPoolExecutor
from .mapping_frontier_clearance import FrontierClearanceGuard
from .mapping_visibility_fallback import select_visibility_fallback


def evaluate(grid,robot_xy,visited,goal_clearance,start_clearance,sensor_range,inspections=(),rejected=()):
    guard=FrontierClearanceGuard(goal_clearance,start_clearance)
    guard.update_map(*grid)
    from functools import lru_cache
    rejected=set(rejected)
    @lru_cache(maxsize=None)
    def goal_clear(x,y):
        if (round(x,2),round(y,2)) in rejected:return False,'CURRENT_MAP_PATH_REJECTED'
        return guard.goal_clear(x,y)
    points,metrics=select_visibility_fallback(grid,robot_xy,visited,goal_clear,sensor_range_m=sensor_range)
    if not points and metrics['unknown_boundary_cells']:
        from .mapping_edge_observation import select_edge_observations
        probes,detail=select_edge_observations(grid,robot_xy,goal_clear,sensor_range,inspections)
        points=[tuple(p['xy']) for p in probes]
        metrics['edge_probes']=probes;metrics['edge_probe_diagnostic']=detail
        metrics['geometry_candidates']=len(probes)
    return points,metrics


class VisibilityWorker:
    def __init__(self,evaluator=evaluate):
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='mapping_observation')
        self.future=None;self.key=None;self.evaluator=evaluator;self.closed=False

    def poll(self,key,grid,robot_xy,visited,goal_clearance,start_clearance,sensor_range,inspections=(),rejected=()):
        if self.closed:raise RuntimeError('Observation worker is closed')
        if self.future is None:
            snapshot=(tuple(grid[0]),*grid[1:])
            self.key=key
            self.future=self.pool.submit(self.evaluator,snapshot,tuple(robot_xy),tuple(map(tuple,visited)),
                                         goal_clearance,start_clearance,sensor_range,tuple(map(tuple,inspections)),tuple(map(tuple,rejected)))
            return None
        if not self.future.done():return None
        future=self.future;source_key=self.key;self.future=None
        return source_key,future.result()

    def close(self):
        self.closed=True
        self.pool.shutdown(wait=False,cancel_futures=True)
