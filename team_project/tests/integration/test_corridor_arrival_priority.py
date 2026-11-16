import json,math
from types import SimpleNamespace as NS
from cafe_autonomy.corridor_priority import CorridorArrivalPriority,forward_entry_observed,priority_yield_options
from cafe_autonomy.policy import FleetCore,Task

Z={'passage':[-1.,1.,-.5,.5]}

def test_first_physical_arrival_beats_older_order_and_robot_id():
    a=CorridorArrivalPriority();a.reset('map-a')
    a.observe({'robot_02':(1.5,0.)},Z,1.,.78)
    poses={'robot_01':(-1.5,0.),'robot_02':(1.5,0.)}
    a.observe(poses,Z,2.,.78)
    p=a.choose('passage',poses,{'robot_01':1,'robot_02':2},poses,Z['passage'])
    assert p['winner']=='robot_02' and p['reason']=='FIRST_ARRIVAL'

def test_same_observation_batch_uses_first_received_signal():
    a=CorridorArrivalPriority();poses={'robot_01':(-1.5,0.),'robot_02':(1.5,0.)}
    a.observe(poses,Z,1.,.78)
    p=a.choose('passage',poses,{'robot_01':12,'robot_02':11},poses,Z['passage'])
    assert p['winner']=='robot_02' and p['reason']=='EARLIER_REQUEST'
    assert a.choose('passage',poses,{'robot_01':None,'robot_02':None},poses,Z['passage']) is None

def test_changed_translated_vertical_map_has_no_previous_arrivals():
    a=CorridorArrivalPriority();a.observe({'robot_01':(-1.5,0.)},Z,1.,.78)
    a.reset('map-b');assert not a.entries
    z={'new-name':[10.,11.,20.,24.]};p={'robot_02':(10.5,24.6)}
    a.observe(p,z,2.,.78);p['robot_01']=(10.5,19.4);a.observe(p,z,3.,.78)
    assert a.choose('new-name',p,{'robot_01':1,'robot_02':2},p,z['new-name'])['winner']=='robot_02'

def test_departure_resets_arrival_and_stale_missing_pose_does_not_invent_one():
    a=CorridorArrivalPriority();p={'robot_01':(-1.5,0.),'robot_02':(1.5,0.)}
    a.observe(p,Z,1.,.78);a.observe({'robot_01':(-4.,0.)},Z,2.,.78)
    assert 'robot_01' not in a.entries['passage'] and 'robot_02' in a.entries['passage']
    a.observe(p,Z,3.,.78)
    assert a.choose('passage',p,{'robot_01':1,'robot_02':2},p,Z['passage'])['winner']=='robot_02'

def test_priority_never_swaps_when_only_winners_yield_route_exists():
    options=[{'robot':'robot_01'}]
    assert not priority_yield_options(options,{'winner':'robot_01'})

def test_duplicate_and_restart_preserve_signal_receipt_sequence():
    c=FleetCore();one,_=c.submit('first','table_1','robot_02');two,_=c.submit('second','table_1','robot_01')
    assert one.received_seq<two.received_seq
    old=(one.received_seq,one.received_at)
    again,new=c.submit('first','table_1','robot_02');assert not new and (again.received_seq,again.received_at)==old
    restored=FleetCore(tasks={t.request_id:Task(**json.loads(json.dumps(vars(t)))) for t in c.tasks.values()})
    third,_=restored.submit('third','table_1','robot_01');assert third.received_seq>two.received_seq

def test_front_perception_requires_fresh_sensor_and_free_visible_map_segment():
    grid=NS(info=NS(width=30,height=30,resolution=.1,origin=NS(position=NS(x=-1.,y=-1.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*900)
    report=dict(sensor_fresh=True,camera_fresh=True,lidar_fresh=True,front_clear_m=2.)
    args=((-1.5,0.),0.,[-1.,1.,-.5,.5],report,grid,5.,.35)
    # Robot is outside this map, so no inferred visibility is permitted.
    assert not forward_entry_observed(*args)
    args=((-0.5,0.),0.,[0.,1.,-.5,.5],report,grid,5.,.35)
    assert forward_entry_observed(*args)
    assert not forward_entry_observed(args[0],math.pi,*args[2:])
    report['lidar_fresh']=False;assert not forward_entry_observed(*args)
    report['lidar_fresh']=True;grid.data[10*30+8]=100
    assert not forward_entry_observed(*args)


def test_corridor_gate_uses_actual_path_entry_for_either_map_axis():
    from cafe_autonomy.corridor_priority import entry_on_path,distance_to_portal
    from cafe_autonomy.yield_policy import opposed_corridor_paths
    z={'arbitrary-name':[10.,11.,20.,24.]}
    p={'robot_01':(9.5,22.),'robot_02':(11.5,22.)};g={'robot_01':(12.,22.),'robot_02':(9.,22.)}
    paths={r:[p[r],g[r]] for r in p}
    assert opposed_corridor_paths(p,g,paths,z)==['arbitrary-name']
    assert entry_on_path(paths['robot_01'],z['arbitrary-name'])==(10.,22.)
    assert distance_to_portal(p['robot_01'],z['arbitrary-name'])==.5

def test_ros_and_http_input_share_one_fifo_before_task_assignment():
    import queue
    from std_msgs.msg import String
    from cafe_autonomy.fleet import Fleet
    m=NS(http_queue=queue.Queue())
    from types import MethodType
    m.enqueue_request=MethodType(Fleet.enqueue_request,m)
    m.enqueue_request({'action':'SUBMIT','request_id':'http-first'})
    Fleet.on_request(m,String(data=json.dumps({'action':'SUBMIT','request_id':'ros-second'})))
    assert m.http_queue.get_nowait()[0]['request_id']=='http-first'
    assert m.http_queue.get_nowait()[0]['request_id']=='ros-second'


def test_blocked_path_can_arbitrate_goal_intent_but_valid_bypass_is_preserved():
    from cafe_autonomy.corridor_priority import reservation_intents
    p={'robot_01':(-2.,0.),'robot_02':(2.,0.)};g={'robot_01':(3.,0.),'robot_02':(-3.,0.)}
    expected={r:(r,g[r]) for r in p}
    paths={'robot_01':[(-2.,2.),(3.,2.)]};contexts={'robot_01':expected['robot_01']}
    routes,sources=reservation_intents(p,g,paths,contexts,expected,{'robot_01':1.},1.5)
    assert routes['robot_01']==paths['robot_01'] and sources['robot_01']=='NAV2_PATH'
    assert routes['robot_02']==[p['robot_02'],g['robot_02']] and sources['robot_02']=='GOAL_INTENT_ONLY'
    contexts['robot_01']=('old-map-or-task',g['robot_01'])
    routes,sources=reservation_intents(p,g,paths,contexts,expected,{'robot_01':1.},1.5)
    assert sources['robot_01']=='GOAL_INTENT_ONLY'

def test_recovery_wait_is_eligible_only_after_goal_inactivity_is_confirmed():
    from cafe_autonomy.yield_policy import preentry_pause_acknowledged
    ids={'robot_01':'one','robot_02':'two'}
    states={r:dict(request_id=rid,state='RECOVERY_WAIT',nav_paused=False,goal_execution_verified_idle=True) for r,rid in ids.items()}
    assert preentry_pause_acknowledged(states,ids)
    states['robot_01']['goal_execution_verified_idle']=False
    assert not preentry_pause_acknowledged(states,ids)
