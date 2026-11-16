from cafe_autonomy.yield_policy import opposed_corridor_paths,reserved_winner_clear

def fixture():
    poses={'robot_01':(-2.,0.),'robot_02':(2.,0.)}
    goals={'robot_01':(3.,0.),'robot_02':(-3.,0.)}
    paths={r:[poses[r],goals[r]] for r in poses}
    return poses,goals,paths,{'lane':[-1.,1.,-.5,.5]}

def test_opposite_actual_paths_are_reserved_before_entry():
    p,g,paths,z=fixture();assert opposed_corridor_paths(p,g,paths,z)==['lane']
    paths['robot_01']=[(-2.,2.),(2.,2.)]
    assert not opposed_corridor_paths(p,g,paths,z)

def test_same_direction_inside_and_missing_path_use_normal_safe_policy():
    p,g,paths,z=fixture();p['robot_01']=(0.,0.)
    assert not opposed_corridor_paths(p,g,paths,z)
    p['robot_01']=(-2.,0.);g['robot_02']=(3.,0.)
    assert not opposed_corridor_paths(p,g,paths,z)
    g['robot_02']=(-3.,0.);paths['robot_02']=[]
    assert not opposed_corridor_paths(p,g,paths,z)

def test_reservation_never_overrides_close_robot_or_occupied_corridor():
    p,g,paths,z=fixture()
    assert reserved_winner_clear(p,'robot_01','robot_02',z,['lane'],.9,.33)
    p['robot_02']=(.8,0.)
    assert not reserved_winner_clear(p,'robot_01','robot_02',z,['lane'],.9,.33)
    p['robot_01']=(1.9,0.);p['robot_02']=(2.,0.)
    assert not reserved_winner_clear(p,'robot_01','robot_02',z,['lane'],.9,.33)


def test_setup_hold_persists_across_path_query_and_yield_acknowledgement():
    from cafe_autonomy.yield_policy import reservation_setup_pending
    assert reservation_setup_pending({'proactive':True},None,{})
    recovery={'proactive':True,'loser':'robot_02'}
    assert reservation_setup_pending(None,recovery,{'robot_02':{'state':'GOING_DIRECT'}})
    assert not reservation_setup_pending(None,recovery,{'robot_02':{'state':'YIELDING'}})
    assert not reservation_setup_pending(None,None,{})

def test_path_queries_wait_for_both_current_goal_cancellation_acknowledgements():
    from cafe_autonomy.yield_policy import preentry_pause_acknowledged
    ids={'robot_01':'one','robot_02':'two'}
    states={r:dict(request_id=rid,state='GOING_DIRECT',nav_paused=True,goal_execution_verified_idle=True) for r,rid in ids.items()}
    assert preentry_pause_acknowledged(states,ids)
    states['robot_02']['nav_paused']=False
    assert not preentry_pause_acknowledged(states,ids)
    states['robot_02']['nav_paused']=True;states['robot_02']['request_id']='old'
    assert not preentry_pause_acknowledged(states,ids)
