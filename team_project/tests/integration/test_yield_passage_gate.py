from cafe_autonomy.yield_policy import passed_blocking_position

def test_observed_outside_boundary_before_entry_is_not_completed_passage():
    start=(5.659595,1.998401);blocker=(2.432738,1.992588)
    assert not passed_blocking_position(start,start,blocker,.9)
    assert not passed_blocking_position((5.66,2.),start,blocker,.9)
    assert not passed_blocking_position((2.35,2.),start,blocker,.9)
    assert passed_blocking_position((1.50,2.),start,blocker,.9)

def test_sideways_or_away_motion_and_missing_pose_cannot_release_waiter():
    a=(5.6,2.);b=(2.4,2.)
    assert not passed_blocking_position((5.6,0.),a,b,.9)
    assert not passed_blocking_position((7.,2.),a,b,.9)
    assert not passed_blocking_position(None,a,b,.9)
    assert not passed_blocking_position((1.,2.),a,a,.9)
