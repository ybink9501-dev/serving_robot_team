from cafe_autonomy.rgbd_projection import lidar_corroborates_person

def test_lower_confidence_side_view_needs_supported_matching_body():
    assert lidar_corroborates_person((1.1,0.),(1.2,0.),.22,True,20,.5)

def test_walls_and_unsupported_or_unmatched_returns_do_not_promote():
    assert not lidar_corroborates_person((1.1,0.),(1.2,0.),.22,False,20,.5)
    assert not lidar_corroborates_person((1.1,0.),(1.2,0.),.22,True,3,.5)
    assert not lidar_corroborates_person((1.1,0.),(2.,0.),.22,True,20,.5)
    assert not lidar_corroborates_person((1.1,0.),(1.2,0.),.8,True,20,.5)


def test_observed_leg_arc_is_valid_corroboration_for_side_view():
    assert lidar_corroborates_person((.9266,-.0619),(1.0927,-.0228),.09158,True,17,.5)
    assert not lidar_corroborates_person((.9266,-.0619),(1.0927,-.0228),.03,True,17,.5)
