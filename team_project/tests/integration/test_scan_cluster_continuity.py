"""Removed static beams must not create a synthetic merged foreground body."""
import numpy as np
from cafe_autonomy.dynamic_obstacles import foreground_clusters


def test_static_gap_separates_spatially_close_foreground_returns():
    points = np.array([[0.,0.], [.01,0.], [.02,0.], [.03,0.], [.04,0.], [.05,0.]])
    groups = foreground_clusters(points, [1,2,3,8,9,10], .25, 3)
    assert [len(group) for group in groups] == [3,3]


def test_contiguous_visible_arc_remains_one_object():
    points = np.array([[0.,0.], [.01,0.], [.02,0.], [.03,0.]])
    assert len(foreground_clusters(points, [1,2,3,4], .25, 3)) == 1


def test_real_distance_gap_and_minimum_support_are_preserved():
    points = np.array([[0.,0.], [.01,0.], [1.,0.], [1.01,0.], [1.02,0.]])
    groups = foreground_clusters(points, [1,2,3,4,5], .25, 3)
    assert len(groups) == 1
    assert np.allclose(groups[0], points[2:])
