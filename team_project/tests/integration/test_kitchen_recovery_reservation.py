"""Pickup recovery must retain the Fleet kitchen reservation."""
from types import SimpleNamespace

from cafe_autonomy.fleet import Fleet


def busy(state, target='kitchen', mode='KITCHEN_PICKUP', assignment='RUNNING'):
    task = SimpleNamespace(mode=mode, state=assignment)
    fleet = SimpleNamespace(
        core=SimpleNamespace(active={'robot_01': 'order'}, tasks={'order': task}),
        status={'robot_01': {'state': state, 'target': target}},
    )
    return Fleet.kitchen_busy(fleet)


def test_pickup_recovery_keeps_reservation():
    assert busy('RECOVERY_WAIT')
    assert busy('RECOVERY_WAIT', mode='GO_KITCHEN')


def test_leaving_kitchen_and_other_recovery_do_not_keep_reservation():
    assert not busy('GOING_TABLE', target='table_03')
    assert not busy('RECOVERY_WAIT', target='table_03')
    assert not busy('RECOVERY_WAIT', mode='DELIVER_DIRECT')


def test_new_assignment_reserves_before_first_status():
    assert busy('COMPLETED', target='table_03', assignment='ASSIGNED')
