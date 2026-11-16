"""Settled arrival may complete during a fresh non-contact safety hold."""
from types import SimpleNamespace
from rclpy.time import Time
from cafe_autonomy.mission import Mission


def run(*, fresh=True, contact=False, pose=(.02,0.,0.), inactive=True, margin=0.):
    commands=[]; arrivals=[]
    m=SimpleNamespace(
        hold=True, localization_margin=margin, fine_since=0., fine_timeout=120., target=(0.,0.,0.),
        robot_pose=lambda:pose, fine_control_xy_target=.04, xy_tolerance=.1,
        fine_aligning=False, arrival_settle_since=9., goal_confirmed_inactive=inactive,
        dynamic_report={'sensor_fresh':fresh,'contact_latched':contact},
        limits={'arrival':{'control_yaw_target_rad':.18,'settle_linear_mps':.015,
                           'settle_angular_rps':.05,'settle_s':.5}},
        fine_pub=SimpleNamespace(publish=commands.append),
        odom=SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.),angular=SimpleNamespace(z=0.)))),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.)),
        arrived=arrivals.append,
    )
    Mission.fine_step(m)
    assert commands and all(cmd.linear.x==0. and cmd.angular.z==0. for cmd in commands)
    return arrivals


def test_fresh_noncontact_hold_can_confirm_stationary_arrival():
    assert run()


def test_stale_contact_or_unreconciled_goal_cannot_confirm_arrival():
    assert not run(fresh=False)
    assert not run(contact=True)
    assert not run(inactive=False)


def test_hold_outside_goal_never_commands_motion_or_finishes():
    assert not run(pose=(.4,0.,0.))


def test_position_inside_ten_cm_does_not_chase_four_cm():
    assert run(pose=(.09,0.,0.))


def test_localization_budget_does_not_mistake_estimated_nine_cm_for_physical_acceptance():
    assert not run(pose=(.09,0.,0.),margin=.03)
    assert run(pose=(.06,0.,0.),margin=.03)
