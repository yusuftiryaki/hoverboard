"""Kinematic world tests — no ROS, no hardware.

The world is the answer key everything else gets graded against, so it has to be
right on its own terms. A wrong world does not fail loudly; it silently makes a
broken EKF look good, or a correct one look broken.
"""

import math

import pytest

from robot_sim.world import KinematicWorld

# Wheel RPM for 1 m/s at r=0.0825: 1 / (2*pi*0.0825) * 60
RPM_FOR_1MS = 60.0 / (2.0 * math.pi * 0.0825)


def settled(world, target_l, target_r, seconds=2.0, dt=0.02):
    """Run until the wheel lag has converged, discarding the spin-up transient."""
    for _ in range(int(seconds / dt)):
        world.step(target_l, target_r, dt)


def test_stationary_robot_stays_put():
    w = KinematicWorld()
    settled(w, 0.0, 0.0)
    assert w.pose.x == 0.0 and w.pose.y == 0.0 and w.pose.yaw == 0.0
    assert w.v == 0.0 and w.omega == 0.0


def test_wheels_converge_on_the_commanded_units():
    w = KinematicWorld(tau=0.09)
    settled(w, 100.0, 100.0)
    meas_l, meas_r = w.step(100.0, 100.0, 0.02)
    assert meas_l == pytest.approx(100.0, rel=1e-3)
    assert meas_r == pytest.approx(100.0, rel=1e-3)


def test_straight_line_speed_and_distance():
    w = KinematicWorld(wheel_radius=0.0825, board_units_per_rpm=1.0)
    settled(w, RPM_FOR_1MS, RPM_FOR_1MS, seconds=3.0)
    assert w.v == pytest.approx(1.0, rel=1e-3)
    assert w.omega == pytest.approx(0.0, abs=1e-9)

    start = w.pose.x
    for _ in range(100):          # 100 * 0.02 s = 2 s at 1 m/s
        w.step(RPM_FOR_1MS, RPM_FOR_1MS, 0.02)
    assert w.pose.x - start == pytest.approx(2.0, rel=1e-3)
    assert w.pose.y == pytest.approx(0.0, abs=1e-9)
    assert w.pose.yaw == pytest.approx(0.0, abs=1e-9)


def test_spin_in_place():
    # Equal and opposite wheels: pure rotation, no translation.
    w = KinematicWorld(wheel_separation=0.5)
    settled(w, -RPM_FOR_1MS, RPM_FOR_1MS, seconds=3.0)
    # v_r - v_l = 2 m/s over a 0.5 m track -> 4 rad/s
    assert w.omega == pytest.approx(4.0, rel=1e-3)
    assert w.v == pytest.approx(0.0, abs=1e-9)
    x0, y0 = w.pose.x, w.pose.y
    for _ in range(50):
        w.step(-RPM_FOR_1MS, RPM_FOR_1MS, 0.02)
    assert w.pose.x == pytest.approx(x0, abs=1e-9)
    assert w.pose.y == pytest.approx(y0, abs=1e-9)


def test_a_full_circle_returns_to_the_start():
    # The strongest check on the arc integration: drive a constant curve for
    # exactly one revolution and land back where you began. Euler integration
    # would drift outward and fail this.
    w = KinematicWorld(wheel_separation=0.5, tau=1e-9)  # no lag: exact kinematics
    v, omega = 0.5, 0.5                                  # 1 m radius circle
    half = omega * w.wheel_separation / 2.0
    rpm = lambda speed: speed / (2.0 * math.pi * w.wheel_radius) * 60.0
    target_l, target_r = rpm(v - half), rpm(v + half)

    dt = 0.001
    steps = int(round(2.0 * math.pi / omega / dt))       # one full revolution
    for _ in range(steps):
        w.step(target_l, target_r, dt)

    assert math.hypot(w.pose.x, w.pose.y) < 0.01         # back to the origin
    assert abs(math.atan2(math.sin(w.pose.yaw), math.cos(w.pose.yaw))) < 0.01


def test_quarter_turn_lands_where_geometry_says():
    w = KinematicWorld(wheel_separation=0.5, tau=1e-9)
    v, omega = 0.5, 0.5                                  # radius = 1 m, CCW
    half = omega * w.wheel_separation / 2.0
    rpm = lambda speed: speed / (2.0 * math.pi * w.wheel_radius) * 60.0

    dt = 0.001
    for _ in range(int(round((math.pi / 2) / omega / dt))):
        w.step(rpm(v - half), rpm(v + half), dt)

    # Starting at the origin facing east, turning left on a 1 m radius: the
    # centre is at (0, 1), so a quarter turn lands at (1, 1) facing north.
    assert w.pose.x == pytest.approx(1.0, abs=0.01)
    assert w.pose.y == pytest.approx(1.0, abs=0.01)
    assert w.pose.yaw == pytest.approx(math.pi / 2, abs=0.01)


def test_reverse_goes_backwards():
    w = KinematicWorld()
    settled(w, -RPM_FOR_1MS, -RPM_FOR_1MS, seconds=3.0)
    assert w.v == pytest.approx(-1.0, rel=1e-3)
    x0 = w.pose.x
    for _ in range(50):
        w.step(-RPM_FOR_1MS, -RPM_FOR_1MS, 0.02)
    assert w.pose.x < x0


def test_board_units_per_rpm_scales_the_world_not_the_belief():
    # The knob that lets the sim be miscalibrated relative to the Pi: at 2 units
    # per RPM the same command turns the wheels half as fast.
    slow = KinematicWorld(board_units_per_rpm=2.0)
    fast = KinematicWorld(board_units_per_rpm=1.0)
    settled(slow, RPM_FOR_1MS, RPM_FOR_1MS, seconds=3.0)
    settled(fast, RPM_FOR_1MS, RPM_FOR_1MS, seconds=3.0)
    assert slow.v == pytest.approx(fast.v / 2.0, rel=1e-3)


def test_acceleration_is_reported_for_the_fake_imu():
    w = KinematicWorld(tau=0.09)
    w.step(RPM_FOR_1MS, RPM_FOR_1MS, 0.02)
    assert w.accel_x > 0.0          # spinning up
    settled(w, RPM_FOR_1MS, RPM_FOR_1MS, seconds=3.0)
    w.step(RPM_FOR_1MS, RPM_FOR_1MS, 0.02)
    assert w.accel_x == pytest.approx(0.0, abs=1e-3)   # steady state


def test_zero_dt_is_a_no_op():
    w = KinematicWorld()
    settled(w, 50.0, 50.0)
    pose_before = (w.pose.x, w.pose.y, w.pose.yaw)
    w.step(50.0, 50.0, 0.0)
    assert (w.pose.x, w.pose.y, w.pose.yaw) == pose_before
