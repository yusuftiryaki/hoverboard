"""command_limits: pinned to hand-computed numbers, not to its own formulas.

ROS-free on purpose (decision 10: a fixture tested with its own maths only proves
it agrees with itself — so the expected values here are worked out by hand).

Geometry used throughout: 165 mm wheel (r = 0.0825), 0.5 m track, 1 raw unit per
RPM. One metre per second at the wheel is 60 / (2 pi 0.0825) = 115.749 RPM.
"""

import math

import pytest

from hoverboard_bridge.command_limits import (
    DriveGeometry,
    limit_twist,
    wheel_commands,
)

GEO = DriveGeometry(wheel_radius=0.0825, wheel_separation=0.5,
                    cmd_per_rpm=1.0, steer_sign=1.0)
RPM_PER_MPS = 115.749


@pytest.mark.parametrize("v, w", [
    (math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0), (0.0, -math.inf),
    (math.nan, math.inf),
])
def test_a_non_finite_command_means_stop(v, w):
    assert limit_twist(v, w, 1.0, 1.5) == (0.0, 0.0)
    assert wheel_commands(v, w, GEO, 300) == (0, 0)


def test_within_limits_nothing_changes():
    assert limit_twist(0.5, -0.6, 1.0, 1.5) == (0.5, -0.6)


def test_linear_limit_scales_both_axes_and_keeps_the_arc():
    # 2 m/s at 1 rad/s is a 2 m radius. Halving both keeps the 2 m radius;
    # clipping v alone to 1 m/s would have tightened it to 1 m.
    assert limit_twist(2.0, 1.0, 1.0, 1.5) == (1.0, 0.5)


def test_angular_limit_scales_both_axes_too():
    v, w = limit_twist(0.5, 3.0, 1.0, 1.5)
    assert (v, w) == pytest.approx((0.25, 1.5))


def test_straight_line_by_hand():
    # 1 m/s -> 115.749 RPM on both wheels -> speed 116, no steer.
    assert wheel_commands(1.0, 0.0, GEO, 300) == (116, 0)


def test_turn_on_the_spot_by_hand():
    # 1 rad/s, half-track 0.25 m -> wheels at -0.25 / +0.25 m/s -> -/+28.94 RPM.
    # speed = (l + r) / 2 = 0, steer = (l - r) / 2 = -28.94 -> -29.
    assert wheel_commands(0.0, 1.0, GEO, 300) == (0, -29)


def test_steer_sign_flips_only_the_steer():
    flipped = DriveGeometry(0.0825, 0.5, 1.0, -1.0)
    assert wheel_commands(0.0, 1.0, flipped, 300) == (0, 29)


def test_raw_limit_is_per_wheel_and_keeps_the_arc():
    # 5 m/s at 2 rad/s: left 4.5 m/s = 520.9, right 5.5 m/s = 636.6 raw.
    # Scaled so the faster wheel is exactly 300: left 245.5, right 300.
    # speed = (245.45 + 300) / 2 = 272.7 -> 273, steer = (245.45 - 300) / 2 -> -27.
    speed, steer = wheel_commands(5.0, 2.0, GEO, 300)
    assert (speed, steer) == (273, -27)
    left, right = speed + steer, speed - steer          # the board's mixer
    assert right == 300
    # right/left was 5.5/4.5 before limiting and still is, to rounding.
    assert right / left == pytest.approx(5.5 / 4.5, rel=0.01)


def test_the_firmware_clamp_never_has_to_act():
    # Whatever is asked, speed and steer each stay within the limit, so the
    # firmware's separate constrain() on each never distorts the arc.
    for v in (-9.0, -1.0, 0.0, 0.7, 9.0):
        for w in (-9.0, -0.4, 0.0, 2.0, 9.0):
            speed, steer = wheel_commands(v, w, GEO, 300)
            assert abs(speed) <= 300 and abs(steer) <= 300, (v, w, speed, steer)


def test_in_limit_commands_match_the_old_bridge_exactly():
    """Calibration must not move. This is the bridge's previous inline
    kinematics, copied verbatim. A grid, plus the inputs that sit next to a .5
    rounding tie — where reassociated float maths (an earlier version of this
    module) flipped the result by one count."""
    def old(v, w, g):
        half_track = 0.5 * g.wheel_separation
        rpm_l = (v - w * half_track) / (2.0 * math.pi * g.wheel_radius) * 60.0
        rpm_r = (v + w * half_track) / (2.0 * math.pi * g.wheel_radius) * 60.0
        cmd_l, cmd_r = rpm_l * g.cmd_per_rpm, rpm_r * g.cmd_per_rpm
        return (int(round((cmd_l + cmd_r) * 0.5)),
                int(round((cmd_l - cmd_r) * 0.5 * g.steer_sign)))

    for v in (-1.0, -0.4, 0.0, 0.25, 0.5, 1.0):
        for w in (-1.5, -0.6, 0.0, 0.3, 1.5):
            assert wheel_commands(v, w, GEO, 300) == old(v, w, GEO), (v, w)

    # Every speed whose straight-line command lands within a few ulp of n + .5.
    for n in range(-116, 116):
        tie = (n + 0.5) / RPM_PER_MPS_EXACT
        for step in range(-4, 5):
            v = tie + step * math.ulp(tie)
            assert wheel_commands(v, 0.0, GEO, 300) == old(v, 0.0, GEO), v


RPM_PER_MPS_EXACT = 60.0 / (2.0 * math.pi * 0.0825)


@pytest.mark.parametrize("max_v, max_w", [
    (-1.0, 1.5), (1.0, -1.5), (0.0, 1.5), (1.0, 0.0),
    (math.nan, 1.5), (1.0, math.inf),
])
def test_an_unusable_twist_limit_means_stop_not_reverse(max_v, max_w):
    # A negative limit used to give a negative scale: 0.5 m/s forward became
    # 1 m/s REVERSE, and a zero command divided by zero inside the bridge.
    assert limit_twist(0.5, 0.2, max_v, max_w) == (0.0, 0.0)
    assert limit_twist(0.0, 0.0, max_v, max_w) == (0.0, 0.0)


@pytest.mark.parametrize("raw_limit", [-300, 0, math.nan, -math.inf])
def test_an_unusable_raw_limit_means_stop(raw_limit):
    assert wheel_commands(0.5, 0.0, GEO, raw_limit) == (0, 0)
    assert wheel_commands(0.0, 0.0, GEO, raw_limit) == (0, 0)


@pytest.mark.parametrize("v, w", [(1e308, 0.0), (0.0, 1e308), (1e308, -1e308),
                                  (-2e306, 5.0)])
def test_finite_input_that_overflows_halfway_is_a_stop_not_a_crash(v, w):
    # 1e308 * 115.7 is inf; inf * (300 / inf) is NaN; int(round(NaN)) raises.
    assert wheel_commands(v, w, GEO, 300) == (0, 0)


def test_steer_sign_minus_one_still_keeps_both_wheels_under_the_limit():
    flipped = DriveGeometry(0.0825, 0.5, 1.0, -1.0)
    speed, steer = wheel_commands(5.0, 2.0, flipped, 300)
    assert (speed, steer) == (273, 27)
    # with steer_sign -1 the board's left wheel is speed - steer
    assert max(abs(speed + steer), abs(speed - steer)) == 300
