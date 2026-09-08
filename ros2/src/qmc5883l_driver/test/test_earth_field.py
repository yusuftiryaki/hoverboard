"""The one definition of the simulated earth field. No ROS, no hardware.

This model had a mirrored sign for weeks (A6) and nothing caught it, because it
existed in two files that were wrong the same way and a test helper that was
wrong a third time. So the tests here refuse to use any formula at all: every
expected value is a compass bearing worked out by hand, and the sweep goes all
the way round rather than sampling the one heading where the bug is invisible.
"""

import math

import pytest

from qmc5883l_driver.earth_field import (
    EARTH_DOWN_T,
    EARTH_NORTH_T,
    field_in_body_frame,
)
from qmc5883l_driver.fake_bus import FakeQMC5883LBus

N = EARTH_NORTH_T


def test_the_four_cardinal_headings_are_hand_computed():
    """REP-103: yaw 0 is EAST, x is forward, y is left, z is up.

    Reasoned as a compass, not as algebra. The field points north; where it
    lands in the robot's own axes depends only on which way the robot faces:
      facing east  -> north is to the robot's LEFT      -> +y
      facing north -> north is straight AHEAD           -> +x
      facing west  -> north is to the robot's RIGHT     -> -y
      facing south -> north is straight BEHIND          -> -x
    z is -down in every case, because down is -z.
    """
    east = field_in_body_frame(0.0)
    assert east == pytest.approx((0.0, N, -EARTH_DOWN_T), abs=1e-12)

    north = field_in_body_frame(math.radians(90.0))
    assert north == pytest.approx((N, 0.0, -EARTH_DOWN_T), abs=1e-12)

    west = field_in_body_frame(math.radians(180.0))
    assert west == pytest.approx((0.0, -N, -EARTH_DOWN_T), abs=1e-12)

    south = field_in_body_frame(math.radians(-90.0))
    assert south == pytest.approx((-N, 0.0, -EARTH_DOWN_T), abs=1e-12)


def test_a_mirrored_field_would_fail_somewhere_other_than_east():
    """Why the test above cannot be one assertion.

    At yaw 0 the mirrored model and the correct one are IDENTICAL: the flipped
    term is N*sin(yaw) and sin(0) is 0. Every sim test used to start there. So
    pin the fact that the two models differ as soon as the robot turns at all —
    if this ever stops being true, the sweep below has stopped protecting
    anything.
    """
    mirrored = lambda yaw: (-N * math.sin(yaw), N * math.cos(yaw), -EARTH_DOWN_T)

    assert field_in_body_frame(0.0) == pytest.approx(mirrored(0.0), abs=1e-12)
    for degrees in (30.0, 90.0, 150.0, 210.0, 270.0, 330.0):
        yaw = math.radians(degrees)
        assert field_in_body_frame(yaw) != pytest.approx(mirrored(yaw), abs=1e-9)


def test_heading_is_recoverable_through_a_full_turn():
    """Sweep the whole circle: the field must give the yaw back.

    atan2(x, y) inverts (N sin, N cos). This is the one place a formula is
    allowed, because it is the INVERSE being checked against the input yaw
    rather than a second copy of the same forward model.
    """
    for degrees in range(-180, 180, 15):
        yaw = math.radians(degrees)
        bx, by, _ = field_in_body_frame(yaw)
        assert math.degrees(math.atan2(bx, by)) == pytest.approx(degrees, abs=1e-9)


def test_horizontal_magnitude_is_constant_all_the_way_round():
    """Turning cannot change how strong the earth is."""
    for degrees in range(0, 360, 10):
        bx, by, bz = field_in_body_frame(math.radians(degrees))
        assert math.hypot(bx, by) == pytest.approx(N, rel=1e-12)
        assert bz == pytest.approx(-EARTH_DOWN_T, abs=1e-12)


def test_the_fake_chip_reports_this_field_and_does_not_model_its_own():
    """The A6 regression, stated directly.

    fake_bus used to carry its own copy of the rotation. Strip the hard iron
    and the noise away and what is left must be exactly the shared model —
    if this fails, a second model of the earth has come back.
    """
    bus = FakeQMC5883LBus(hard_iron_t=(0.0, 0.0, 0.0), noise_t=0.0)
    for degrees in range(0, 360, 30):
        bus.yaw = math.radians(degrees)
        assert bus.true_field() == pytest.approx(
            field_in_body_frame(bus.yaw), abs=1e-12)
