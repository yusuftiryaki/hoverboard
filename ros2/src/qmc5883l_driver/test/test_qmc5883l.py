"""QMC5883L tests — no I2C, no ROS.

These pin the quiet failures: byte order (this chip is little-endian while the
MPU6050 on the same bus is big-endian), the range bits actually written, and that
hard-iron calibration recovers a heading the raw reading gets wrong. None of
those crash anything — they just produce a confident, wrong heading, which is the
one thing this sensor exists to prevent.
"""

import math

import pytest

from qmc5883l_driver.fake_bus import EARTH_NORTH_T, FakeQMC5883LBus
from qmc5883l_driver.qmc5883l import (
    CHIP_ID_VALUE,
    REG_CONTROL_1,
    REG_SET_RESET,
    QMC5883L,
    _to_signed16,
)


def make(**kwargs):
    bus = FakeQMC5883LBus(noise_t=0.0, **kwargs)
    mag = QMC5883L(bus, range_gauss=2)
    mag.reset()
    mag.configure()
    return bus, mag


def heading_of(field):
    """Level-robot compass heading, REP-103: 0 = east, CCW positive.

    Projecting the world's north-pointing field into the body frame gives
    x = B sin(yaw), y = B cos(yaw) — so the yaw is that vector's angle measured
    back from +y, which is exactly atan2(x, y).

    ⚠️ This deliberately does NOT mirror fake_bus's own maths. It used to read
    atan2(-x, y), the exact inverse of the sign error fake_bus had, so the two
    cancelled and every heading test passed against a mirrored world. A helper
    that undoes the fixture's bug tests only that they agree with each other.
    test_field_matches_hand_computed_physics below is what anchors both to
    reality; keep it independent of this function.
    """
    return math.atan2(field[0], field[1])


def test_field_matches_hand_computed_physics():
    """The convention, pinned to numbers derived by hand rather than by code.

    This is the test that was missing, and its absence let a mirrored
    magnetometer through: sim_node and fake_bus both had x = -B sin(yaw), and
    the test helper inverted it right back. Everything agreed; all of it was
    the mirror image of the earth.

    Facing north, a north-pointing field lies straight along the robot's
    FORWARD axis. If x comes out negative here, the fixture is claiming the
    field is behind a robot that is driving at it.
    """
    # No hard iron: this test is about the earth's field and the body frame,
    # nothing else. The default fixture carries an offset on purpose.
    bus, mag = make(hard_iron_t=(0.0, 0.0, 0.0))

    # One LSB at the 2 G range (12000 LSB/Gauss). The reading is quantised by a
    # 12-bit ADC, so demanding more precision than this tests the arithmetic of
    # the test rather than the convention it is here to pin.
    lsb = 1e-4 / 12000.0

    # Facing east (yaw 0): forward is east, left is north -> all of it on +y.
    bus.set_heading_deg(0.0)
    x, y, _ = mag.read().field
    assert x == pytest.approx(0.0, abs=lsb)
    assert y == pytest.approx(EARTH_NORTH_T, abs=lsb)

    # Facing north (yaw +90): forward IS north -> all of it on +x, and POSITIVE.
    bus.set_heading_deg(90.0)
    x, y, _ = mag.read().field
    assert x == pytest.approx(EARTH_NORTH_T, abs=lsb)
    assert y == pytest.approx(0.0, abs=lsb)

    # Facing west (yaw 180): north is now to the robot's RIGHT -> -y.
    bus.set_heading_deg(180.0)
    x, y, _ = mag.read().field
    assert x == pytest.approx(0.0, abs=lsb)
    assert y == pytest.approx(-EARTH_NORTH_T, abs=lsb)

    # Facing north-east (yaw +45): the field splits evenly, both components +.
    bus.set_heading_deg(45.0)
    x, y, _ = mag.read().field
    half = EARTH_NORTH_T * math.sqrt(0.5)
    assert x == pytest.approx(half, abs=lsb)
    assert y == pytest.approx(half, abs=lsb)


def test_chip_id():
    _, mag = make()
    assert mag.chip_id() == CHIP_ID_VALUE


def test_little_endian_decoding():
    # The opposite of the MPU6050 on the same bus. Swapping these does not fail,
    # it just rotates every heading.
    assert _to_signed16(0x01, 0x00) == 1
    assert _to_signed16(0x00, 0x01) == 256
    assert _to_signed16(0xFF, 0xFF) == -1
    assert _to_signed16(0x00, 0x80) == -32768


def test_reset_leaves_standby_and_configure_wakes_it():
    bus = FakeQMC5883LBus()
    mag = QMC5883L(bus)
    mag.reset()
    assert bus.standby
    assert mag.read() is None          # standby -> no data ready
    mag.configure()
    assert not bus.standby
    assert mag.read() is not None


def test_configure_writes_the_range_bits_and_the_set_reset_period():
    bus = FakeQMC5883LBus()
    QMC5883L(bus, range_gauss=8, odr_hz=200, osr=64).configure()
    control_1 = bus.read_byte_data(0x0D, REG_CONTROL_1)
    assert (control_1 >> 4) & 0b11 == 0b01     # 8 G
    assert (control_1 >> 2) & 0b11 == 0b11     # 200 Hz
    assert (control_1 >> 6) & 0b11 == 0b11     # OSR 64
    assert control_1 & 0b11 == 0b01            # continuous
    # Datasheet-mandated and easy to forget; the readings wander without it.
    assert bus.read_byte_data(0x0D, REG_SET_RESET) == 0x01


def test_field_magnitude_is_earthlike():
    bus, mag = make(hard_iron_t=(0.0, 0.0, 0.0))
    field = mag.read().field
    total = math.sqrt(sum(v * v for v in field))
    # The earth's total field is 25-65 uT everywhere.
    assert 25e-6 < total < 65e-6


def angle_error_deg(measured_rad, expected_rad):
    """Shortest angle between two headings. +180 and -180 are the same heading."""
    diff = measured_rad - expected_rad
    return abs(math.degrees(math.atan2(math.sin(diff), math.cos(diff))))


@pytest.mark.parametrize("heading_deg", [0, 45, 90, 180, -90, 270])
def test_uncalibrated_reading_gives_the_heading_when_there_is_no_hard_iron(heading_deg):
    bus, mag = make(hard_iron_t=(0.0, 0.0, 0.0))
    bus.set_heading_deg(heading_deg)
    measured = heading_of(mag.read().field)
    assert angle_error_deg(measured, math.radians(heading_deg)) < 0.5


def test_hard_iron_wrecks_the_heading_and_calibration_recovers_it():
    """The reason calibration is not optional, in one test.

    A hard-iron offset is a constant field the robot carries, so it does NOT
    average out as the robot turns — it biases the heading by an amount that
    itself changes with heading. Subtracting the offset is what fixes it.
    """
    offset = (8e-6, -5e-6, 3e-6)   # ~1/3 of the earth's horizontal field
    bus, mag = make(hard_iron_t=offset)

    worst_raw = 0.0
    worst_calibrated = 0.0
    for heading_deg in range(0, 360, 15):
        bus.set_heading_deg(heading_deg)
        raw = mag.read().field
        expected = math.radians(heading_deg)

        def error(field):
            diff = heading_of(field) - expected
            return abs(math.atan2(math.sin(diff), math.cos(diff)))

        worst_raw = max(worst_raw, error(raw))
        corrected = tuple(raw[i] - offset[i] for i in range(3))
        worst_calibrated = max(worst_calibrated, error(corrected))

    # Uncalibrated: tens of degrees out. This is what "confidently wrong" means.
    assert math.degrees(worst_raw) > 15.0
    # Calibrated: essentially exact.
    assert math.degrees(worst_calibrated) < 0.5


def test_overflow_is_reported_not_swallowed():
    # A field far beyond the 2 G range: the chip flags OVL and the sample is junk.
    bus, mag = make(hard_iron_t=(1.0, 0.0, 0.0))   # 1 tesla of "steel"
    sample = mag.read()
    assert sample.overflow is True


def test_no_overflow_in_a_normal_field():
    bus, mag = make(hard_iron_t=(0.0, 0.0, 0.0))
    assert mag.read().overflow is False


def test_rejects_settings_it_cannot_scale():
    bus = FakeQMC5883LBus()
    with pytest.raises(ValueError):
        QMC5883L(bus, range_gauss=4)
    with pytest.raises(ValueError):
        QMC5883L(bus, odr_hz=25)
    with pytest.raises(ValueError):
        QMC5883L(bus, osr=1024)


def test_wrong_address_raises_like_a_real_bus():
    bus = FakeQMC5883LBus(address=0x0D)
    mag = QMC5883L(bus, address=0x1E)   # HMC5883L's address
    with pytest.raises(OSError):
        mag.chip_id()


def test_earth_field_constant_is_sane():
    assert 20e-6 < EARTH_NORTH_T < 40e-6
