"""MPU6050 driver tests against the simulated bus — no I2C, no ROS.

These pin the things that are silently wrong rather than loudly broken: byte
order, scale factors, and the range bits actually written to the chip. A wrong
gyro scale does not crash anything — it just makes every turn 2x too big, which
you discover as a robot that will not drive straight.
"""

import math

import pytest

from mpu6050_driver.fake_bus import FakeMPU6050Bus
from mpu6050_driver.mpu6050 import (
    REG_ACCEL_CONFIG,
    REG_GYRO_CONFIG,
    REG_SMPLRT_DIV,
    STANDARD_GRAVITY,
    WHO_AM_I_VALUE,
    MPU6050,
    _to_signed16,
)


def make(**kwargs):
    # Noise off by default: these tests check the maths, not the statistics.
    bus = FakeMPU6050Bus(noise_gyro_dps=0.0, noise_accel_g=0.0, **kwargs)
    imu = MPU6050(bus, accel_range_g=4, gyro_range_dps=250, sample_rate_hz=100.0)
    imu.reset()
    imu.configure()
    return bus, imu


def test_who_am_i():
    bus, imu = make()
    assert imu.who_am_i() == WHO_AM_I_VALUE


def test_reset_leaves_the_device_asleep_and_configure_wakes_it():
    bus = FakeMPU6050Bus()
    imu = MPU6050(bus)
    imu.reset()
    assert bus.asleep
    # Reading a sleeping device must fail rather than return zeros.
    with pytest.raises(OSError):
        imu.read()
    imu.configure()
    assert not bus.asleep
    imu.read()  # no raise


def test_big_endian_decoding():
    # The chip is big-endian; getting this backwards yields plausible garbage.
    assert _to_signed16(0x00, 0x01) == 1
    assert _to_signed16(0x01, 0x00) == 256
    assert _to_signed16(0xFF, 0xFF) == -1
    assert _to_signed16(0x80, 0x00) == -32768
    assert _to_signed16(0x7F, 0xFF) == 32767


def test_configure_writes_the_expected_range_bits():
    bus = FakeMPU6050Bus()
    MPU6050(bus, accel_range_g=8, gyro_range_dps=1000, sample_rate_hz=100.0).configure()
    assert (bus.read_byte_data(0x68, REG_ACCEL_CONFIG) >> 3) & 0x03 == 2   # AFS_SEL for 8 g
    assert (bus.read_byte_data(0x68, REG_GYRO_CONFIG) >> 3) & 0x03 == 2    # FS_SEL for 1000 dps
    # 1 kHz / (9 + 1) = 100 Hz
    assert bus.read_byte_data(0x68, REG_SMPLRT_DIV) == 9


@pytest.mark.parametrize("rate,divider,actual", [(100.0, 9, 100.0), (50.0, 19, 50.0),
                                                 (1000.0, 0, 1000.0), (30.0, 32, 1000.0 / 33)])
def test_sample_rate_divider(rate, divider, actual):
    imu = MPU6050(FakeMPU6050Bus(), sample_rate_hz=rate)
    assert imu.sample_rate_divider() == divider
    assert imu.actual_sample_rate_hz() == pytest.approx(actual)


def test_stationary_level_robot_reads_one_g_up_and_no_rotation():
    bus, imu = make(gyro_bias_dps=(0.0, 0.0, 0.0))
    sample = imu.read()
    assert sample.accel[0] == pytest.approx(0.0, abs=0.01)
    assert sample.accel[1] == pytest.approx(0.0, abs=0.01)
    # An accelerometer at rest measures the reaction to gravity: +1 g up, not 0.
    assert sample.accel[2] == pytest.approx(STANDARD_GRAVITY, rel=1e-3)
    assert sample.gyro[2] == pytest.approx(0.0, abs=1e-3)


def test_gyro_scale_is_right_across_ranges():
    for dps_range in (250, 500, 1000, 2000):
        bus = FakeMPU6050Bus(gyro_bias_dps=(0.0, 0.0, 0.0),
                             noise_gyro_dps=0.0, noise_accel_g=0.0)
        imu = MPU6050(bus, gyro_range_dps=dps_range)
        imu.configure()
        bus.set_yaw_rate(1.0)  # 1 rad/s
        assert imu.read().gyro[2] == pytest.approx(1.0, rel=2e-3), f"range {dps_range}"


def test_accel_scale_is_right_across_ranges():
    for g_range in (2, 4, 8, 16):
        bus = FakeMPU6050Bus(noise_gyro_dps=0.0, noise_accel_g=0.0)
        imu = MPU6050(bus, accel_range_g=g_range)
        imu.configure()
        bus.set_forward_accel(1.5)  # 1.5 m/s^2 forward, plus 1 g up
        sample = imu.read()
        assert sample.accel[0] == pytest.approx(1.5, rel=5e-3), f"range {g_range}"
        assert sample.accel[2] == pytest.approx(STANDARD_GRAVITY, rel=5e-3)


def test_temperature_conversion():
    bus, imu = make()
    bus.true_temp_c = 41.25
    assert imu.read().temperature == pytest.approx(41.25, abs=0.01)


def test_gyro_bias_is_present_and_measurable():
    # The node's startup calibration exists to remove this; if the fake had no
    # bias, a broken calibration would still pass.
    bus = FakeMPU6050Bus(gyro_bias_dps=(1.7, -0.9, 2.4), noise_gyro_dps=0.0,
                         noise_accel_g=0.0)
    imu = MPU6050(bus)
    imu.configure()
    sample = imu.read()
    assert sample.gyro[2] == pytest.approx(2.4 * math.pi / 180.0, rel=1e-2)


def test_averaging_recovers_the_bias_through_noise():
    # This is exactly what Mpu6050Node._calibrate does.
    bus = FakeMPU6050Bus(gyro_bias_dps=(1.7, -0.9, 2.4), noise_gyro_dps=0.5, seed=7)
    imu = MPU6050(bus)
    imu.configure()
    samples = [imu.read().gyro[2] for _ in range(500)]
    estimated_dps = (sum(samples) / len(samples)) * 180.0 / math.pi
    assert estimated_dps == pytest.approx(2.4, abs=0.05)


def test_rejects_ranges_and_filters_it_cannot_scale():
    bus = FakeMPU6050Bus()
    with pytest.raises(ValueError):
        MPU6050(bus, accel_range_g=3)
    with pytest.raises(ValueError):
        MPU6050(bus, gyro_range_dps=100)
    # dlpf=0 turns the filter off and changes the output rate to 8 kHz, which
    # would silently break the SMPLRT_DIV maths.
    with pytest.raises(ValueError):
        MPU6050(bus, dlpf=0)


def test_wrong_address_raises_like_a_real_bus():
    bus = FakeMPU6050Bus(address=0x68)
    imu = MPU6050(bus, address=0x69)
    with pytest.raises(OSError):
        imu.who_am_i()
