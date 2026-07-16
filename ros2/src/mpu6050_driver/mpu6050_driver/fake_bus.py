"""A simulated MPU6050 on a simulated I2C bus.

The dev machine has no I2C (docs/devcontainer.md), and the real chip is not
bought-and-mounted yet, so this stands in — the same trick as
hoverboard_bridge's fake_esp32.

It is deliberately register-level rather than a stub that returns canned
ImuSamples: it decodes FS_SEL/AFS_SEL back out of the config registers the
driver wrote and scales its raw counts accordingly. So if configure() writes the
wrong range bits, the numbers come out wrong and the tests fail — a stub at the
sample level would happily agree with a broken driver.

What it models: register semantics, byte order, ranges, gravity, a constant gyro
bias and gaussian noise. What it does NOT model: temperature drift, clipping,
cross-axis sensitivity, I2C glitches, or motor vibration — the things that will
actually bite on the robot. Passing here means the maths is right, not that the
sensor is good.
"""

from __future__ import annotations

import random
from typing import Dict, List

from mpu6050_driver.mpu6050 import (
    ACCEL_LSB_PER_G,
    BIT_DEVICE_RESET,
    BURST_LENGTH,
    DEFAULT_ADDRESS,
    DEG_TO_RAD,
    GYRO_LSB_PER_DPS,
    REG_ACCEL_CONFIG,
    REG_ACCEL_XOUT_H,
    REG_GYRO_CONFIG,
    REG_PWR_MGMT_1,
    REG_WHO_AM_I,
    STANDARD_GRAVITY,
    WHO_AM_I_VALUE,
)

_FS_SEL_TO_ACCEL_G = {0: 2, 1: 4, 2: 8, 3: 16}
_FS_SEL_TO_GYRO_DPS = {0: 250, 1: 500, 2: 1000, 3: 2000}


def _clamp_int16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def _split_be(value: int) -> List[int]:
    """Signed 16-bit to [high, low] — the chip's big-endian data registers."""
    unsigned = value & 0xFFFF
    return [(unsigned >> 8) & 0xFF, unsigned & 0xFF]


class FakeMPU6050Bus:
    """Quacks like the slice of smbus2.SMBus that MPU6050 uses."""

    def __init__(
        self,
        address: int = DEFAULT_ADDRESS,
        gyro_bias_dps: tuple = (1.7, -0.9, 2.4),
        noise_gyro_dps: float = 0.05,
        noise_accel_g: float = 0.01,
        seed: int = 0,
    ) -> None:
        self._address = address
        self._regs: Dict[int, int] = {REG_WHO_AM_I: WHO_AM_I_VALUE}
        self._rng = random.Random(seed)  # seeded: the tests must be deterministic

        # A real MPU6050 always has a gyro bias of a few deg/s. It is the whole
        # reason the node calibrates at startup, so the fake must have one too —
        # a zero-bias fake would let a broken calibration pass.
        self._gyro_bias_dps = gyro_bias_dps
        self._noise_gyro_dps = noise_gyro_dps
        self._noise_accel_g = noise_accel_g

        # Ground truth the test can drive. Defaults: level and stationary, so
        # the accelerometer reads +1 g on z (it measures the reaction to gravity).
        self.true_accel_g = (0.0, 0.0, 1.0)
        self.true_gyro_dps = (0.0, 0.0, 0.0)
        self.true_temp_c = 32.0
        self.asleep = True

    # ---- smbus2-compatible surface ------------------------------------------
    def read_byte_data(self, addr: int, register: int) -> int:
        self._check_addr(addr)
        return self._regs.get(register, 0)

    def write_byte_data(self, addr: int, register: int, value: int) -> None:
        self._check_addr(addr)
        if register == REG_PWR_MGMT_1:
            if value & BIT_DEVICE_RESET:
                # Reset clears config and leaves the device asleep, as the real
                # one does — a driver that forgets to wake up must fail here.
                self._regs = {REG_WHO_AM_I: WHO_AM_I_VALUE}
                self.asleep = True
                return
            self.asleep = bool(value & 0x40)  # SLEEP bit
        self._regs[register] = value

    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]:
        self._check_addr(addr)
        if register != REG_ACCEL_XOUT_H or length != BURST_LENGTH:
            raise OSError(f"unexpected block read: reg=0x{register:02x} len={length}")
        if self.asleep:
            raise OSError("device is asleep — no data")
        return self._burst()

    def close(self) -> None:
        pass

    # ---- Simulation ----------------------------------------------------------
    def _check_addr(self, addr: int) -> None:
        if addr != self._address:
            raise OSError(121, "Remote I/O error")  # what a real bus gives you

    def _accel_lsb_per_g(self) -> float:
        fs_sel = (self._regs.get(REG_ACCEL_CONFIG, 0) >> 3) & 0x03
        return ACCEL_LSB_PER_G[_FS_SEL_TO_ACCEL_G[fs_sel]]

    def _gyro_lsb_per_dps(self) -> float:
        fs_sel = (self._regs.get(REG_GYRO_CONFIG, 0) >> 3) & 0x03
        return GYRO_LSB_PER_DPS[_FS_SEL_TO_GYRO_DPS[fs_sel]]

    def _burst(self) -> List[int]:
        accel_lsb = self._accel_lsb_per_g()
        gyro_lsb = self._gyro_lsb_per_dps()

        out: List[int] = []
        for axis in range(3):
            g = self.true_accel_g[axis] + self._rng.gauss(0.0, self._noise_accel_g)
            out += _split_be(_clamp_int16(g * accel_lsb))
        out += _split_be(_clamp_int16((self.true_temp_c - 36.53) * 340.0))
        for axis in range(3):
            dps = (
                self.true_gyro_dps[axis]
                + self._gyro_bias_dps[axis]
                + self._rng.gauss(0.0, self._noise_gyro_dps)
            )
            out += _split_be(_clamp_int16(dps * gyro_lsb))
        return out

    # ---- Convenience for callers driving the simulation ----------------------
    def set_yaw_rate(self, rad_s: float) -> None:
        self.true_gyro_dps = (0.0, 0.0, rad_s / DEG_TO_RAD)

    def set_gravity_only(self) -> None:
        self.true_accel_g = (0.0, 0.0, 1.0)

    def set_forward_accel(self, m_s2: float) -> None:
        self.true_accel_g = (m_s2 / STANDARD_GRAVITY, 0.0, 1.0)
