"""A simulated QMC5883L on a simulated I2C bus.

Same trick as mpu6050_driver's fake_bus: register-level, not a canned-sample
stub, so it decodes the range bits the driver actually wrote and scales its
counts accordingly. A stub at the sample level would agree with a broken driver.

It synthesises the earth's field for a given heading, and — importantly — can add
a HARD IRON offset. That distortion is the whole reason the mag needs calibrating
(handoff decision 4: motors and steel warp it), so a fake without one would let a
broken calibration pass.
"""

from __future__ import annotations

import math
import random
from typing import Dict, List

from qmc5883l_driver.qmc5883l import (
    BIT_SOFT_RST,
    CHIP_ID_VALUE,
    DATA_LENGTH,
    DEFAULT_ADDRESS,
    GAUSS_TO_TESLA,
    LSB_PER_GAUSS,
    MODE_CONTINUOUS,
    REG_CHIP_ID,
    REG_CONTROL_1,
    REG_CONTROL_2,
    REG_DATA_X_LSB,
    REG_STATUS,
    STATUS_DRDY,
    STATUS_OVL,
)

_RNG_BITS_TO_GAUSS = {0b00: 2, 0b01: 8}

# Istanbul-ish: ~26 uT north, ~36 uT down. The horizontal component is what a
# compass actually works with; the vertical part only matters once the robot tilts.
EARTH_NORTH_T = 26e-6
EARTH_DOWN_T = 36e-6


def _clamp_int16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def _split_le(value: int) -> List[int]:
    """Signed 16-bit to [low, high] — this chip is little-endian."""
    unsigned = value & 0xFFFF
    return [unsigned & 0xFF, (unsigned >> 8) & 0xFF]


class FakeQMC5883LBus:
    """Quacks like the slice of smbus2.SMBus that QMC5883L uses."""

    def __init__(
        self,
        address: int = DEFAULT_ADDRESS,
        hard_iron_t: tuple = (8e-6, -5e-6, 3e-6),
        noise_t: float = 0.2e-6,
        seed: int = 0,
    ) -> None:
        self._address = address
        self._regs: Dict[int, int] = {REG_CHIP_ID: CHIP_ID_VALUE}
        self._rng = random.Random(seed)
        self._hard_iron = hard_iron_t
        self._noise = noise_t

        # Ground truth the test drives. yaw is REP-103: 0 = facing east, CCW.
        self.yaw = 0.0
        self.standby = True

    # ---- smbus2-compatible surface ------------------------------------------
    def read_byte_data(self, addr: int, register: int) -> int:
        self._check(addr)
        if register == REG_STATUS:
            if self.standby:
                return 0
            status = STATUS_DRDY
            if self._saturated():
                status |= STATUS_OVL
            return status
        return self._regs.get(register, 0)

    def write_byte_data(self, addr: int, register: int, value: int) -> None:
        self._check(addr)
        if register == REG_CONTROL_2 and value & BIT_SOFT_RST:
            # Reset drops config and returns to standby, as the real one does.
            self._regs = {REG_CHIP_ID: CHIP_ID_VALUE}
            self.standby = True
            return
        if register == REG_CONTROL_1:
            self.standby = (value & 0b11) != MODE_CONTINUOUS
        self._regs[register] = value

    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]:
        self._check(addr)
        if register != REG_DATA_X_LSB or length != DATA_LENGTH:
            raise OSError(f"unexpected block read: reg=0x{register:02x} len={length}")
        if self.standby:
            raise OSError("device is in standby — no data")
        return self._data()

    def close(self) -> None:
        pass

    # ---- Simulation ----------------------------------------------------------
    def _check(self, addr: int) -> None:
        if addr != self._address:
            raise OSError(121, "Remote I/O error")

    def _range_gauss(self) -> int:
        return _RNG_BITS_TO_GAUSS[(self._regs.get(REG_CONTROL_1, 0) >> 4) & 0b11]

    def true_field(self) -> tuple:
        """The field in the sensor's frame, level, plus hard iron and noise.

        REP-103: x forward, y left, z up; yaw 0 = facing east. The earth's
        horizontal field points north, i.e. along +y at yaw 0.
        """
        # Rotating the robot by yaw rotates the field by -yaw in the body frame.
        bx = -EARTH_NORTH_T * math.sin(self.yaw)
        by = EARTH_NORTH_T * math.cos(self.yaw)
        bz = -EARTH_DOWN_T                       # down is -z in REP-103
        return (
            bx + self._hard_iron[0] + self._rng.gauss(0.0, self._noise),
            by + self._hard_iron[1] + self._rng.gauss(0.0, self._noise),
            bz + self._hard_iron[2] + self._rng.gauss(0.0, self._noise),
        )

    def _saturated(self) -> bool:
        limit = self._range_gauss() * GAUSS_TO_TESLA
        return any(abs(v) >= limit for v in self.true_field())

    def _data(self) -> List[int]:
        lsb_per_tesla = LSB_PER_GAUSS[self._range_gauss()] / GAUSS_TO_TESLA
        out: List[int] = []
        for value in self.true_field():
            out += _split_le(_clamp_int16(value * lsb_per_tesla))
        return out

    def set_heading_deg(self, degrees: float) -> None:
        self.yaw = math.radians(degrees)
