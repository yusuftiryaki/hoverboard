"""Register-level QMC5883L driver. No ROS, no I2C library import.

Same shape as mpu6050_driver: the bus is injected, so this is unit testable on a
machine with no I2C — see fake_bus.py.

Register map: QST QMC5883L datasheet rev 1.0.

⚠️ This chip is LITTLE-endian in its data registers — the opposite of the
MPU6050 sitting on the same I2C bus. Getting it backwards does not fail, it just
produces a heading that is confidently wrong, so the byte order is pinned by test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol, Tuple

# ---- Registers --------------------------------------------------------------
REG_DATA_X_LSB = 0x00   # X, Y, Z: 6 bytes, little-endian
REG_STATUS = 0x06
REG_CONTROL_1 = 0x09
REG_CONTROL_2 = 0x0A
REG_SET_RESET = 0x0B
REG_CHIP_ID = 0x0D      # confusingly the same number as the I2C address

DATA_LENGTH = 6

# Status bits
STATUS_DRDY = 0x01      # a fresh measurement is waiting
STATUS_OVL = 0x02       # the field saturated the selected range
STATUS_DOR = 0x04       # data was overwritten before being read

# Control 1 fields
MODE_STANDBY = 0b00
MODE_CONTINUOUS = 0b01

# Control 2 bits
BIT_SOFT_RST = 0x80

DEFAULT_ADDRESS = 0x0D
CHIP_ID_VALUE = 0xFF

# Datasheet section 7.1: counts per Gauss for each range.
LSB_PER_GAUSS = {2: 12000.0, 8: 3000.0}
GAUSS_TO_TESLA = 1e-4   # sensor_msgs/MagneticField is in tesla

ODR_HZ = {10: 0b00, 50: 0b01, 100: 0b10, 200: 0b11}
OSR_SAMPLES = {512: 0b00, 256: 0b01, 128: 0b10, 64: 0b11}
RANGE_GAUSS = {2: 0b00, 8: 0b01}


class I2CBus(Protocol):
    def read_byte_data(self, addr: int, register: int) -> int: ...
    def write_byte_data(self, addr: int, register: int, value: int) -> None: ...
    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]: ...


@dataclass(frozen=True)
class MagSample:
    """One reading, in tesla, in the chip's own axes. Raw — no calibration."""

    field: Tuple[float, float, float]
    overflow: bool   # the range saturated; this sample is not trustworthy


def _to_signed16(low: int, high: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


class QMC5883L:
    def __init__(
        self,
        bus: I2CBus,
        address: int = DEFAULT_ADDRESS,
        range_gauss: int = 2,
        odr_hz: int = 100,
        osr: int = 512,
    ) -> None:
        if range_gauss not in RANGE_GAUSS:
            raise ValueError(f"range must be one of {sorted(RANGE_GAUSS)} Gauss")
        if odr_hz not in ODR_HZ:
            raise ValueError(f"ODR must be one of {sorted(ODR_HZ)} Hz")
        if osr not in OSR_SAMPLES:
            raise ValueError(f"OSR must be one of {sorted(OSR_SAMPLES)}")

        self._bus = bus
        self._address = address
        self._range_gauss = range_gauss
        self._odr_hz = odr_hz
        self._osr = osr
        self._lsb_per_gauss = LSB_PER_GAUSS[range_gauss]

    def chip_id(self) -> int:
        return self._bus.read_byte_data(self._address, REG_CHIP_ID)

    def reset(self) -> None:
        self._bus.write_byte_data(self._address, REG_CONTROL_2, BIT_SOFT_RST)

    def configure(self) -> None:
        """Wake into continuous mode. Call after reset() and a short delay."""
        # Datasheet 7.5: the SET/RESET period register must be 0x01. It is not
        # optional and not explained — leave it out and the readings wander.
        self._bus.write_byte_data(self._address, REG_SET_RESET, 0x01)
        control_1 = (
            (OSR_SAMPLES[self._osr] << 6)
            | (RANGE_GAUSS[self._range_gauss] << 4)
            | (ODR_HZ[self._odr_hz] << 2)
            | MODE_CONTINUOUS
        )
        self._bus.write_byte_data(self._address, REG_CONTROL_1, control_1)

    def standby(self) -> None:
        self._bus.write_byte_data(self._address, REG_CONTROL_1, MODE_STANDBY)

    def data_ready(self) -> bool:
        return bool(self._bus.read_byte_data(self._address, REG_STATUS) & STATUS_DRDY)

    def read(self) -> Optional[MagSample]:
        """One reading in tesla, or None if no fresh data is waiting."""
        status = self._bus.read_byte_data(self._address, REG_STATUS)
        if not status & STATUS_DRDY:
            return None

        raw = self._bus.read_i2c_block_data(self._address, REG_DATA_X_LSB, DATA_LENGTH)
        if len(raw) != DATA_LENGTH:
            raise OSError(f"short I2C read: {len(raw)} of {DATA_LENGTH} bytes")

        scale = GAUSS_TO_TESLA / self._lsb_per_gauss
        return MagSample(
            field=(
                _to_signed16(raw[0], raw[1]) * scale,
                _to_signed16(raw[2], raw[3]) * scale,
                _to_signed16(raw[4], raw[5]) * scale,
            ),
            # OVL means the field exceeded the range: near a steel chassis or the
            # hub motors this is the reading that quietly lies, so it is surfaced
            # rather than swallowed.
            overflow=bool(status & STATUS_OVL),
        )
