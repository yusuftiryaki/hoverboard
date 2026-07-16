"""Register-level MPU6050 driver. No ROS, no I2C library import.

The bus is injected (anything with smbus2.SMBus's read_byte_data /
write_byte_data / read_i2c_block_data), so this module can be unit tested on a
dev machine with no I2C at all — see fake_bus.py. That is the whole reason it is
separate from imu_node.py.

Register map and scale factors: InvenSense MPU-6000/MPU-6050 Register Map rev 4.2
and Product Specification rev 3.4.

⚠️ This chip is BIG-endian in its data registers (high byte first), unlike the
hoverboard link's little-endian frames. Mixing the two up yields plausible-looking
garbage, so the byte order is asserted in the tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol

# ---- Registers --------------------------------------------------------------
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B   # start of the 14-byte burst: accel, temp, gyro
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

BURST_LENGTH = 14

# PWR_MGMT_1 bits
BIT_DEVICE_RESET = 0x80
CLKSEL_PLL_X_GYRO = 0x01  # datasheet: better reference than the internal oscillator

DEFAULT_ADDRESS = 0x68
WHO_AM_I_VALUE = 0x68

# Scale factors, LSB per unit (PS rev 3.4, sections 6.1 and 6.2).
ACCEL_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

STANDARD_GRAVITY = 9.80665  # m/s^2, per REP-103
DEG_TO_RAD = 3.141592653589793 / 180.0

# With the DLPF enabled (CONFIG 1..6) the gyro output rate is 1 kHz, and
# SMPLRT_DIV divides that down. With DLPF off it would be 8 kHz — one more reason
# to always keep the filter on.
GYRO_OUTPUT_RATE_HZ = 1000.0


class I2CBus(Protocol):
    """The slice of smbus2.SMBus this driver uses."""

    def read_byte_data(self, addr: int, register: int) -> int: ...
    def write_byte_data(self, addr: int, register: int, value: int) -> None: ...
    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]: ...


@dataclass(frozen=True)
class ImuSample:
    """One burst read, already in SI units and the chip's own axes."""

    accel: tuple      # (x, y, z) m/s^2
    gyro: tuple       # (x, y, z) rad/s
    temperature: float  # deg C (the die, not the ambient — it self-heats)


def _to_signed16(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


class MPU6050:
    def __init__(
        self,
        bus: I2CBus,
        address: int = DEFAULT_ADDRESS,
        accel_range_g: int = 4,
        gyro_range_dps: int = 250,
        dlpf: int = 3,
        sample_rate_hz: float = 100.0,
    ) -> None:
        if accel_range_g not in ACCEL_LSB_PER_G:
            raise ValueError(f"accel range must be one of {sorted(ACCEL_LSB_PER_G)}")
        if gyro_range_dps not in GYRO_LSB_PER_DPS:
            raise ValueError(f"gyro range must be one of {sorted(GYRO_LSB_PER_DPS)}")
        if not 1 <= dlpf <= 6:
            # 0 disables the filter and raises the output rate to 8 kHz, which
            # breaks the SMPLRT_DIV maths below and lets motor vibration straight
            # through. Refuse rather than silently mis-sample.
            raise ValueError("dlpf must be 1..6 (0 = filter off, not supported)")

        self._bus = bus
        self._address = address
        self._accel_lsb_per_g = ACCEL_LSB_PER_G[accel_range_g]
        self._gyro_lsb_per_dps = GYRO_LSB_PER_DPS[gyro_range_dps]
        self._accel_range_g = accel_range_g
        self._gyro_range_dps = gyro_range_dps
        self._dlpf = dlpf
        self._sample_rate_hz = sample_rate_hz

    # ---- Setup --------------------------------------------------------------
    def who_am_i(self) -> int:
        return self._bus.read_byte_data(self._address, REG_WHO_AM_I)

    def reset(self) -> None:
        """Reset the device. Caller must wait ~100 ms afterwards."""
        self._bus.write_byte_data(self._address, REG_PWR_MGMT_1, BIT_DEVICE_RESET)

    def configure(self) -> None:
        """Wake the device and apply the ranges, filter and sample rate.

        Call after reset() and its settling delay.
        """
        # Wake up (reset leaves SLEEP set) and pick the gyro PLL as the clock.
        self._bus.write_byte_data(self._address, REG_PWR_MGMT_1, CLKSEL_PLL_X_GYRO)
        self._bus.write_byte_data(self._address, REG_CONFIG, self._dlpf)
        self._bus.write_byte_data(
            self._address, REG_GYRO_CONFIG, self._gyro_fs_sel() << 3
        )
        self._bus.write_byte_data(
            self._address, REG_ACCEL_CONFIG, self._accel_fs_sel() << 3
        )
        self._bus.write_byte_data(self._address, REG_SMPLRT_DIV, self.sample_rate_divider())

    def sample_rate_divider(self) -> int:
        divider = int(round(GYRO_OUTPUT_RATE_HZ / self._sample_rate_hz)) - 1
        return max(0, min(255, divider))

    def actual_sample_rate_hz(self) -> float:
        """The rate the chip will really produce — SMPLRT_DIV is an integer."""
        return GYRO_OUTPUT_RATE_HZ / (self.sample_rate_divider() + 1)

    def _accel_fs_sel(self) -> int:
        return {2: 0, 4: 1, 8: 2, 16: 3}[self._accel_range_g]

    def _gyro_fs_sel(self) -> int:
        return {250: 0, 500: 1, 1000: 2, 2000: 3}[self._gyro_range_dps]

    # ---- Reading ------------------------------------------------------------
    def read(self) -> ImuSample:
        """One burst read of accel + temp + gyro, converted to SI units.

        Burst-reading all 14 bytes in one transaction matters: reading the axes
        separately can straddle a sample update and mix two different instants
        into one "measurement".
        """
        raw = self._bus.read_i2c_block_data(self._address, REG_ACCEL_XOUT_H, BURST_LENGTH)
        if len(raw) != BURST_LENGTH:
            raise OSError(f"short I2C read: {len(raw)} of {BURST_LENGTH} bytes")

        ax = _to_signed16(raw[0], raw[1])
        ay = _to_signed16(raw[2], raw[3])
        az = _to_signed16(raw[4], raw[5])
        temp_raw = _to_signed16(raw[6], raw[7])
        gx = _to_signed16(raw[8], raw[9])
        gy = _to_signed16(raw[10], raw[11])
        gz = _to_signed16(raw[12], raw[13])

        accel_scale = STANDARD_GRAVITY / self._accel_lsb_per_g
        gyro_scale = DEG_TO_RAD / self._gyro_lsb_per_dps
        return ImuSample(
            accel=(ax * accel_scale, ay * accel_scale, az * accel_scale),
            gyro=(gx * gyro_scale, gy * gyro_scale, gz * gyro_scale),
            # PS rev 3.4 section 4.18.
            temperature=temp_raw / 340.0 + 36.53,
        )
