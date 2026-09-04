"""Register-level INA228 definitions and 20-bit register decoding."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol

REG_CONFIG = 0x00
REG_ADC_CONFIG = 0x01
REG_VSHUNT = 0x04
REG_VBUS = 0x05
REG_DIETEMP = 0x06
REG_MANUFACTURER_ID = 0x3E
REG_DEVICE_ID = 0x3F

DEFAULT_ADDRESS = 0x40
MANUFACTURER_TI = 0x5449
DEVICE_ID_VALUE = 0x228

VSHUNT_LSB_V = 312.5e-9
VBUS_LSB_V = 195.3125e-6
DIETEMP_LSB_C = 7.8125e-3

BIT_RESET = 0x8000
ADC_CONFIG_VALUE = (0xF << 12) | (0x5 << 9) | (0x5 << 6) | (0x5 << 3) | 0x2


class I2CBus(Protocol):
    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]: ...

    def write_i2c_block_data(self, addr: int, register: int, data: List[int]) -> None: ...


@dataclass(frozen=True)
class PowerSample:
    bus_voltage: float
    current: float
    temperature: float


def _to_signed20(raw24: int) -> int:
    value = raw24 >> 4
    return value - (1 << 20) if value >= (1 << 19) else value


def _to_signed16(raw16: int) -> int:
    return raw16 - 65536 if raw16 >= 32768 else raw16


class INA228:
    def __init__(self, bus: I2CBus, address: int = DEFAULT_ADDRESS,
                 shunt_ohms: float = 0.0015) -> None:
        if shunt_ohms <= 0.0:
            raise ValueError("shunt_ohms must be positive")
        self._bus = bus
        self._address = address
        self._shunt_ohms = shunt_ohms

    def probe(self) -> int:
        return self._read_u16(REG_DEVICE_ID) >> 4

    def reset(self) -> None:
        self._write_u16(REG_CONFIG, BIT_RESET)

    def configure(self) -> None:
        self._write_u16(REG_ADC_CONFIG, ADC_CONFIG_VALUE)

    def read(self) -> PowerSample:
        vshunt = _to_signed20(self._read_u24(REG_VSHUNT)) * VSHUNT_LSB_V
        vbus = (self._read_u24(REG_VBUS) >> 4) * VBUS_LSB_V
        temperature = _to_signed16(self._read_u16(REG_DIETEMP)) * DIETEMP_LSB_C
        return PowerSample(
            bus_voltage=vbus,
            current=vshunt / self._shunt_ohms,
            temperature=temperature,
        )

    def _read_u16(self, register: int) -> int:
        raw = self._bus.read_i2c_block_data(self._address, register, 2)
        if len(raw) != 2:
            raise OSError(f"short I2C read: {len(raw)} of 2 bytes")
        return (raw[0] << 8) | raw[1]

    def _read_u24(self, register: int) -> int:
        raw = self._bus.read_i2c_block_data(self._address, register, 3)
        if len(raw) != 3:
            raise OSError(f"short I2C read: {len(raw)} of 3 bytes")
        return (raw[0] << 16) | (raw[1] << 8) | raw[2]

    def _write_u16(self, register: int, value: int) -> None:
        self._bus.write_i2c_block_data(
            self._address, register, [(value >> 8) & 0xFF, value & 0xFF]
        )