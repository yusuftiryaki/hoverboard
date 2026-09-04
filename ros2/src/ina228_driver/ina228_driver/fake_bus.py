"""Register-level INA228 simulation for deterministic tests."""

from __future__ import annotations

import random
from typing import Dict, List

from ina228_driver.ina228 import (
    BIT_RESET,
    DEFAULT_ADDRESS,
    DEVICE_ID_VALUE,
    DIETEMP_LSB_C,
    MANUFACTURER_TI,
    REG_CONFIG,
    REG_DIETEMP,
    REG_DEVICE_ID,
    REG_MANUFACTURER_ID,
    REG_VBUS,
    REG_VSHUNT,
    VBUS_LSB_V,
    VSHUNT_LSB_V,
)

_WIDTHS = {REG_VSHUNT: 3, REG_VBUS: 3}


def _clamp_s20(value: float) -> int:
    return max(-(1 << 19), min((1 << 19) - 1, int(round(value))))


class FakeINA228Bus:
    def __init__(self, address: int = DEFAULT_ADDRESS, rshunt_ohms: float = 0.0015,
                 noise_current_a: float = 0.0, seed: int = 0) -> None:
        self._address = address
        self._rshunt = rshunt_ohms
        self._noise_a = noise_current_a
        self._rng = random.Random(seed)
        self._regs: Dict[int, int] = {}
        self.true_current_a = 0.0
        self.true_bus_v = 36.0
        self.true_temp_c = 30.0

    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]:
        self._check_addr(addr)
        if length != _WIDTHS.get(register, 2):
            raise OSError(f"unexpected read length {length} for register 0x{register:02x}")
        if register == REG_MANUFACTURER_ID:
            value = MANUFACTURER_TI
        elif register == REG_DEVICE_ID:
            value = (DEVICE_ID_VALUE << 4) | 0x1
        elif register == REG_VSHUNT:
            amps = self.true_current_a + self._rng.gauss(0.0, self._noise_a)
            value = (_clamp_s20(amps * self._rshunt / VSHUNT_LSB_V) & 0xFFFFF) << 4
        elif register == REG_VBUS:
            counts = max(0, int(round(self.true_bus_v / VBUS_LSB_V)))
            value = (counts & 0xFFFFF) << 4
        elif register == REG_DIETEMP:
            value = int(round(self.true_temp_c / DIETEMP_LSB_C)) & 0xFFFF
        else:
            value = self._regs.get(register, 0)
        width = _WIDTHS.get(register, 2)
        return [(value >> (8 * index)) & 0xFF for index in reversed(range(width))]

    def write_i2c_block_data(self, addr: int, register: int, data: List[int]) -> None:
        self._check_addr(addr)
        if len(data) != 2:
            raise OSError(f"expected 2 write bytes, got {len(data)}")
        value = (data[0] << 8) | data[1]
        if register == REG_CONFIG and value & BIT_RESET:
            self._regs = {}
            return
        self._regs[register] = value

    def close(self) -> None:
        pass

    def reg16(self, register: int) -> int:
        return self._regs.get(register, 0)

    def _check_addr(self, addr: int) -> None:
        if addr != self._address:
            raise OSError(121, "Remote I/O error")