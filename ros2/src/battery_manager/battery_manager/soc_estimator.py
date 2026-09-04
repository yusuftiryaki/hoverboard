"""Coulomb-counting state-of-charge estimator. No ROS or hardware imports."""

from __future__ import annotations

OCV_PER_CELL = [
    (3.00, 0.00), (3.30, 0.02), (3.50, 0.06), (3.60, 0.13), (3.70, 0.28),
    (3.80, 0.45), (3.90, 0.62), (4.00, 0.78), (4.10, 0.90), (4.20, 1.00),
]


def _interp_ocv(cell_voltage: float) -> float:
    if cell_voltage <= OCV_PER_CELL[0][0]:
        return OCV_PER_CELL[0][1]
    for (v0, s0), (v1, s1) in zip(OCV_PER_CELL, OCV_PER_CELL[1:]):
        if cell_voltage <= v1:
            return s0 + (s1 - s0) * (cell_voltage - v0) / (v1 - v0)
    return OCV_PER_CELL[-1][1]


class SocEstimator:
    def __init__(self, capacity_ah: float, cells: int = 10,
                 full_voltage_per_cell: float = 4.15,
                 taper_current_a: float = 0.3, full_hold_s: float = 30.0) -> None:
        if capacity_ah <= 0.0:
            raise ValueError("capacity_ah must be positive")
        if cells <= 0:
            raise ValueError("cells must be positive")
        if full_hold_s < 0.0 or taper_current_a < 0.0:
            raise ValueError("charge completion parameters must be non-negative")
        self._capacity_ah = capacity_ah
        self._cells = cells
        self._full_voltage = full_voltage_per_cell * cells
        self._taper_a = taper_current_a
        self._full_hold_s = full_hold_s
        self._soc: float | None = None
        self._full_timer_s = 0.0
        self.just_reached_full = False

    @property
    def initialized(self) -> bool:
        return self._soc is not None

    @property
    def soc(self) -> float | None:
        return self._soc

    def initialize_from_rest_voltage(self, pack_voltage: float) -> float:
        self._soc = _interp_ocv(pack_voltage / self._cells)
        self._full_timer_s = 0.0
        self.just_reached_full = False
        return self._soc

    def update(self, current_a: float, pack_voltage: float, dt: float) -> float:
        if self._soc is None:
            raise RuntimeError("call initialize_from_rest_voltage() first")
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        self.just_reached_full = False
        self._soc += current_a * dt / 3600.0 / self._capacity_ah
        self._soc = max(0.0, min(1.0, self._soc))

        if pack_voltage >= self._full_voltage and abs(current_a) <= self._taper_a:
            self._full_timer_s += dt
            if self._full_timer_s >= self._full_hold_s and self._soc < 1.0:
                self._soc = 1.0
                self.just_reached_full = True
        else:
            self._full_timer_s = 0.0
        return self._soc