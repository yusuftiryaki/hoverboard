"""The simulated 10S pack: what drains it, what charges it, what the wires read.

ROS-free. sim_node steps it every ESP32 tick and hands its two outputs to the
two sensors that would measure them on the robot:

    terminal voltage  -> Esp32Sim's bat_voltage -> protocol -> bridge -> /battery_raw
    pack current      -> /sim/battery_current   -> battery_monitor's fake INA228

so battery_monitor's estimator, the INA228 register driver and the bridge's
protocol path all stay in the loop. Nothing here is what the stack BELIEVES; it
is what the pack DOES, and the estimator has to work it out.

⚠️ Every number below is a GUESS, labelled as one. The chemistry curve is a
generic 18650 NMC datasheet shape, not this pack (its cells are unknown and one
of the two packs is dead); the load figures are a ~15 kg robot on grass. They
are good enough to exercise the return-to-dock logic and NOT good enough to
predict range. B4 (calibration) replaces them.

Sign convention: sensor_msgs/BatteryState — current is POSITIVE INTO the pack
(charging) and NEGATIVE out of it, the same sign battery_monitor publishes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

# Rest (open-circuit) voltage of ONE cell against state of charge. A generic
# NMC 18650 curve read off datasheet plots, deliberately NOT battery_manager's
# OCV_PER_CELL: the estimator's table is its own belief about the pack, and a
# simulator that shares it would only prove the estimator agrees with itself
# (handoff decision 10). The two disagree by a few percent, as a real pack and
# a generic table do.
SIM_OCV_PER_CELL = (
    (0.00, 3.00), (0.05, 3.45), (0.10, 3.55), (0.20, 3.65), (0.30, 3.72),
    (0.40, 3.78), (0.50, 3.85), (0.60, 3.92), (0.70, 3.99), (0.80, 4.06),
    (0.90, 4.13), (1.00, 4.20),
)


def cell_ocv(soc: float) -> float:
    soc = max(0.0, min(1.0, soc))
    for (s0, v0), (s1, v1) in zip(SIM_OCV_PER_CELL, SIM_OCV_PER_CELL[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return SIM_OCV_PER_CELL[-1][1]


@dataclass
class PackModel:
    capacity_ah: float = 4.4
    cells: int = 10
    soc: float = 0.8
    # Pack internal resistance. GUESS: ~15 mOhm per 18650 cell, 10S1P.
    internal_resistance: float = 0.15
    # What the mainboard draws standing still. The Pi and ESP32 are NOT on this
    # pack (handoff decision 13: they run off the powerbank).
    idle_current_a: float = 0.2
    # GUESS: rolling resistance on grass, ~15 kg, plus motor losses.
    amps_per_mps: float = 1.5
    amps_per_radps: float = 0.5
    # The station's charger: the stock 42 V / 2 A hoverboard brick, CC/CV.
    charger_current_a: float = 2.0
    charger_voltage: float = 42.0

    def __post_init__(self) -> None:
        if self.capacity_ah <= 0.0 or self.cells <= 0 or self.internal_resistance <= 0.0:
            raise ValueError("capacity, cells and internal resistance must be positive")
        self.soc = max(0.0, min(1.0, self.soc))

    @property
    def open_circuit_voltage(self) -> float:
        return cell_ocv(self.soc) * self.cells

    def step(self, dt: float, speed_mps: float, yaw_rate: float,
             on_charger: bool) -> Tuple[float, float]:
        """Advance dt seconds. Returns (terminal voltage, pack current)."""
        load = (self.idle_current_a + self.amps_per_mps * abs(speed_mps)
                + self.amps_per_radps * abs(yaw_rate))
        ocv = self.open_circuit_voltage
        charge = 0.0
        if on_charger:
            # CC until the terminal would pass the charger's CV setpoint, then
            # whatever current holds it there: the taper a real charger shows.
            charge = min(self.charger_current_a,
                         max(0.0, (self.charger_voltage - ocv) / self.internal_resistance))
        current = charge - load
        if dt > 0.0:
            self.soc += current * dt / 3600.0 / self.capacity_ah
            self.soc = max(0.0, min(1.0, self.soc))
        return ocv + current * self.internal_resistance, current
