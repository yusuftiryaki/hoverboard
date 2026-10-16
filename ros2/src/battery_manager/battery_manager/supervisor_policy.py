"""SP5: what to do about the battery. ROS-free decisions, no I/O.

    NORMAL ──LOW──▶ RETURNING ─────────────────────▶ DOCKING ──charging──▶ CHARGING ──FULL──▶ DOCKED_IDLE
      │             cancel missions → staging → face     (DockRobot)                  │
      │                                                                              resume → UNDOCKING → NORMAL
      └── CRITICAL, from NORMAL / RETURNING / DOCKING / UNDOCKING ──▶ SAFE_STOP (latched; resume clears)
          any step failing (staging, facing, docking, undocking) ──▶ SAFE_STOP

battery_supervisor_node runs this: it feeds /battery samples and the results of
the actions it was told to start, and executes the Steps that come back.

Thresholds read percentage when battery_monitor has one (INA228 fitted) and fall
back to per-cell voltage when it does not — through a slow filter, because a
pack under load sags and recovers, and a sag is not an empty battery. The
fallback is conservative on purpose: losing the current sensor makes the robot
cautious, not blind (design doc, "INA228 arızası").

What it deliberately does NOT do:
  * add a gate to the motor command path. "Stop" is: cancel every Nav2 goal and
    lock twist_mux's Nav2 input (`lock`). Teleop outranks the lock, so a person
    can still drive a stopped robot home; the E-stop stays the human/hardware
    channel; the deadman, watchdog and bumper veto are untouched.
  * leave the dock by itself. A full robot waits (v1 scope); `resume` undocks.
  * defend the return against a mission client that keeps sending goals. It
    cancels what is running when the battery goes LOW, once.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

# sensor_msgs/BatteryState power_supply_status values (kept here so this
# module stays importable without ROS).
STATUS_UNKNOWN = 0
STATUS_CHARGING = 1
STATUS_DISCHARGING = 2
STATUS_NOT_CHARGING = 3
STATUS_FULL = 4


class Phase(str, Enum):
    NORMAL = "NORMAL"
    RETURNING = "RETURNING"
    DOCKING = "DOCKING"
    CHARGING = "CHARGING"
    DOCKED_IDLE = "DOCKED_IDLE"
    UNDOCKING = "UNDOCKING"
    SAFE_STOP = "SAFE_STOP"


class Level(str, Enum):
    OK = "OK"
    LOW = "LOW"
    CRITICAL = "CRITICAL"
    UNKNOWN = "UNKNOWN"


class Step(str, Enum):
    """Things the node must go and do. Each one but CANCEL_ALL reports back
    through Supervisor.on_result."""
    CANCEL_MISSIONS = "CANCEL_MISSIONS"   # every Nav2 navigation/waypoint goal
    GO_TO_STAGING = "GO_TO_STAGING"       # NavigateToPose to the staging pose
    FACE_DOCK = "FACE_DOCK"               # turn in place until the marker is in view
    DOCK = "DOCK"                         # opennav_docking DockRobot
    UNDOCK = "UNDOCK"                     # opennav_docking UndockRobot
    CANCEL_ALL = "CANCEL_ALL"             # all of the above, fire and forget


# A robot that may drive: CRITICAL anywhere here stops it.
MOVING = (Phase.NORMAL, Phase.RETURNING, Phase.DOCKING, Phase.UNDOCKING)
# Phases that hold twist_mux's Nav2 input shut.
LOCKED = (Phase.SAFE_STOP, Phase.CHARGING, Phase.DOCKED_IDLE)


@dataclass(frozen=True)
class Thresholds:
    # CALIBRATE all of these on the real pack (design doc: DÖN ~25 %, KRİTİK ~10 %).
    return_soc: float = 0.25
    critical_soc: float = 0.10
    # Voltage fallback, per cell, on the FILTERED voltage.
    return_cell_v: float = 3.50
    critical_cell_v: float = 3.30
    cells: int = 10
    # A level must hold this long before it counts: one noisy sample, or one
    # hill climbed under load, is not a decision.
    debounce_s: float = 3.0
    # Time constant of the voltage filter used by the fallback.
    voltage_filter_s: float = 30.0
    # A /battery older than this is no reading at all.
    stale_s: float = 5.0
    # Charging current above which the robot counts as on the charger. Same
    # number as nav2.yaml's charging_threshold, which ends the docking action.
    charging_current_a: float = 0.5
    # In CHARGING, the pack discharging for this long means the charger is gone.
    charge_lost_s: float = 10.0

    def __post_init__(self) -> None:
        values = (self.return_soc, self.critical_soc, self.return_cell_v,
                  self.critical_cell_v, self.debounce_s, self.voltage_filter_s,
                  self.stale_s, self.charging_current_a, self.charge_lost_s)
        if not all(math.isfinite(v) for v in values) or self.cells <= 0:
            raise ValueError("thresholds must be finite and cells positive")
        # Inverted thresholds would send the robot home only AFTER it should
        # already have stopped where it was — i.e. never.
        if not 0.0 <= self.critical_soc < self.return_soc <= 1.0:
            raise ValueError("need 0 <= critical_soc < return_soc <= 1")
        if not 0.0 < self.critical_cell_v < self.return_cell_v:
            raise ValueError("need 0 < critical_cell_v < return_cell_v")


@dataclass(frozen=True)
class BatterySample:
    """The fields of sensor_msgs/BatteryState this module reads."""
    percentage: float          # 0..1, NaN when there is no current sensor
    voltage: float
    current: float             # + into the pack, NaN without a current sensor
    status: int


class LevelMonitor:
    """Raw sample -> debounced Level, from SoC or (fallback) filtered voltage."""

    def __init__(self, thresholds: Thresholds) -> None:
        self._t = thresholds
        self._filtered_v: Optional[float] = None
        self._last_time: Optional[float] = None
        self._candidate = Level.UNKNOWN
        self._candidate_since = 0.0
        self.level = Level.UNKNOWN
        self.source = "none"
        self.value = float("nan")

    def update(self, sample: BatterySample, now: float) -> Level:
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        raw = self._raw_level(sample, dt)
        if raw != self._candidate:
            self._candidate, self._candidate_since = raw, now
        # Getting WORSE waits out the debounce; UNKNOWN and recovering do not
        # need to (they never trigger an action by themselves).
        if raw in (Level.LOW, Level.CRITICAL) and raw != self.level:
            if now - self._candidate_since >= self._t.debounce_s:
                self.level = raw
        else:
            self.level = raw
        return self.level

    def stale(self) -> Level:
        """No /battery for stale_s: say so instead of keeping a guess."""
        self.level = self._candidate = Level.UNKNOWN
        self.source = "none"
        self._filtered_v = None
        self._last_time = None
        return self.level

    def _raw_level(self, sample: BatterySample, dt: float) -> Level:
        t = self._t
        if math.isfinite(sample.percentage):
            self.source, self.value = "soc", sample.percentage
            if sample.percentage < t.critical_soc:
                return Level.CRITICAL
            return Level.LOW if sample.percentage < t.return_soc else Level.OK
        if not math.isfinite(sample.voltage) or sample.voltage <= 0.0:
            self.source, self.value = "none", float("nan")
            return Level.UNKNOWN
        if self._filtered_v is None:
            self._filtered_v = sample.voltage
        else:
            alpha = 1.0 - math.exp(-dt / t.voltage_filter_s)
            self._filtered_v += (sample.voltage - self._filtered_v) * alpha
        cell = self._filtered_v / t.cells
        self.source, self.value = "voltage", cell
        if cell < t.critical_cell_v:
            return Level.CRITICAL
        return Level.LOW if cell < t.return_cell_v else Level.OK


def load_dock(path: str, dock_id: str) -> Dict:
    """The dock's database entry. The SAME file docking_server loads, so the
    two cannot disagree about where home is."""
    import yaml

    with open(path, encoding="utf-8") as handle:
        docks = (yaml.safe_load(handle) or {}).get("docks", {})
    if dock_id not in docks:
        raise ValueError(f"dock '{dock_id}' is not in {path} (has {sorted(docks)})")
    entry = docks[dock_id]
    pose = [float(v) for v in entry["pose"]]
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError(f"dock '{dock_id}' pose must be [x, y, yaw], got {entry['pose']}")
    frame = entry.get("frame", "map")
    return {"x": pose[0], "y": pose[1], "yaw": pose[2], "frame": frame}


class Supervisor:
    def __init__(self, thresholds: Thresholds) -> None:
        self.t = thresholds
        self.monitor = LevelMonitor(thresholds)
        self.phase = Phase.NORMAL
        self.reason = "starting"
        self.pending: Optional[Step] = None      # the step we are waiting on
        self._discharging_since: Optional[float] = None

    @property
    def lock(self) -> bool:
        return self.phase in LOCKED

    # ---- inputs ---------------------------------------------------------------
    def on_battery(self, sample: BatterySample, now: float) -> List[Step]:
        level = self.monitor.update(sample, now)
        return self._evaluate(level, sample, now)

    def on_stale(self, now: float) -> List[Step]:
        level = self.monitor.stale()
        return self._evaluate(level, None, now)

    def on_result(self, step: Step, ok: bool, detail: str = "") -> List[Step]:
        """An action the node started has finished."""
        if step != self.pending:
            return []          # a late result for something already superseded
        self.pending = None
        if self.phase == Phase.RETURNING:
            if step == Step.CANCEL_MISSIONS:
                # Even if a cancel went unanswered: going home beats staying.
                return self._start(Step.GO_TO_STAGING)
            if step == Step.GO_TO_STAGING:
                if not ok:
                    return self._safe_stop(f"could not reach the staging pose: {detail}")
                return self._start(Step.FACE_DOCK)
            if step == Step.FACE_DOCK:
                if not ok:
                    return self._safe_stop(f"could not turn to face the dock: {detail}")
                self._enter(Phase.DOCKING, "docking")
                return self._start(Step.DOCK)
        if self.phase == Phase.DOCKING and step == Step.DOCK:
            if not ok:
                return self._safe_stop(f"docking failed: {detail}")
            self._discharging_since = None
            self._enter(Phase.CHARGING, "docked and charging")
            return []
        if self.phase == Phase.UNDOCKING and step == Step.UNDOCK:
            if not ok:
                return self._safe_stop(f"undocking failed: {detail}")
            self._enter(Phase.NORMAL, "undocked")
            return []
        return []

    def resume(self) -> Tuple[bool, str, List[Step]]:
        """A person says carry on. Returns (accepted, message, steps)."""
        if self.phase in (Phase.CHARGING, Phase.DOCKED_IDLE):
            self._enter(Phase.UNDOCKING, "undocking on request")
            return True, "undocking", self._start(Step.UNDOCK)
        if self.phase == Phase.SAFE_STOP:
            self._enter(Phase.NORMAL, "resumed by request")
            return True, "resumed — the battery is re-evaluated on the next sample", []
        return False, f"nothing to resume in {self.phase.value}", []

    # ---- the machine ------------------------------------------------------------
    def _evaluate(self, level: Level, sample: Optional[BatterySample],
                  now: float) -> List[Step]:
        if level == Level.CRITICAL and self.phase in MOVING:
            return self._safe_stop(
                f"battery CRITICAL ({self.monitor.source} {self.monitor.value:.3f})")

        if self.phase == Phase.NORMAL:
            if sample is not None and self._charging(sample):
                # Put on the charger by hand, or booted on the dock.
                self._discharging_since = None
                self._enter(Phase.CHARGING, "charging")
                return []
            if level == Level.LOW:
                self._enter(Phase.RETURNING, f"battery LOW ({self.monitor.source} "
                                             f"{self.monitor.value:.3f}) — returning to dock")
                return self._start(Step.CANCEL_MISSIONS)
            return []

        if self.phase == Phase.CHARGING and sample is not None:
            if sample.status == STATUS_FULL:
                self._enter(Phase.DOCKED_IDLE, "full — waiting on the dock")
                return []
            if sample.status == STATUS_DISCHARGING:
                if self._discharging_since is None:
                    self._discharging_since = now
                elif now - self._discharging_since >= self.t.charge_lost_s:
                    return self._safe_stop(
                        "on the dock but the pack is discharging — contacts or "
                        "charger lost")
            else:
                self._discharging_since = None
        return []

    def _charging(self, sample: BatterySample) -> bool:
        return (math.isfinite(sample.current)
                and sample.current > self.t.charging_current_a)

    def _start(self, step: Step) -> List[Step]:
        self.pending = step
        return [step]

    def _enter(self, phase: Phase, reason: str) -> None:
        self.phase, self.reason = phase, reason

    def _safe_stop(self, reason: str) -> List[Step]:
        self.pending = None
        self._enter(Phase.SAFE_STOP, reason)
        return [Step.CANCEL_ALL]
