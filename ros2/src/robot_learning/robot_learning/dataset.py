"""Episode -> fixed-rate training frames (docs/learning.md, 5d). ROS-free.

export.py reads a bag into plain series ((t, values...) tuples, seconds);
this turns them into one row per frame at `fps`:

    frame k at t_k = t0 + k / fps
    state      every series' LATEST value at or before t_k (zero-order hold)
    image      the latest camera frame stamped at or before t_k
    action     the command given in (t_k, t_k + 1/fps]: what was done in
               answer to observation k — and if none came in that interval,
               the latest one still in force
    source     who gave it: teleop if a /cmd_vel_teleop is fresh (twist_mux's
               0.5 s), else nav if a /cmd_vel_nav is fresh, else none (= 0)

Three action columns are kept apart on purpose (learning.md decision 3):
`action` (the label above), `action_executed` (/cmd_vel: what reached the
bridge after mux, gate and locks) — and `teleop` marks the frames where a
person held the wheels; in an "intervention" episode those are DAgger's data.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

FRESH_S = 0.5                    # twist_mux's input timeout (config/twist_mux.yaml)
SOURCES = ("none", "teleop", "nav")

# The state vector, in this order. A missing series is NaN (never 0: a zero
# battery or speed is a fact, an unrecorded one is not).
STATE_NAMES = (
    "odom.v", "odom.w",                              # wheel odometry twist
    "local.x", "local.y", "local.yaw",               # ekf_local, odom frame
    "map.x", "map.y", "map.yaw",                     # ekf_global, map frame
    "imu.gz", "imu.ax", "imu.ay",
    "gps.lat", "gps.lon",                            # /gps/filtered
    "battery.pct", "bumper",
)
STATE_SERIES = (("odom", 2), ("local", 3), ("map", 3), ("imu", 3), ("gps", 2),
                ("battery", 1), ("bumper", 1))
ACTION_NAMES = ("linear", "angular")


@dataclass
class Series:
    """Sorted (t, values) samples of one signal."""
    times: List[float] = field(default_factory=list)
    values: List[Tuple[float, ...]] = field(default_factory=list)

    @classmethod
    def of(cls, samples: Sequence[Tuple[float, ...]]) -> "Series":
        ordered = sorted(samples, key=lambda s: s[0])
        return cls([s[0] for s in ordered], [tuple(s[1:]) for s in ordered])

    def latest(self, t: float) -> Optional[Tuple[int, float]]:
        """(index, time) of the last sample at or before t."""
        i = bisect.bisect_right(self.times, t) - 1
        return (i, self.times[i]) if i >= 0 else None

    def first_in(self, start: float, end: float) -> Optional[int]:
        """Index of the LAST sample in (start, end], or None."""
        i = bisect.bisect_right(self.times, end) - 1
        return i if i >= 0 and self.times[i] > start else None


@dataclass
class Frames:
    timestamp: np.ndarray        # (N,) seconds from the episode's t0
    state: np.ndarray            # (N, len(STATE_NAMES)) float, NaN = no data yet
    action: np.ndarray           # (N, 2) the label
    action_source: np.ndarray    # (N,) int8 index into SOURCES
    action_executed: np.ndarray  # (N, 2) /cmd_vel
    teleop: np.ndarray           # (N,) bool: a person held the wheels
    image_index: np.ndarray      # (N,) int: index into the image series, -1 = none
    image_age: np.ndarray        # (N,) seconds since that image's stamp
    t0: float


def _command(series: Series, start: float, end: float) -> Optional[Tuple[float, float]]:
    """The command given in (start, end], else the latest one still fresh at end."""
    i = series.first_in(start, end)
    if i is None:
        found = series.latest(end)
        if found is None or end - found[1] > FRESH_S:
            return None
        i = found[0]
    return series.values[i][:2]


def build_frames(series: Dict[str, Series], images: Series, fps: float, t0: float,
                 t_end: float) -> Frames:
    if fps <= 0 or t_end <= t0:
        raise ValueError("need fps > 0 and t_end > t0")
    dt = 1.0 / fps
    n = int(np.floor((t_end - t0) * fps)) + 1
    times = t0 + np.arange(n) * dt
    state = np.full((n, len(STATE_NAMES)), np.nan)
    action = np.zeros((n, 2))
    source = np.zeros(n, dtype=np.int8)
    executed = np.zeros((n, 2))
    teleop_held = np.zeros(n, dtype=bool)
    image_index = np.full(n, -1, dtype=np.int64)
    image_age = np.full(n, np.nan)
    teleop, nav = series.get("teleop", Series()), series.get("nav", Series())
    cmd = series.get("executed", Series())
    for k, t in enumerate(times):
        column = 0
        for name, width in STATE_SERIES:
            s = series.get(name)
            found = s.latest(t) if s is not None else None
            if found is not None:
                state[k, column:column + width] = s.values[found[0]][:width]
            column += width
        held = _command(teleop, t, t + dt)
        if held is not None:
            action[k], source[k], teleop_held[k] = held, 1, True
        else:
            proposed = _command(nav, t, t + dt)
            if proposed is not None:
                action[k], source[k] = proposed, 2
        done = _command(cmd, t, t + dt)
        if done is not None:
            executed[k] = done
        shot = images.latest(t)
        if shot is not None:
            image_index[k], image_age[k] = shot[0], t - shot[1]
    return Frames(times - t0, state, action, source, executed, teleop_held, image_index,
                  image_age, t0)


def marker_frames(markers: Sequence[Dict], ready_after_s: float, fps: float) -> List[Dict]:
    """episode.yaml's markers (t from the episode's start) as frame indices
    (frames start at READY)."""
    out = []
    for marker in markers:
        frame = int(round((marker["t"] - ready_after_s) * fps))
        out.append({"frame": max(0, frame), "label": marker["label"]})
    return out
