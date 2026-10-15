"""From a requested (v, omega) to the (speed, steer) the board is sent — safely.

Deliberately ROS-free, like protocol.py: this is the last software between Nav2
and a 36 V robot, so it has to be testable in a bare interpreter.

Three rules, in order:

  1. A non-finite command means STOP. NaN or inf reaching int(round()) used to
     raise inside the bridge's timer and kill the node; the motors then stopped
     only because destroy_node() sent a zero and the ESP32 watchdog backed it
     up. Fail-stop by accident, and the drivetrain stayed dead until restart.

  2. Limits SCALE, they never clip one axis. Clipping v and omega (or speed and
     steer) separately changes the ratio between them, i.e. the curvature: the
     robot then drives a different arc from the one the controller planned.
     Scaling both by the same factor keeps the path and only slows down.

  3. The raw limit is applied per WHEEL, before the mixer. The firmware clamps
     speed and steer separately (main.cpp constrain), which is rule 2's
     distortion again; keeping |left| and |right| under the limit keeps both
     speed = (l + r) / 2 and steer = (l - r) / 2 under it too, so the firmware
     clamp never has to act.

And both functions are TOTAL: no input — a NaN, 1e308 that overflows halfway,
a limit of zero, a negative or NaN limit from a bad parameter file — can make
them raise or reverse the direction. Every one of those returns a stop. (The
bridge also refuses to start on such parameters; this is the second line.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class DriveGeometry:
    wheel_radius: float       # m
    wheel_separation: float   # m, centre to centre
    cmd_per_rpm: float        # raw board units per wheel RPM
    steer_sign: float         # +1 / -1, how the board's mixer is wired


def is_finite_command(v: float, w: float) -> bool:
    return math.isfinite(v) and math.isfinite(w)


def _usable_limit(limit: float) -> bool:
    # A negative limit would make the scale factor negative — forward becomes
    # full reverse. Zero would divide by zero. NaN silently disables the check.
    return math.isfinite(limit) and limit > 0.0


def limit_twist(v: float, w: float, max_v: float, max_w: float) -> Tuple[float, float]:
    """Scale (v, w) so |v| <= max_v and |w| <= max_w, keeping v / w.

    Non-finite input, or an unusable limit, returns (0, 0): see rule 1.
    """
    if not (is_finite_command(v, w) and _usable_limit(max_v) and _usable_limit(max_w)):
        return 0.0, 0.0
    scale = 1.0
    if abs(v) > max_v:
        scale = min(scale, max_v / abs(v))
    if abs(w) > max_w:
        scale = min(scale, max_w / abs(w))
    return v * scale, w * scale


def wheel_commands(v: float, w: float, geometry: DriveGeometry,
                   raw_limit: float) -> Tuple[int, int]:
    """(v, w) in SI -> (speed, steer) in raw board units, both wheels kept
    within raw_limit by a common scale factor.
    """
    if not (is_finite_command(v, w) and _usable_limit(raw_limit)):
        return 0, 0
    # Same operations in the same ORDER as the bridge before command_limits
    # existed (v / circumference * 60): reassociating them moves round() by one
    # count at exact .5 ties, and "in-limit commands are unchanged" is a claim
    # the tests pin exactly.
    half_track = 0.5 * geometry.wheel_separation
    circumference = 2.0 * math.pi * geometry.wheel_radius
    cmd_l = (v - w * half_track) / circumference * 60.0 * geometry.cmd_per_rpm
    cmd_r = (v + w * half_track) / circumference * 60.0 * geometry.cmd_per_rpm
    if not is_finite_command(cmd_l, cmd_r):     # finite in, overflowed halfway
        return 0, 0

    peak = max(abs(cmd_l), abs(cmd_r))
    if peak > raw_limit:
        cmd_l *= raw_limit / peak
        cmd_r *= raw_limit / peak

    # Undo the board's mixer: speedL = speed + steer, speedR = speed - steer.
    speed = (cmd_l + cmd_r) * 0.5
    steer = (cmd_l - cmd_r) * 0.5 * geometry.steer_sign
    if not is_finite_command(speed, steer):
        return 0, 0
    return int(round(speed)), int(round(steer))
