"""Driving from the web page (docs/missions.md, 5a). ROS-free; web_ui ticks it.

The page sends a command at ~10 Hz while someone is driving — zero when the
stick is let go — and web_ui publishes what `output()` says at 20 Hz on
/cmd_vel_teleop, twist_mux's highest-priority input:

    since the last command   output()
    < deadman_s              the command, scaled to max_linear / max_angular
    < lease_s                ZERO — a Wi-Fi hiccup stops the robot now, not
                             after the bridge's own 0.5 s deadman
    beyond                   None (publish nothing) — the lease is gone, the
                             mux hands the wheels back to Nav2

The page sends RATIOS in [-1, 1], never m/s: whatever it sends, the robot
cannot be driven faster than this side's limits. Everything below this node
(mux timeout, bridge cmd_timeout, the ESP32 watchdog, E-stop, the bumper veto)
is untouched by it.

One driver at a time: there is no login, so `acquire()` hands out a token and
a command without the current token is refused. A tab that goes away loses
the lease after lease_s, so a closed laptop never keeps the robot locked.

The clock is injected (monotonic by default) so the tests can step it; the
deadman is about the browser, which lives in wall time, never sim time.
"""

from __future__ import annotations

import math
import secrets
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple


class TeleopError(ValueError):
    """A malformed command (400)."""


class TeleopConflict(RuntimeError):
    """Someone else holds the lease, or this token's lease is gone (409)."""


def _ratio(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TeleopError(f"{what} must be a finite number in [-1, 1], got {value!r}")
    return max(-1.0, min(1.0, float(value)))


class TeleopGate:
    def __init__(self, max_linear: float = 0.6, max_angular: float = 1.0,
                 deadman_s: float = 0.3, lease_s: float = 2.0,
                 clock: Callable[[], float] = time.monotonic):
        if not (max_linear > 0 and max_angular > 0):
            raise ValueError("speed limits must be positive")
        if not 0 < deadman_s < lease_s:
            raise ValueError("need 0 < deadman_s < lease_s")
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.deadman_s = float(deadman_s)
        self.lease_s = float(lease_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._token: Optional[str] = None
        self._heard = 0.0                     # last acquire or command
        self._command: Optional[Tuple[float, float]] = None
        self._commanded = 0.0                 # when _command arrived

    def limits(self) -> Dict[str, float]:
        return {"max_linear": self.max_linear, "max_angular": self.max_angular,
                "deadman_s": self.deadman_s, "lease_s": self.lease_s}

    def _expire(self, now: float) -> None:
        if self._token is not None and now - self._heard > self.lease_s:
            self._token, self._command = None, None

    def acquire(self) -> str:
        with self._lock:
            now = self._clock()
            self._expire(now)
            if self._token is not None:
                raise TeleopConflict("someone else is driving (their lease ends "
                                     f"{self.lease_s:g} s after their last command)")
            self._token = secrets.token_hex(8)
            self._heard, self._command = now, None
            return self._token

    def command(self, token: Any, linear: Any, angular: Any) -> None:
        lin, ang = _ratio(linear, "linear"), _ratio(angular, "angular")
        with self._lock:
            now = self._clock()
            self._expire(now)
            if self._token is None or token != self._token:
                raise TeleopConflict("not driving: acquire first (the lease may have expired)")
            self._heard = self._commanded = now
            self._command = (lin * self.max_linear, ang * self.max_angular)

    def release(self, token: Any) -> bool:
        with self._lock:
            if self._token is not None and token == self._token:
                self._token, self._command = None, None
                return True
            return False

    def output(self) -> Optional[Tuple[float, float]]:
        """(linear m/s, angular rad/s) to publish now, or None for nothing."""
        with self._lock:
            now = self._clock()
            self._expire(now)
            if self._token is None:
                return None
            if self._command is None or now - self._commanded >= self.deadman_s:
                return 0.0, 0.0
            return self._command

    def state(self) -> Dict[str, Any]:
        out = self.output()
        with self._lock:
            age = None if self._command is None else round(self._clock() - self._commanded, 2)
        return {"held": out is not None,
                "active": out is not None and out != (0.0, 0.0),
                "linear": None if out is None else out[0],
                "angular": None if out is None else out[1],
                "command_age_s": age, **self.limits()}
