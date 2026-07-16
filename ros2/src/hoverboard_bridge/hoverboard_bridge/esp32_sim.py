"""The ESP32's brain, minus the wheels — shared by every simulator backend.

This is everything firmware/esp32_bridge/src/main.cpp does except turn a motor:
frame parsing, the watchdog, the latched e-stop, the bumper veto, the mixer, and
the feedback frame. What it does NOT decide is what the wheels actually do in
response — that is the backend's job:

    Esp32Sim  ──target_l, target_r──►  backend.step()  ──measured_l, measured_r──►
       ▲                                  │
       └────────── EspFeedback ───────────┘

    backend = LagBackend       fast, no world, no pose        (fake_esp32 CLI)
    backend = KinematicWorld   integrates a ground-truth pose (robot_sim)
    backend = GazeboBackend    real physics                   (planned, A3)

The point of the split: `hoverboard_bridge` and the 0xABCD protocol stay in the
loop in EVERY simulated world. If a backend spoke ROS directly and published
/odom itself, the bridge, the protocol, the watchdog and the bumper veto — real
code that runs on the real robot — would never be exercised.

Deliberately ROS-free, like protocol.py. The backend may import ROS; this must not.
"""

from __future__ import annotations

import os
import pty
import struct
import time
import tty
from typing import Optional, Protocol, Tuple

from hoverboard_bridge.protocol import (
    PI_COMMAND_FMT,
    PI_COMMAND_SIZE,
    SPEED_LIMIT,
    STEER_LIMIT,
    EspFeedback,
    pack_esp_feedback,
)

WATCHDOG_S = 0.200   # WATCHDOG_MS in the firmware
TX_PERIOD_S = 0.020  # TX_PERIOD_MS

# The two hub motors face opposite ways, so the right hall stream reads negative
# for forward travel. hoverboard_bridge's invert_right param undoes it. This is a
# property of the board's wiring, not of any simulated world, so it lives here.
RIGHT_HALL_INVERTED = True


class Backend(Protocol):
    """Turns commanded wheel units into measured wheel units."""

    def step(self, target_l: float, target_r: float, dt: float) -> Tuple[float, float]:
        ...


class LagBackend:
    """Wheels as a first-order lag. No world, no pose — just plausible dynamics.

    Enough to exercise the protocol and the safety paths, which is all the
    fake_esp32 CLI is for. robot_sim's KinematicWorld replaces this when the
    question is "where does the robot end up".
    """

    def __init__(self, tau: float = 0.09) -> None:
        self._tau = tau
        self._l = 0.0
        self._r = 0.0

    def step(self, target_l: float, target_r: float, dt: float) -> Tuple[float, float]:
        # Exponential approach, framed in dt so the behaviour does not change
        # with the loop rate.
        alpha = 1.0 - pow(2.718281828459045, -dt / self._tau) if self._tau > 0 else 1.0
        self._l += (target_l - self._l) * alpha
        self._r += (target_r - self._r) * alpha
        return self._l, self._r


class PtyLink:
    """A pseudo-terminal standing in for the ESP32's USB serial port."""

    def __init__(self, link_path: str) -> None:
        self._master, self._slave = pty.openpty()
        # Raw mode is not optional. A pty defaults to a terminal line discipline
        # with ECHO on, so every frame written to the master is echoed straight
        # back and parsed as a bogus PiCommand. Raw mode also stops the
        # discipline mangling binary bytes (\r\n translation, ^C as a signal).
        # A real USB serial port has none of this — the artefact is the
        # simulator's, not the robot's.
        tty.setraw(self._slave)
        tty.setraw(self._master)
        os.set_blocking(self._master, False)

        self.tty_path = os.ttyname(self._slave)
        self.link_path = link_path
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(self.tty_path, link_path)

    def read(self) -> bytes:
        try:
            return os.read(self._master, 4096)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> None:
        os.write(self._master, data)

    def close(self) -> None:
        if os.path.islink(self.link_path):
            os.remove(self.link_path)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


class Esp32Sim:
    """Protocol + safety state machine. Mirrors main.cpp; owns no wheels."""

    def __init__(
        self,
        estop: bool = False,
        bump: bool = False,
        bump_file: Optional[str] = None,
        bat_voltage: int = 3720,   # 37.2 V at 0.01 V/count
        board_temp: int = 305,     # 30.5 C at 0.1 C/count
    ) -> None:
        self.estop = estop
        self._bump_forced = bump
        self._bump_file = bump_file
        self.bump = bump
        self._bat_voltage = bat_voltage
        self._board_temp = board_temp

        self._buf = bytearray()
        self._prev: Optional[int] = None
        self._syncing = True
        self._last_cmd = 0.0
        self.cmd_speed = 0
        self.cmd_steer = 0

    # ---- Pi -> ESP32 ---------------------------------------------------------
    def feed(self, data: bytes, now: float) -> None:
        """Parse PiCommand frames. Mirrors the firmware's piReceive()."""
        for byte in data:
            if self._syncing:
                if self._prev == 0xCD and byte == 0xAB:
                    self._buf = bytearray((0xCD, 0xAB))
                    self._syncing = False
                self._prev = byte
                continue
            self._buf.append(byte)
            if len(self._buf) == PI_COMMAND_SIZE:
                _, speed, steer, clear, _pad, _chk = struct.unpack(
                    PI_COMMAND_FMT, bytes(self._buf)
                )
                self._last_cmd = now
                # The firmware constrains before the command ever reaches the
                # board; mirror it or the simulator is more permissive than the
                # hardware it stands in for.
                self.cmd_speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
                self.cmd_steer = max(-STEER_LIMIT, min(STEER_LIMIT, steer))
                if clear and self.estop:
                    self.estop = False
                self._syncing = True
                self._prev = None

    def watchdog_ok(self, now: float) -> bool:
        return (now - self._last_cmd) <= WATCHDOG_S

    def _poll_bump(self) -> bool:
        if self._bump_forced:
            return True
        return bool(self._bump_file) and os.path.exists(self._bump_file)

    # ---- ESP32 -> wheels -----------------------------------------------------
    def targets(self, now: float) -> Tuple[int, int]:
        """Wheel commands after safety and the mixer, in raw board units."""
        self.bump = self._poll_bump()
        driving = self.watchdog_ok(now) and not self.estop

        # Bumper veto, mirrored from the firmware's applyBumpVeto(): forward is
        # zeroed, reverse and steering pass untouched.
        out_speed = self.cmd_speed
        if self.bump and out_speed > 0:
            out_speed = 0

        if not driving:
            return 0, 0
        # Mixer, mirrored from the hoverboard firmware.
        return out_speed + self.cmd_steer, out_speed - self.cmd_steer

    # ---- ESP32 -> Pi ---------------------------------------------------------
    def feedback_frame(self, meas_l: float, meas_r: float, now: float) -> bytes:
        right = -meas_r if RIGHT_HALL_INVERTED else meas_r
        return pack_esp_feedback(
            EspFeedback(
                # round(), not int(): int() truncates toward zero, and a lag that
                # approaches its target from below then reads one unit low
                # forever. That is a systematic bias — small, always in the same
                # direction, and exactly the kind of thing that would quietly
                # skew an odometry-vs-ground-truth comparison.
                speed_l=round(meas_l),
                speed_r=round(right),
                bat_voltage=self._bat_voltage,
                board_temp=self._board_temp,
                estop=self.estop,
                watchdog_ok=self.watchdog_ok(now),
                bump=self.bump,
            )
        )


def step_once(sim: Esp32Sim, backend: Backend, link: PtyLink,
              now: float, dt: float) -> Tuple[float, float]:
    """One read -> decide -> actuate -> report cycle. Returns measured wheels."""
    sim.feed(link.read(), now)
    target_l, target_r = sim.targets(now)
    meas_l, meas_r = backend.step(target_l, target_r, dt)
    link.write(sim.feedback_frame(meas_l, meas_r, now))
    return meas_l, meas_r


def run_loop(sim: Esp32Sim, backend: Backend, link: PtyLink,
             on_step=None, poll_s: float = 0.002) -> None:
    """Drive the simulator at the firmware's TX rate until interrupted."""
    last_tx = 0.0
    try:
        while True:
            now = time.monotonic()
            if now - last_tx >= TX_PERIOD_S:
                dt = TX_PERIOD_S if last_tx == 0.0 else now - last_tx
                last_tx = now
                meas = step_once(sim, backend, link, now, dt)
                if on_step is not None:
                    on_step(now, dt, meas)
            else:
                # Between TX ticks, still drain the input so commands are not
                # delayed by up to a full period.
                sim.feed(link.read(), now)
            time.sleep(poll_s)
    except KeyboardInterrupt:
        pass
