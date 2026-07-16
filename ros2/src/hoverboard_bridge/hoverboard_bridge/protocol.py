"""Pi <-> ESP32 binary frame protocol.

This is the ROS 2 side of the link defined in firmware/esp32_bridge/src/main.cpp.
Keep the two files in sync — the structs here mirror `PiCommand` / `EspFeedback`
byte for byte.

Deliberately dependency-free (no rclpy): it can be unit tested on the dev machine
without ROS, and it is the piece we keep if we ever move to a ros2_control
hardware_interface (4WD upgrade path).

Frame layout (little-endian, __attribute__((packed)) => no alignment padding):

    PiCommand   (10 B)  uint16 start | int16 speed | int16 steer
                        | uint8 clear_estop | uint8 _pad | uint16 checksum
    EspFeedback (16 B)  uint16 start | int16 speedL_meas | int16 speedR_meas
                        | int16 batVoltage | int16 boardTemp
                        | uint8 estop | uint8 watchdog_ok
                        | uint8 bump | uint8 _pad2 | uint16 checksum

Checksum is XOR of every preceding 16-bit word, truncated to 16 bits. The uint8
fields are folded into words in pairs as (low | high << 8), exactly as the
firmware does it — which is why flags arrive two at a time, each with a pad.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional

START = 0xABCD
_START_LO = START & 0xFF          # 0xCD — arrives first (little-endian)
_START_HI = (START >> 8) & 0xFF   # 0xAB

PI_COMMAND_FMT = "<HhhBBH"
PI_COMMAND_SIZE = struct.calcsize(PI_COMMAND_FMT)      # 10

ESP_FEEDBACK_FMT = "<HhhhhBBBBH"
ESP_FEEDBACK_SIZE = struct.calcsize(ESP_FEEDBACK_FMT)  # 16

# Mirrors SPEED_LIMIT / STEER_LIMIT in the firmware. The ESP32 clamps anyway;
# clamping here too keeps struct.pack from raising on out-of-range ints.
SPEED_LIMIT = 300
STEER_LIMIT = 300  # noqa: F401  (re-exported for the simulator)


def _u16(value: int) -> int:
    """Two's-complement 16-bit view of an int, matching C's cast to uint16_t."""
    return value & 0xFFFF


def _clamp(value: int, limit: int) -> int:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class EspFeedback:
    """One decoded ESP32 -> Pi telemetry frame. Raw units, as sent."""

    speed_l: int       # left wheel, RPM as measured by the hoverboard's halls
    speed_r: int       # right wheel, RPM
    bat_voltage: int   # raw; scale to volts in the node (EFeru: 0.01 V/count)
    board_temp: int    # raw; scale to °C in the node (EFeru: 0.1 °C/count)
    estop: bool        # e-stop asserted and latched on the ESP32
    watchdog_ok: bool  # ESP32 considers our heartbeat fresh
    # Bumper hit: the ESP32 is vetoing forward motion right now. Not latched —
    # it clears itself when the switch releases, so reverse and turning still
    # work and the robot can back out on its own.
    bump: bool = False


def pack_pi_command(speed: int, steer: int, clear_estop: bool = False) -> bytes:
    """Build a PiCommand frame. Every frame doubles as the watchdog heartbeat."""
    speed = _clamp(int(speed), SPEED_LIMIT)
    steer = _clamp(int(steer), STEER_LIMIT)
    pad = 0
    payload2 = _u16((1 if clear_estop else 0) | (pad << 8))
    checksum = _u16(START ^ _u16(speed) ^ _u16(steer) ^ payload2)
    return struct.pack(
        PI_COMMAND_FMT, START, speed, steer, 1 if clear_estop else 0, pad, checksum
    )


def unpack_esp_feedback(raw: bytes) -> Optional[EspFeedback]:
    """Decode one EspFeedback frame. Returns None on a checksum mismatch."""
    if len(raw) != ESP_FEEDBACK_SIZE:
        return None
    (start, speed_l, speed_r, bat, temp, estop, watchdog, bump, pad2,
     checksum) = struct.unpack(ESP_FEEDBACK_FMT, raw)
    if start != START:
        return None
    expected = _u16(
        start
        ^ _u16(speed_l)
        ^ _u16(speed_r)
        ^ _u16(bat)
        ^ _u16(temp)
        ^ _u16(estop | (watchdog << 8))
        ^ _u16(bump | (pad2 << 8))
    )
    if expected != checksum:
        return None
    return EspFeedback(
        speed_l=speed_l,
        speed_r=speed_r,
        bat_voltage=bat,
        board_temp=temp,
        estop=bool(estop),
        watchdog_ok=bool(watchdog),
        bump=bool(bump),
    )


def pack_esp_feedback(fb: EspFeedback) -> bytes:
    """Encode an EspFeedback frame — used by the simulator and the tests."""
    payload2 = _u16(int(fb.estop) | (int(fb.watchdog_ok) << 8))
    payload3 = _u16(int(fb.bump))  # _pad2 = 0
    checksum = _u16(
        START
        ^ _u16(fb.speed_l)
        ^ _u16(fb.speed_r)
        ^ _u16(fb.bat_voltage)
        ^ _u16(fb.board_temp)
        ^ payload2
        ^ payload3
    )
    return struct.pack(
        ESP_FEEDBACK_FMT,
        START,
        fb.speed_l,
        fb.speed_r,
        fb.bat_voltage,
        fb.board_temp,
        int(fb.estop),
        int(fb.watchdog_ok),
        int(fb.bump),
        0,  # _pad2
        checksum,
    )


class FeedbackParser:
    """Byte-wise resynchronising frame parser (port of the firmware's).

    Never blocks and never desyncs permanently on a corrupt frame: it hunts for
    the 0xCD 0xAB marker, collects a fixed-size frame, then goes back to hunting.
    A garbled frame costs at most one frame, not the stream.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._prev: Optional[int] = None
        self._syncing = True
        self.checksum_errors = 0
        self.frames_ok = 0

    def feed(self, data: bytes) -> List[EspFeedback]:
        """Push received bytes; get back every complete, valid frame in them."""
        out: List[EspFeedback] = []
        for byte in data:
            if self._syncing:
                if self._prev == _START_LO and byte == _START_HI:
                    self._buf = bytearray((_START_LO, _START_HI))
                    self._syncing = False
                self._prev = byte
                continue

            self._buf.append(byte)
            if len(self._buf) == ESP_FEEDBACK_SIZE:
                frame = unpack_esp_feedback(bytes(self._buf))
                if frame is None:
                    self.checksum_errors += 1
                else:
                    self.frames_ok += 1
                    out.append(frame)
                self._syncing = True
                self._prev = None
        return out
