"""A pretend ESP32 on a pty, so the bridge can be driven without hardware.

The dev machine has no robot attached (docs/devcontainer.md). This creates a
pseudo-terminal that speaks the same frames as firmware/esp32_bridge, prints a
symlink path to point the node at, and echoes commanded speed back as "measured"
wheel RPM through a crude first-order lag.

    ros2 run hoverboard_bridge fake_esp32
    ros2 run hoverboard_bridge hoverboard_bridge --ros-args -p port:=/tmp/fake_esp32

It models the watchdog, the bumper veto and the feedback rate — NOT the motors,
the board, or any of the electrical failure modes. Passing here means the framing
and the kinematics are right; it says nothing about the real board.

The bumper can be toggled while running, so the veto can be tested both ways:

    ros2 run hoverboard_bridge fake_esp32 --bump       # start bumped
    touch /tmp/fake_esp32.bump                          # assert it live
    rm /tmp/fake_esp32.bump                             # release it
"""

from __future__ import annotations

import argparse
import os
import pty
import struct
import time
import tty

from hoverboard_bridge.protocol import (
    ESP_FEEDBACK_SIZE,
    PI_COMMAND_FMT,
    PI_COMMAND_SIZE,
    SPEED_LIMIT,
    STEER_LIMIT,
    EspFeedback,
    pack_esp_feedback,
)

WATCHDOG_S = 0.200   # WATCHDOG_MS in the firmware
TX_PERIOD_S = 0.020  # TX_PERIOD_MS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--link", default="/tmp/fake_esp32", help="symlink to create")
    parser.add_argument("--estop", action="store_true", help="start with e-stop latched")
    parser.add_argument("--bump", action="store_true", help="start with the bumper hit")
    parser.add_argument(
        "--bump-file", default="/tmp/fake_esp32.bump",
        help="touch/rm this file to assert/release the bumper while running",
    )
    args = parser.parse_args()

    master, slave = pty.openpty()
    # Raw mode is not optional. A pty defaults to a terminal line discipline with
    # ECHO on, so every frame we write to the master gets echoed straight back at
    # us and parsed as a bogus PiCommand. Raw mode also stops the discipline from
    # mangling binary bytes (\r\n translation, ^C as a signal). A real USB serial
    # port has none of this — the artefact is the simulator's, not the robot's.
    tty.setraw(slave)
    tty.setraw(master)
    os.set_blocking(master, False)
    tty_path = os.ttyname(slave)
    if os.path.islink(args.link) or os.path.exists(args.link):
        os.remove(args.link)
    os.symlink(tty_path, args.link)
    print(f"fake ESP32 on {tty_path} (symlinked as {args.link})")
    print("point the node at it:  -p port:=" + args.link)

    buf = bytearray()
    prev = None
    syncing = True
    last_cmd = 0.0
    last_tx = 0.0
    cmd_speed = 0
    cmd_steer = 0
    meas_l = 0.0
    meas_r = 0.0
    estop = args.estop

    try:
        while True:
            try:
                data = os.read(master, 4096)
            except BlockingIOError:
                data = b""

            for byte in data:
                if syncing:
                    if prev == 0xCD and byte == 0xAB:
                        buf = bytearray((0xCD, 0xAB))
                        syncing = False
                    prev = byte
                    continue
                buf.append(byte)
                if len(buf) == PI_COMMAND_SIZE:
                    _, speed, steer, clear, _pad, _chk = struct.unpack(
                        PI_COMMAND_FMT, bytes(buf)
                    )
                    last_cmd = time.monotonic()
                    # The firmware constrains to SPEED_LIMIT/STEER_LIMIT before
                    # it ever reaches the board; mirror that or the simulator is
                    # more permissive than the hardware.
                    cmd_speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, speed))
                    cmd_steer = max(-STEER_LIMIT, min(STEER_LIMIT, steer))
                    if clear and estop:
                        estop = False
                        print("e-stop cleared")
                    syncing = True
                    prev = None

            now = time.monotonic()
            if now - last_tx >= TX_PERIOD_S:
                last_tx = now
                watchdog_ok = (now - last_cmd) <= WATCHDOG_S
                driving = watchdog_ok and not estop
                bump = args.bump or os.path.exists(args.bump_file)

                # Bumper veto, mirrored from the firmware's applyBumpVeto():
                # forward is zeroed, reverse and steering pass untouched.
                out_speed = cmd_speed
                if bump and out_speed > 0:
                    out_speed = 0

                # Mixer, mirrored from the hoverboard firmware.
                target_l = (out_speed + cmd_steer) if driving else 0
                target_r = (out_speed - cmd_steer) if driving else 0
                # First-order lag so the "wheels" do not step instantly.
                meas_l += (target_l - meas_l) * 0.2
                meas_r += (target_r - meas_r) * 0.2
                frame = pack_esp_feedback(
                    EspFeedback(
                        speed_l=int(meas_l),
                        speed_r=int(-meas_r),  # right hall stream is inverted
                        bat_voltage=3720,      # 37.2 V at 0.01 V/count
                        board_temp=305,        # 30.5 C at 0.1 C/count
                        estop=estop,
                        watchdog_ok=watchdog_ok,
                        bump=bump,
                    )
                )
                assert len(frame) == ESP_FEEDBACK_SIZE
                os.write(master, frame)

            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link):
            os.remove(args.link)


if __name__ == "__main__":
    main()
