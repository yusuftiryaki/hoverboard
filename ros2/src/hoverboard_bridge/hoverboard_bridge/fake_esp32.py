"""A pretend ESP32 on a pty, so the bridge can be driven without hardware.

The dev machine has no robot attached (docs/devcontainer.md). This creates a
pseudo-terminal that speaks the same frames as firmware/esp32_bridge and echoes
commanded speed back as "measured" wheel RPM through a first-order lag.

    ros2 run hoverboard_bridge fake_esp32
    ros2 run hoverboard_bridge hoverboard_bridge --ros-args -p port:=/tmp/fake_esp32

This is the PROTOCOL-level simulator: it has no world and no pose, so it answers
"do the frames, the watchdog and the safety vetoes work" and nothing else. For
"where does the robot end up", use robot_sim, which drives the same Esp32Sim
brain with a kinematic world behind it.

The bumper can be toggled while running, so the veto can be tested both ways:

    ros2 run hoverboard_bridge fake_esp32 --bump       # start bumped
    touch /tmp/fake_esp32.bump                          # assert it live
    rm /tmp/fake_esp32.bump                             # release it

It models the watchdog, the e-stop, the bumper veto and the feedback rate — NOT
the motors, the board, or any electrical failure mode. Passing here means the
framing and the safety logic are right; it says nothing about the real board.
"""

from __future__ import annotations

import argparse

from hoverboard_bridge.esp32_sim import Esp32Sim, LagBackend, PtyLink, run_loop


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

    link = PtyLink(args.link)
    print(f"fake ESP32 on {link.tty_path} (symlinked as {args.link})")
    print("point the node at it:  -p port:=" + args.link)

    sim = Esp32Sim(estop=args.estop, bump=args.bump, bump_file=args.bump_file)
    try:
        run_loop(sim, LagBackend(), link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
