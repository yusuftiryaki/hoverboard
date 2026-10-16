"""twist_mux, exactly as robot.launch.py wires it: teleop outranks Nav2.

    /cmd_vel_teleop (100) ─┐
    /cmd_vel_nav_gated (10) ┴─► twist_mux ─► /cmd_vel

Starts robot_bringup's twist_mux.launch.py on its own — the remap and the
config file together, no bridge and no world — and plays both inputs at it.
Nav2's end of the claim (a human taking over a LIVE goal, and the goal resuming)
is in test_nav2.py; this file pins the mux's own behaviour, in seconds.

Spawns one process, ~10 s. SKIPs without ROS.
"""

import os
import signal
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                        # noqa: E402
from geometry_msgs.msg import Twist                 # noqa: E402
from rclpy.node import Node                         # noqa: E402

MUX_PATTERN = "lib/twist_mux/twist_mux"
RATE_HZ = 20.0
# config/twist_mux.yaml's teleop timeout.
TELEOP_TIMEOUT_S = 0.5

NAV_SPEED = 0.3
TELEOP_SPEED = -0.2


class MuxPlayer(Node):
    """Publishes both mux inputs and records everything the mux lets through."""

    def __init__(self):
        super().__init__("twist_mux_test_player")
        self.teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.nav = self.create_publisher(Twist, "/cmd_vel_nav_gated", 10)
        self.received = []          # (monotonic time, linear.x)
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda m: self.received.append((time.monotonic(), m.linear.x)), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def play(self, seconds, nav=None, teleop=None):
        """Publish the given speeds at RATE_HZ for `seconds` (None = silent).

        Returns the monotonic time of the last teleop message sent."""
        last_teleop = None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if nav is not None:
                self.nav.publish(Twist(linear=_vec(nav)))
            if teleop is not None:
                self.teleop.publish(Twist(linear=_vec(teleop)))
                last_teleop = time.monotonic()
            self.spin_for(1.0 / RATE_HZ)
        return last_teleop

    def wait_for_mux(self, timeout=20.0):
        """Until the mux subscribes to both inputs and publishes /cmd_vel."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.count_subscribers("/cmd_vel_teleop") and
                    self.count_subscribers("/cmd_vel_nav_gated") and
                    self.count_publishers("/cmd_vel")):
                # Matching is per endpoint and asynchronous; let it settle so
                # the first message is not dropped by a half-made connection.
                self.spin_for(0.5)
                return
            self.spin_for(0.1)
        raise AssertionError(
            "twist_mux never subscribed to both inputs and published /cmd_vel "
            f"(teleop subs {self.count_subscribers('/cmd_vel_teleop')}, nav subs "
            f"{self.count_subscribers('/cmd_vel_nav_gated')}, cmd_vel pubs "
            f"{self.count_publishers('/cmd_vel')}). If the counts are non-zero "
            "but the TYPES differ, use_stamped is missing from twist_mux.yaml.")


def pids_matching(pattern):
    found = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    return found.stdout.split()


def _vec(x):
    from geometry_msgs.msg import Vector3
    return Vector3(x=float(x))


@pytest.fixture(scope="module")
def mux(ros):
    stray = pids_matching(MUX_PATTERN)
    assert not stray, f"a twist_mux is already running (pids {stray})"
    proc = subprocess.Popen(
        ["ros2", "launch", "robot_bringup", "twist_mux.launch.py"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    try:
        yield proc
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=15)
        deadline = time.monotonic() + 15.0
        while pids_matching(MUX_PATTERN) and time.monotonic() < deadline:
            time.sleep(0.25)


@pytest.fixture
def player(mux):
    node = MuxPlayer()
    node.wait_for_mux()
    # Each test starts from a mux whose inputs have all timed out.
    node.spin_for(2 * TELEOP_TIMEOUT_S)
    node.received.clear()
    yield node
    node.destroy_node()


def test_nav2_alone_reaches_cmd_vel_as_plain_twist(player):
    """The use_stamped trap: twist_mux 4.5 defaults to TwistStamped, and a
    type mismatch forwards NOTHING without a single error."""
    types = {t for i in player.get_publishers_info_by_topic("/cmd_vel")
             for t in [i.topic_type]}
    assert types == {"geometry_msgs/msg/Twist"}, (
        f"/cmd_vel is published as {types}; the bridge subscribes plain Twist. "
        "Is use_stamped: false still in twist_mux.yaml?")

    player.play(1.0, nav=NAV_SPEED)
    player.spin_for(0.2)
    speeds = [x for _, x in player.received]
    assert len(speeds) >= 10, (
        f"only {len(speeds)} Nav2 commands reached /cmd_vel in 1 s at "
        f"{RATE_HZ:g} Hz — the mux is not forwarding its Nav2 input")
    assert set(speeds) == {NAV_SPEED}


def test_teleop_takes_the_wheels_from_a_running_nav2(player):
    """Both inputs live at once; from teleop's first message on, NOTHING of
    Nav2's may get through. Interleaving is the race the mux exists to end."""
    player.play(0.5, nav=NAV_SPEED)
    player.play(1.5, nav=NAV_SPEED, teleop=TELEOP_SPEED)
    player.spin_for(0.2)

    speeds = [x for _, x in player.received]
    assert TELEOP_SPEED in speeds, "teleop never reached /cmd_vel"
    after_takeover = speeds[speeds.index(TELEOP_SPEED):]
    leaked = after_takeover.count(NAV_SPEED)
    assert leaked == 0, (
        f"{leaked} of Nav2's commands were interleaved with teleop's after the "
        f"human took over: {after_takeover}")


def test_a_teleop_zero_holds_the_robot_against_nav2(player):
    """A human STOPPING the robot is the case that matters most: a zero from
    teleop is a command, not an absence of one, and must win too."""
    player.play(0.5, nav=NAV_SPEED)
    player.play(1.5, nav=NAV_SPEED, teleop=0.0)
    player.spin_for(0.2)

    speeds = [x for _, x in player.received]
    assert 0.0 in speeds, "teleop's zero never reached /cmd_vel"
    after_takeover = speeds[speeds.index(0.0):]
    assert set(after_takeover) == {0.0}, (
        f"Nav2 drove through a human's stop command: {after_takeover}")


def test_nav2_gets_the_wheels_back_once_teleop_goes_quiet(player):
    """Control returns after the teleop timeout — not on the first key-repeat
    gap, and not never."""
    last_teleop = player.play(1.0, nav=NAV_SPEED, teleop=TELEOP_SPEED)
    player.play(2.0, nav=NAV_SPEED)

    resumed = [t for t, x in player.received if x == NAV_SPEED and t > last_teleop]
    assert resumed, "Nav2 never got the wheels back after teleop went quiet"
    gap = resumed[0] - last_teleop
    # One Nav2 period of slack either side of the configured timeout.
    slack = 1.0 / RATE_HZ + 0.05
    assert TELEOP_TIMEOUT_S - slack <= gap <= TELEOP_TIMEOUT_S + 2 * slack, (
        f"Nav2 resumed {gap:.3f} s after teleop's last message; the mux's "
        f"teleop timeout is {TELEOP_TIMEOUT_S} s")
