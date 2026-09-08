"""The safety paths, end to end: real bridge node against a simulated ESP32.

These exist because the motor command path is the one place in this repo where a
bug moves 15 kg of hoverboard under a 36 V battery. The unit tests prove the
frames are right; only these prove that /cmd_vel actually stops when it must.

They spawn processes and take ~30 s. They SKIP without ROS on the path.

    cd ros2 && source install/setup.bash
    python3 -m pytest src/hoverboard_bridge/test/test_bridge_integration.py -q
"""

import os
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                    # noqa: E402
from geometry_msgs.msg import Twist             # noqa: E402
from nav_msgs.msg import Odometry               # noqa: E402
from rclpy.node import Node                     # noqa: E402
from std_msgs.msg import Bool                   # noqa: E402
from std_srvs.srv import Trigger                # noqa: E402

# The simulated wheels report integer RPM, so a 0.4 m/s command comes back as
# ~0.397. Assert on "is it driving" / "is it stopped", never on exact speed.
DRIVING = 0.3
STOPPED = 0.02


class Probe(Node):
    def __init__(self):
        super().__init__("integration_probe")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vx = None
        self.bumper = None
        self.diag = None
        self.cmd = 0.0
        self.sending = True
        self.create_subscription(
            Odometry, "/odom", lambda m: setattr(self, "vx", m.twist.twist.linear.x), 10)
        self.create_subscription(
            Bool, "/bumper", lambda m: setattr(self, "bumper", m.data), 10)
        self.create_timer(0.05, self._tick)

    def _tick(self):
        if self.sending:
            msg = Twist()
            msg.linear.x = self.cmd
            self._pub.publish(msg)

    def drive(self, vx, seconds=3.0):
        """Command a speed for a while; return the measured speed at the end."""
        self.cmd = vx
        self.settle(seconds)
        return self.vx

    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.destroy_node()


def test_bumper_veto_is_directional(harness, probe, bump_file):
    """Forward blocked, reverse open. The whole safety claim of the bumper.

    A veto that blocks everything strands the robot against whatever it touched;
    a veto on the wrong sign drives it harder into it. Both look identical from
    the outside unless each direction is checked separately.
    """
    harness()
    probe.settle(1.0)

    assert probe.drive(+0.4) > DRIVING, "baseline: forward should drive"
    assert probe.drive(-0.4) < -DRIVING, "baseline: reverse should drive"
    assert probe.bumper is False

    open(bump_file, "w").close()          # hit something
    probe.drive(0.0, 1.0)
    assert probe.bumper is True, "the hit must reach ROS"

    assert abs(probe.drive(+0.4)) < STOPPED, "FORWARD MUST BE VETOED"
    assert probe.drive(-0.4) < -DRIVING, "reverse must stay open — the escape route"

    os.remove(bump_file)                  # backed off
    assert probe.drive(+0.4) > DRIVING, "forward resumes with no handshake"
    assert probe.bumper is False


def test_estop_blocks_both_directions_then_clears(harness, probe):
    """E-stop is a full stop, not a veto — and stays that way until cleared.

    Guards the gate ordering: the bumper veto only ever touches forward, so if
    reverse escaped under e-stop, the two had been wired up the wrong way round.
    """
    harness("--estop")
    probe.settle(1.0)

    assert abs(probe.drive(+0.4)) < STOPPED, "e-stop must block forward"
    assert abs(probe.drive(-0.4)) < STOPPED, "e-stop must block REVERSE too"

    client = probe.create_client(Trigger, "/hoverboard_bridge/clear_estop")
    assert client.wait_for_service(timeout_sec=5.0), "clear_estop service missing"
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 10
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    assert future.done(), "clear_estop never answered"

    assert probe.drive(+0.4) > DRIVING, "cleared e-stop must resume driving"


def test_watchdog_stops_the_wheels_when_cmd_vel_stops(harness, probe):
    """Silence must stop the robot. The one guarantee that survives a Pi crash."""
    harness()
    probe.settle(1.0)
    assert probe.drive(+0.4) > DRIVING

    probe.sending = False                 # the Pi "dies"
    probe.settle(3.0)
    assert abs(probe.vx) < STOPPED, "no /cmd_vel must mean no motion"


def test_sim_time_without_a_clock_silences_the_bridge_instead_of_freezing_it(ros):
    """The assumption robot.launch.py's use_sim_time warning rests on.

    ⚠️ This pins a PLATFORM behaviour, not our own code, and it is here because
    the safety note about it was once written from reasoning rather than
    measurement — and the reasoning was wrong. The note claimed that on sim time
    with no /clock the bridge's `age = now - last_cmd_time` would be
    permanently zero, so `cmd_timeout` would never trip and the last /cmd_vel
    would be resent forever after its publisher died. That would be a runaway.

    Measured instead: rclpy does not run a clock-driven timer at all while ROS
    time is frozen. `_tx_tick` never fires, so the bridge sends nothing, and the
    ESP32's own 200 ms watchdog stops the wheels. It fails SAFE — but silently
    and completely: no commands, no odometry, no diagnostics, no log line. Which
    is still a good enough reason to keep use_sim_time false on the robot.

    If a future rclpy runs frozen-clock timers after all, this test fails, and
    the warning it guards has to become a real runtime guard in bridge_node.
    """
    node = rclpy.create_node(
        "frozen_clock_probe",
        parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
    )
    fires = []
    node.create_timer(0.05, lambda: fires.append(node.get_clock().now().nanoseconds))
    try:
        # 80 periods' worth of wall time. Nobody publishes /clock in this test.
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not fires, (
            f"a clock-driven timer fired {len(fires)} times on a frozen sim "
            "clock. hoverboard_bridge's _tx_tick would then run with a "
            "permanently zero command age, cmd_timeout would never trip, and a "
            "dead /cmd_vel publisher would leave the last command driving the "
            "wheels. That needs a real guard in bridge_node, not a comment.")
    finally:
        node.destroy_node()
