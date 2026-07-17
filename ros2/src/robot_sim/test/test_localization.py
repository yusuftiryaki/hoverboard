"""How wrong is the EKF? The question the real robot can never answer.

On hardware there is no ground truth: you can ask whether the estimate looks
smooth, never whether it is right. Here the world knows where the robot actually
is, so "the localization works" becomes a number.

Thresholds come from measured runs, with room to spare — they are regression
guards, not accuracy targets. They caught a real bug already: the fake IMU
originally published the chip's raw gyro bias, which nothing removed (the real
mpu6050_driver calibrates it out), and the estimate drifted 100 degrees.

⚠️ Passing this does NOT mean the robot will localize well outdoors. The world
here is pure kinematics: no wheel slip, which is the single largest source of
real odometry error. This proves the maths is wired up correctly and nothing
more. Slip needs A3 (Gazebo); the truth needs dirt.

Spawns processes, ~60 s. SKIPs without ROS.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                            # noqa: E402
from geometry_msgs.msg import Twist     # noqa: E402
from nav_msgs.msg import Odometry       # noqa: E402
from rclpy.node import Node             # noqa: E402
from sensor_msgs.msg import Imu         # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Probe(Node):
    def __init__(self):
        super().__init__("localization_probe")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.truth = None
        self.ekf = None
        self.ekf_global = None
        self.imu = None
        self.cmd = (0.0, 0.0)
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        self.create_subscription(Odometry, "/odometry/filtered/local",
                                 lambda m: setattr(self, "ekf", m), 10)
        # Only published when the global half is up (GlobalStack).
        self.create_subscription(Odometry, "/odometry/filtered/global",
                                 lambda m: setattr(self, "ekf_global", m), 10)
        self.create_subscription(Imu, "/imu/data",
                                 lambda m: setattr(self, "imu", m), 10)
        self.create_timer(0.05, self._tick)

    def _tick(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.cmd
        self._pub.publish(msg)

    def drive(self, v, w, seconds):
        self.cmd = (v, w)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    @property
    def position_error(self):
        t, e = self.truth.pose.pose.position, self.ekf.pose.pose.position
        return math.hypot(t.x - e.x, t.y - e.y)

    @property
    def yaw_error(self):
        return angle_diff(yaw_of(self.truth.pose.pose.orientation),
                          yaw_of(self.ekf.pose.pose.orientation))

    @property
    def true_yaw(self):
        return yaw_of(self.truth.pose.pose.orientation)

    def absolute_yaw_error(self, msg):
        """How far an ABSOLUTE heading estimate is from the truth, in degrees."""
        return math.degrees(angle_diff(self.true_yaw, yaw_of(msg)))


@pytest.fixture
def probe(ros):
    node = Probe()
    yield node
    node.destroy_node()


def test_both_ends_of_the_stack_are_alive(sim_stack, probe):
    sim_stack()
    probe.drive(0.0, 0.0, 3.0)
    assert probe.truth is not None, "sim_node is not publishing /ground_truth"
    assert probe.ekf is not None, "the EKF is not publishing — is the bridge connected?"


def test_ekf_tracks_the_truth_around_a_square(sim_stack, probe):
    """~8 m of driving with four turns. Measured: ~0.08 m and ~4 deg of error."""
    sim_stack()
    probe.drive(0.0, 0.0, 2.0)
    for _ in range(4):
        probe.drive(0.4, 0.0, 5.0)      # ~2 m straight
        probe.drive(0.0, 0.5, 3.14)     # ~90 deg turn
    probe.drive(0.0, 0.0, 1.0)

    assert probe.truth is not None and probe.ekf is not None
    # Generous vs the ~0.08 m measured: this guards against a broken filter, not
    # against a slightly worse one.
    assert probe.position_error < 0.4, (
        f"EKF is {probe.position_error:.2f} m from the truth after ~8 m")
    assert math.degrees(probe.yaw_error) < 15.0, (
        f"EKF is {math.degrees(probe.yaw_error):.1f} deg off after ~8 m")


def test_yaw_drifts_without_a_magnetometer(sim_stack, probe):
    """Documents the gap the magnetometer exists to close.

    With a 6-axis IMU nothing observes absolute heading, so any residual gyro
    bias integrates forever. This asserts the drift is BOUNDED at a realistic
    residual — and its existence is why handoff decision 4 calls the QMC5883L
    the highest-value purchase on the list. When the magnetometer lands and
    ekf_global gains an absolute yaw, this test should get tighter.
    """
    # 1 deg/s residual: ten times worse than a good calibration leaves, so the
    # drift is unmistakable within a short run.
    sim_stack("--ros-args", "-p", "imu_gyro_residual_bias_dps:=1.0")
    probe.drive(0.0, 0.0, 2.0)
    probe.drive(0.3, 0.0, 20.0)

    drift_deg = math.degrees(probe.yaw_error)
    # 1 deg/s over ~22 s of driving, and the wheels disagree with the gyro, so
    # the EKF lands somewhere between. Assert the shape, not a precise value:
    # clearly drifting, but not wildly diverging.
    assert 2.0 < drift_deg < 30.0, (
        f"expected visible bounded yaw drift, got {drift_deg:.1f} deg")


def test_absolute_yaw_holds_through_a_full_turn(global_stack, probe):
    """The magnetometer's whole job, measured at every heading.

    ⚠️ SPIN THE ROBOT — do not test one heading. This is the test that was
    missing, and its absence cost a session: sim_node and qmc5883l's fake_bus
    both had x = -B sin(yaw) instead of +B sin(yaw), mirroring the simulated
    earth, and the driver's unit tests inverted the same sign right back so they
    agreed with each other. Every heading test started the robot at yaw 0, where
    sin(0) = 0 and the mirrored and correct fields are IDENTICAL. The suite was
    green while the compass ran backwards.

    Downstream it was anything but subtle: the mag told madgwick the robot was
    turning the wrong way, madgwick split the difference with the gyro (measured:
    it tracked 0.897 of the true rotation), ekf_global's yaw came out ~120 deg
    off, that landed in map->odom, and Nav2 sent the robot 40 m the wrong way
    while looking like a broken controller.

    A full turn is what makes any of that visible. A mirror, a 90 deg mounting
    offset and a wrong declination all look perfect at exactly one heading.
    """
    global_stack()
    probe.drive(0.0, 0.0, 4.0)

    assert probe.imu is not None, (
        "no /imu/data — imu_filter_madgwick is not running or has no /imu/mag")
    assert probe.ekf_global is not None, (
        "no /odometry/filtered/global — ekf_global is not running")

    worst_madgwick = 0.0
    worst_ekf = 0.0
    # ~0.35 rad/s for 20 s: a bit over one full revolution, sampled throughout.
    end = time.monotonic() + 20.0
    probe.cmd = (0.0, 0.35)
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.02)
        worst_madgwick = max(worst_madgwick, probe.absolute_yaw_error(probe.imu.orientation))
        worst_ekf = max(worst_ekf, probe.absolute_yaw_error(probe.ekf_global.pose.pose.orientation))
    probe.drive(0.0, 0.0, 1.0)

    # Measured after the fix: madgwick within ~2 deg at every heading, ekf_global
    # the same but for brief transients when a GPS update lands. Mirrored, these
    # ran to 88 and 120. The thresholds are loose enough to survive noise and
    # nowhere near loose enough to survive a sign error.
    assert worst_madgwick < 15.0, (
        f"madgwick's fused yaw is {worst_madgwick:.1f} deg off at some heading — "
        "suspect the magnetometer's sign convention or its mounting")
    assert worst_ekf < 25.0, (
        f"ekf_global's yaw is {worst_ekf:.1f} deg off at some heading; "
        "map->odom carries this straight into every Nav2 goal")
