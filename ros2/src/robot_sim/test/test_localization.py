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
        self.cmd = (0.0, 0.0)
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        self.create_subscription(Odometry, "/odometry/filtered/local",
                                 lambda m: setattr(self, "ekf", m), 10)
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
