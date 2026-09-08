"""GazeboBackend's conversions and — the part that matters — its two SOURCES.

No Gazebo process, no ROS graph: the messages are built by hand and handed
straight to the callbacks. What is pinned here is the invariant A3's worst bug
violated, that the wheels and the world are read from two different plugins and
never leak into one another. test_gazebo_physics.py measures the gap between
them; this makes sure the gap can exist at all.
"""

import math
import threading

import pytest

from nav_msgs.msg import Odometry

from robot_sim.gazebo_backend import GazeboBackend


def make_backend():
    """A backend with no ROS node behind it — only the maths and the state."""
    backend = object.__new__(GazeboBackend)
    backend._radius = 0.0825
    backend._separation = 0.5
    backend._units_per_rpm = 1.0
    backend._lock = threading.Lock()
    backend._meas_l = 0.0
    backend._meas_r = 0.0
    backend.pose = type("Pose", (), {"x": 0.0, "y": 0.0, "yaw": 0.0})()
    backend.v = 0.0
    backend.omega = 0.0
    backend.accel_x = 0.0
    backend.collision = False
    backend._truth_seen = False
    backend._last_truth_t = None
    return backend


def odometry(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0, stamp=0.0):
    message = Odometry()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int(round((stamp % 1.0) * 1e9))
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw * 0.5)
    message.pose.pose.orientation.w = math.cos(yaw * 0.5)
    message.twist.twist.linear.x = v
    message.twist.twist.angular.z = omega
    return message


def test_wheel_units_round_trip_through_mps_conversion():
    backend = make_backend()
    speed = 0.75
    units = backend._mps_to_units(speed)
    assert backend._units_to_mps(units) == pytest.approx(speed)


def test_differential_wheel_units_have_expected_turn_rate():
    backend = make_backend()
    left = backend._units_to_mps(backend._mps_to_units(0.25))
    right = backend._units_to_mps(backend._mps_to_units(0.75))
    assert (right - left) / backend._separation == pytest.approx(1.0)
    assert math.isfinite(left)


def test_the_wheels_feed_the_halls_and_never_the_pose():
    """DiffDrive's odometry is the hall sensors' analogue, nothing more.

    Its POSE is the wheels' own idea of where the robot is — which is exactly
    what hoverboard_bridge exists to work out from these same numbers. Taking
    it from here would hand the bridge the answer instead of testing it, and it
    would put dead reckoning into `pose`, which is the answer key.
    """
    backend = make_backend()
    backend._on_wheel_odometry(odometry(x=7.0, y=3.0, yaw=1.0, v=0.5, omega=0.0))

    expected = backend._mps_to_units(0.5)
    assert backend._meas_l == pytest.approx(expected)
    assert backend._meas_r == pytest.approx(expected)
    # Untouched: the wheels do not get to say where the robot is.
    assert (backend.pose.x, backend.pose.y, backend.pose.yaw) == (0.0, 0.0, 0.0)
    assert backend.v == 0.0


def test_the_world_feeds_the_pose_and_never_the_halls():
    """And the mirror image, which is what makes slip observable.

    If ground truth also set the hall readings, /odom and /ground_truth would
    agree by construction and every slip number in test_gazebo_physics.py would
    be exactly zero — a green suite measuring nothing.
    """
    backend = make_backend()
    backend._on_ground_truth(
        odometry(x=2.0, y=-1.0, yaw=0.75, v=0.4, omega=0.2, stamp=1.0))

    assert backend.pose.x == pytest.approx(2.0)
    assert backend.pose.y == pytest.approx(-1.0)
    assert backend.pose.yaw == pytest.approx(0.75)
    assert backend.v == pytest.approx(0.4)
    assert backend.omega == pytest.approx(0.2)
    # Untouched: the world does not get to say what the hall sensors read.
    assert backend._meas_l == 0.0
    assert backend._meas_r == 0.0


def test_no_ground_truth_is_claimed_before_gazebo_has_sent_one():
    """Before the first message `pose` reads (0, 0, 0), which is a LIE that
    looks exactly like a robot sitting at the origin. sim_node publishes
    nothing at all until this turns True."""
    backend = make_backend()
    assert backend.ground_truth_seen is False
    backend._on_ground_truth(odometry(stamp=1.0))
    assert backend.ground_truth_seen is True


def test_acceleration_survives_a_stamp_that_does_not_advance():
    """Sim time repeats a stamp whenever /clock has not ticked between two
    publications. Dividing by that dt would put an inf into /imu/data_raw and
    take the EKF with it."""
    backend = make_backend()
    backend._on_ground_truth(odometry(v=0.0, stamp=1.0))
    backend._on_ground_truth(odometry(v=0.5, stamp=1.5))
    assert backend.accel_x == pytest.approx(1.0)

    backend._on_ground_truth(odometry(v=2.0, stamp=1.5))   # same stamp again
    assert backend.accel_x == pytest.approx(1.0)           # kept, not inf
    assert math.isfinite(backend.accel_x)


def test_a_contact_report_is_a_collision_and_an_empty_one_is_not():
    backend = make_backend()
    empty = type("Contacts", (), {"contacts": []})()
    touching = type("Contacts", (), {"contacts": [object()]})()

    backend._on_contacts(touching)
    assert backend.collision is True
    backend._on_contacts(empty)
    assert backend.collision is False
