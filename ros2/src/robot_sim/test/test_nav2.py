"""Nav2 drives the robot to a goal — checked against ground truth.

Scope, deliberately narrow: this tests the NAVIGATION layer (planner, controller,
costmaps, and the whole /cmd_vel -> protocol -> wheels chain), NOT GPS
localization. It pins map to odom with a static identity transform and runs with
the GPS off.

Why not test the real thing, GPS waypoints? Because it does not work yet, and the
reason is not Nav2's:

  navsat_transform reads the robot's absolute heading from /imu/data's
  orientation. Our 6-axis IMU has none and says so (orientation_covariance[0] =
  -1); navsat_transform does not check that flag, reads the identity quaternion
  and concludes "facing east". Meanwhile ekf_global's yaw is unobservable — no
  absolute heading anywhere — so GPS position updates yank it around. Measured:
  ground truth +38.8 deg while ekf_global claimed -178 deg, then -47 deg. Since
  ekf_global publishes map->odom, a wrong yaw there rotates every Nav2 goal.

  The fix is the QMC5883L (handoff decision 4, roadmap A4/B6), not a Nav2 param.

A GPS waypoint test belongs here once the magnetometer exists. Writing one now
would mostly assert that the robot starts facing east, which robot_sim
guarantees and reality does not.

Spawns processes, ~90 s. SKIPs without ROS.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                        # noqa: E402
from geometry_msgs.msg import PoseStamped           # noqa: E402
from nav2_msgs.action import NavigateToPose         # noqa: E402
from nav_msgs.msg import Odometry                   # noqa: E402
from rclpy.action import ActionClient               # noqa: E402
from rclpy.node import Node                         # noqa: E402

GOAL_X, GOAL_Y = 5.0, 2.0
# The goal checker stops at xy_goal_tolerance (1.0 m), so the robot legitimately
# halts up to a metre out. Allow that plus room for the EKF's own error.
ARRIVAL_TOLERANCE = 2.0


class Navigator(Node):
    def __init__(self):
        super().__init__("nav2_test_client")
        self.truth = None
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def position(self):
        p = self.truth.pose.pose.position
        return p.x, p.y

    def go(self, x, y, timeout=120.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        assert self.client.wait_for_server(timeout_sec=20.0), \
            "navigate_to_pose action server never came up"
        send = self.client.send_goal_async(goal)
        while not send.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send.result()
        assert handle.accepted, "Nav2 rejected the goal"

        result = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert result.done(), f"Nav2 did not finish within {timeout} s"
        return result.result().status


@pytest.fixture
def navigator(ros):
    node = Navigator()
    yield node
    node.destroy_node()


def test_nav2_drives_the_robot_to_a_goal(nav2_stack, navigator):
    nav2_stack()
    navigator.spin(3.0)
    assert navigator.truth is not None, "no /ground_truth — is sim_node up?"
    start = navigator.position

    status = navigator.go(GOAL_X, GOAL_Y)
    navigator.spin(1.0)
    x, y = navigator.position
    error = math.hypot(x - GOAL_X, y - GOAL_Y)

    # 4 == STATUS_SUCCEEDED. Check the ground truth too: a confidently wrong
    # filter would report a triumphant arrival from inside a hedge.
    assert status == 4, f"Nav2 reported status {status} (4 = SUCCEEDED)"
    assert error < ARRIVAL_TOLERANCE, (
        f"Nav2 says it arrived, but the robot is really {error:.2f} m from the "
        f"goal (started at {start}, ended at ({x:.2f}, {y:.2f}))")
