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
from std_msgs.msg import Bool                       # noqa: E402

GOAL_X, GOAL_Y = 5.0, 2.0
# The goal checker stops at xy_goal_tolerance (1.0 m), so the robot legitimately
# halts up to a metre out. Allow that plus room for the EKF's own error.
ARRIVAL_TOLERANCE = 2.0

# ---- A3c: a wall the robot has to go around ------------------------------
# Overlapping circles, because circles are all KinematicWorld collides against.
# Spacing 0.3 with radius 0.3 leaves no gap to squeeze through, and the wall
# spans y in [-1.2, 1.2] once the radii are counted.
WALL_X = 3.0
WALL_YS = (-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9)
WALL_RADIUS = 0.3
WALL_GOAL_X, WALL_GOAL_Y = 6.0, 0.0

# Physics stops the robot at robot_radius + obstacle radius = 0.4 + 0.3.
# The costmap's inflation (robot_radius 0.4, then a further 0.15 of margin)
# makes the planner refuse the same circle, so the two agree by construction —
# see world.py's robot_radius note.
CONTACT_DISTANCE = 0.4 + WALL_RADIUS

# Going around means clearing the wall's tip at y = 1.2 by at least a contact
# distance. Anything under this is the robot trying to bore through.
DETOUR_Y = 1.2
# With no wall in the way NavFn plans a straight line and RPP holds it; the
# handoff measured ~0.08 m of position error over 8 m. Half a metre is a wide
# margin for that and still nowhere near DETOUR_Y.
STRAIGHT_LINE_Y = 0.5


def _as_list(values):
    """ROS CLI syntax for a double array. Every value needs a decimal point:
    `3` would arrive as an integer and rclpy rejects it against DOUBLE_ARRAY."""
    return "[" + ",".join(f"{float(v)}" for v in values) + "]"


def unsurveyed_sim_args():
    """A tree nobody entered into the survey, sitting on the straight line.

    Physically present, absent from /obstacle_map — so the costmaps never learn
    about it and Nav2 plans straight through. Same shape as the surveyed wall,
    so the two tests differ in exactly one thing: whether the map knows.
    """
    centers = [value for y in WALL_YS for value in (WALL_X, y)]
    radii = [WALL_RADIUS] * len(WALL_YS)
    return ("--ros-args",
            "-p", f"unsurveyed_obstacle_centers:={_as_list(centers)}",
            "-p", f"unsurveyed_obstacle_radii:={_as_list(radii)}")


def wall_sim_args():
    """sim_node CLI args placing the wall, SURVEYED: in the world and on the
    published map both, so Nav2 can plan around it."""
    centers = [value for y in WALL_YS for value in (WALL_X, y)]
    radii = [WALL_RADIUS] * len(WALL_YS)
    return ("--ros-args",
            "-p", f"obstacle_centers:={_as_list(centers)}",
            "-p", f"obstacle_radii:={_as_list(radii)}")


class Navigator(Node):
    def __init__(self):
        super().__init__("nav2_test_client")
        self.truth = None
        # The whole path, not just the latest pose: "did it arrive?" and "did it
        # go AROUND?" are different questions, and only the second one can tell
        # a detour from a robot that drove through the wall and out the far side.
        self.path = []
        # The simulator's collision oracle — nothing on the real robot can
        # publish this, which is exactly why a test may read it and the robot
        # stack may not. Latching: a jam that clears still counts as a jam.
        self.collided = False
        self.create_subscription(Odometry, "/ground_truth", self._on_truth, 10)
        self.create_subscription(
            Bool, "/collision_truth",
            lambda m: setattr(self, "collided", self.collided or m.data), 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _on_truth(self, message):
        self.truth = message
        position = message.pose.pose.position
        self.path.append((position.x, position.y))

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
        handle = self._send_until_accepted(goal)

        result = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert result.done(), f"Nav2 did not finish within {timeout} s"
        return result.result().status

    def _send_until_accepted(self, goal, ready_timeout=45.0):
        """Send the goal, retrying while Nav2 is still coming up.

        ⚠️ wait_for_server is NOT a readiness check here. Nav2 creates the
        action server on CONFIGURE but rejects every goal until ACTIVATE, and
        the client cannot tell those two states apart — so the fixture's fixed
        20 s sleep either works or produces a bare "Nav2 rejected the goal",
        which reads like a navigation bug and is really a race. It showed up
        only when the whole suite ran, never on a single test, because three
        Nav2 stacks in one session is when activation runs late.

        Retrying is the honest wait: a genuinely broken Nav2 still fails, just
        ready_timeout later and with a message that says which failure it is.
        """
        deadline = time.monotonic() + ready_timeout
        while True:
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            send = self.client.send_goal_async(goal)
            while not send.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            handle = send.result()
            if handle.accepted:
                return handle
            assert time.monotonic() < deadline, (
                f"Nav2 kept rejecting the goal for {ready_timeout} s — its "
                f"lifecycle servers never reached ACTIVE")
            self.spin(1.0)


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


def wall_clearance(path):
    """Closest the robot ever came to any obstacle CENTRE along its path."""
    return min(
        math.hypot(x - WALL_X, y - wall_y)
        for x, y in path
        for wall_y in WALL_YS
    )


def test_nav2_routes_around_a_wall_it_cannot_drive_through(nav2_stack, navigator):
    """A3c acceptance: a wall across the straight line, and the robot gets past.

    Three separate claims, because arriving is not the same as avoiding:
      1. Nav2 reports success AND ground truth agrees it is at the goal — a
         confidently wrong filter would report a triumphant arrival from
         inside the wall.
      2. The path swings clear of the wall's tip. Without this, a robot that
         drove straight to a goal that happened to be reachable would pass.
      3. It never came within contact distance of an obstacle. The kinematic
         world already refuses to move into one, so this mostly guards world.py
         against a future regression that lets the robot bore through.
    """
    nav2_stack(*wall_sim_args())
    navigator.spin(3.0)
    assert navigator.truth is not None, "no /ground_truth — is sim_node up?"

    status = navigator.go(WALL_GOAL_X, WALL_GOAL_Y, timeout=180.0)
    navigator.spin(1.0)
    x, y = navigator.position
    error = math.hypot(x - WALL_GOAL_X, y - WALL_GOAL_Y)
    detour = max(abs(py) for _, py in navigator.path)

    assert status == 4, f"Nav2 reported status {status} (4 = SUCCEEDED)"
    assert error < ARRIVAL_TOLERANCE, (
        f"Nav2 says it arrived, but ground truth is {error:.2f} m from the goal "
        f"(ended at ({x:.2f}, {y:.2f}))")
    assert detour >= DETOUR_Y, (
        f"the robot never left the straight line (max |y| = {detour:.2f} m). It "
        f"reached the goal without going around — is the static layer actually "
        f"receiving /obstacle_map?")
    clearance = wall_clearance(navigator.path)
    assert clearance >= CONTACT_DISTANCE - 0.05, (
        f"the robot came {clearance:.2f} m from an obstacle centre, inside the "
        f"{CONTACT_DISTANCE:.2f} m contact distance")


def test_without_the_wall_the_same_goal_is_a_straight_line(nav2_stack, navigator):
    """The contrast the detour test needs to mean anything.

    A6's lesson in one sentence: a measurement that only ever ran in one
    configuration proves nothing. Identical goal, identical stack, no wall — if
    the robot swings wide here too, then the previous test was measuring RPP
    wandering rather than obstacle avoidance, and its assertion is worthless.
    """
    nav2_stack()
    navigator.spin(3.0)
    assert navigator.truth is not None, "no /ground_truth — is sim_node up?"

    status = navigator.go(WALL_GOAL_X, WALL_GOAL_Y, timeout=180.0)
    navigator.spin(1.0)
    deviation = max(abs(py) for _, py in navigator.path)

    assert status == 4, f"Nav2 reported status {status} (4 = SUCCEEDED)"
    assert deviation < STRAIGHT_LINE_Y, (
        f"with nothing in the way the robot still swung {deviation:.2f} m off "
        f"the line — the detour test cannot tell avoidance from wandering")


def test_an_unsurveyed_obstacle_jams_the_robot_and_nav2_claims_success(
        nav2_stack, navigator):
    """⚠️ THIS TEST DOCUMENTS A DEFICIENCY, NOT A FEATURE.

    The same wall and the same goal as the detour test, with one difference:
    this wall is not on the map. Nothing in the costmaps knows it exists (the
    robot has no range sensor — nav2.yaml's header explains why), so Nav2 plans
    straight through, the robot jams against it, and Nav2 reports SUCCEEDED.

    Why the lie is convincing: a jammed robot's wheels keep turning. The hall
    sensors report the commanded speed, the bridge integrates it into odometry,
    the EKF believes it — yaw comes from the gyro, but vx comes from the wheels
    and nothing contradicts them — and the goal checker watches the robot
    arrive. Only ground truth knows it never moved. The real robot does exactly
    this: hub motors slipping against a rock report a healthy 0.5 m/s. The
    simulator is not being unfair here, it is being accurate.

    Fixing it needs a second opinion on whether the robot is moving, and every
    candidate is blocked on hardware: GPS (present, but ekf_global is kept out
    of this stack by the yaw problem), motor current from the INA228 (SP1's
    software is ready, the sensor is not bought), or the bumper (contact only,
    and this wall gets hit dead-on). So it is documented rather than fixed.

    When it IS fixed, this test fails. That is the point of it — read the
    failure as "the gap closed", update it, and say so in the handoff.
    """
    nav2_stack(*unsurveyed_sim_args())
    navigator.spin(3.0)
    assert navigator.truth is not None, "no /ground_truth — is sim_node up?"

    status = navigator.go(WALL_GOAL_X, WALL_GOAL_Y, timeout=180.0)
    navigator.spin(1.0)
    x, y = navigator.position
    error = math.hypot(x - WALL_GOAL_X, y - WALL_GOAL_Y)

    assert navigator.collided, (
        "the robot never touched the unsurveyed wall — if something now avoids "
        "obstacles it cannot see, this test no longer describes reality")
    assert x < WALL_X, (
        f"ground truth puts the robot at x={x:.2f}, past the wall at {WALL_X} — "
        f"the world let it drive through")
    assert error > ARRIVAL_TOLERANCE, (
        f"the robot is only {error:.2f} m from the goal despite the wall")
    # The deficiency itself, pinned. Not an endorsement.
    assert status == 4, (
        f"Nav2 reported {status} instead of a bogus SUCCEEDED — if it now "
        f"detects being stuck, delete this assertion and update the handoff")
