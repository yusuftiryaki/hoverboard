"""SP3-SP5 swept: does the robot get onto the contacts from EVERY side?

test_docking.py proves the chain once, from one direction — the robot coming
in from the side it happened to be on. Handoff decision 11: anything about
direction is swept, never checked at a single point (the mirrored magnetometer
looked perfect at yaw 0). So here one stack docks again and again, from start
points spread around the station:

                  -45      0 (front, staging side)      45 ...
                        start ──► staging ──► dock ▌tag
              -90 / 90 beside it, 135..180 BEHIND it

For each start: Nav2 drives there, battery_supervisor's ~/return_home runs the
real sequence (staging, face the dock, DockRobot), ground truth judges the
result, ~/resume undocks, next start. The survey is exact here — survey error
is test_docking.py's question; this file isolates geometry.

The result is a TABLE, and every row is reported before anything is asserted:
one failing direction must not hide what the others did.

One stack, ~11 dockings, ~12 min. Its own CI stage (`sweep`). SKIPs without ROS.
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
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, String               # noqa: E402
from std_srvs.srv import Trigger                    # noqa: E402

# The station: docked base_link at (4, 0) facing east, so its FRONT — where
# staging is — faces west, towards the robot's start at the origin.
DOCK = (4.0, 0.0, 0.0)

# (bearing from the dock's front axis in degrees, distance from the dock in m),
# in an order that walks round the station instead of criss-crossing it.
CASES = [(0, 4.0), (45, 4.0), (90, 4.0), (135, 4.0), (180, 4.0),
         (-135, 4.0), (-90, 4.0), (-45, 4.0), (0, 2.0), (0, 8.0), (90, 2.0)]

# ⚠️ GYRO RESIDUAL BIAS OFF, and why. This stack has no magnetometer or GPS
# (Nav2Stack pins map to odom), so the sim's 0.1 deg/s residual integrates
# for the whole sweep. MEASURED with it on: the EKF was 24 deg / 1.7 m off by
# the fifth docking — and the docking itself still landed every time (the
# marker is a relative measurement), but the TRANSIT to the sixth start steered
# by a map rotated 18 deg against the world and drove into the real station
# (A3c trap 3 then took the estimate 2.7 m further off, and every later case
# failed). That is a localization finding — the robot outdoors runs ekf_global
# with the magnetometer for exactly this — not a docking-geometry one, so here
# it is switched off and the drift column shows what is left.
SIM_ARGS = ("--ros-args",
            "-p", f"dock_pose:=[{DOCK[0]}, {DOCK[1]}, {DOCK[2]}]",
            "-p", "imu_gyro_residual_bias_dps:=0.0")

# The contact window of robot_sim's station (dock_model.py).
CONTACT_LATERAL = 0.06

LATCHED = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def start_point(bearing_deg, distance):
    front = DOCK[2] + math.pi + math.radians(bearing_deg)
    return DOCK[0] + distance * math.cos(front), DOCK[1] + distance * math.sin(front)


def in_dock_frame(x, y, yaw):
    dx, dy = x - DOCK[0], y - DOCK[1]
    c, s = math.cos(DOCK[2]), math.sin(DOCK[2])
    yaw_error = math.atan2(math.sin(yaw - DOCK[2]), math.cos(yaw - DOCK[2]))
    return c * dx + s * dy, -s * dx + c * dy, yaw_error


class Sweeper(Node):
    def __init__(self):
        super().__init__("docking_sweep")
        self.truth = None
        self.estimate = None
        self.states = []
        self.contact = False
        self.collided = False
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        # map == odom in this stack (Nav2Stack pins them), so this IS where
        # the robot believes it is on the survey map.
        self.create_subscription(Odometry, "/odometry/filtered/local",
                                 lambda m: setattr(self, "estimate", m), 10)
        self.create_subscription(String, "/battery_supervisor/state",
                                 lambda m: self.states.append(m.data), LATCHED)
        self.create_subscription(Bool, "/dock_contact_truth",
                                 lambda m: setattr(self, "contact", m.data), 10)
        self.create_subscription(
            Bool, "/collision_truth",
            lambda m: setattr(self, "collided", self.collided or m.data), 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.return_home = self.create_client(Trigger, "/battery_supervisor/return_home")
        self.resume = self.create_client(Trigger, "/battery_supervisor/resume")

    @property
    def phase(self):
        return self.states[-1] if self.states else None

    @property
    def pose(self):
        p, q = self.truth.pose.pose.position, self.truth.pose.pose.orientation
        return p.x, p.y, 2.0 * math.atan2(q.z, q.w)

    def estimate_error(self):
        """(metres, degrees) between the EKF's pose and the truth."""
        p, q = self.estimate.pose.pose.position, self.estimate.pose.pose.orientation
        x, y, yaw = self.pose
        dyaw = 2.0 * math.atan2(q.z, q.w) - yaw
        return math.hypot(p.x - x, p.y - y), math.degrees(math.atan2(math.sin(dyaw),
                                                                     math.cos(dyaw)))

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()

    def call(self, client):
        assert client.wait_for_service(timeout_sec=20.0), client.srv_name
        future = client.call_async(Trigger.Request())
        self.spin_until(future.done, 20.0)
        return future.result()

    def drive_to(self, x, y, timeout=120.0):
        """Nav2 to (x, y); returns its status (Nav2 stops within 1 m)."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        assert self.nav.wait_for_server(timeout_sec=20.0)
        deadline = time.monotonic() + 45.0
        while True:
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            sent = self.nav.send_goal_async(goal)
            self.spin_until(sent.done, 10.0)
            if sent.result().accepted:
                break
            assert time.monotonic() < deadline, "Nav2 kept rejecting goals"
            self.spin_until(lambda: False, 1.0)
        result = sent.result().get_result_async()
        if not self.spin_until(result.done, timeout):
            return "timeout"
        return result.result().status


@pytest.fixture
def sweeper(ros):
    node = Sweeper()
    yield node
    node.destroy_node()


def test_the_robot_docks_from_every_side(dock_stack, sweeper):
    dock_stack(SIM_ARGS, dock_pose=DOCK)
    assert sweeper.spin_until(lambda: sweeper.truth is not None and sweeper.estimate is not None
                               and sweeper.phase, 60.0), \
        "no ground truth or no supervisor state"

    rows = []
    hit_before = False
    for bearing, distance in CASES:
        target = start_point(bearing, distance)
        sweeper.collided = False
        nav_status = sweeper.drive_to(*target) if math.dist(sweeper.pose[:2], target) > 1.0 else "-"
        transit_hit = sweeper.collided
        sweeper.collided = False
        start = sweeper.pose
        drift_m, drift_deg = sweeper.estimate_error()
        started = time.monotonic()
        reply = sweeper.call(sweeper.return_home)
        finished = reply.success and sweeper.spin_until(
            lambda: sweeper.phase in ("CHARGING", "SAFE_STOP"), 240.0)
        sweeper.spin_until(lambda: False, 1.0)
        along, lateral, yaw_error = in_dock_frame(*sweeper.pose)
        rows.append({
            "case": f"{bearing:+4d} deg {distance:.0f} m",
            "start": f"({start[0]:+.1f}, {start[1]:+.1f}, {math.degrees(start[2]):+4.0f})",
            "nav": nav_status,
            "result": sweeper.phase if finished else f"stuck in {sweeper.phase}",
            "contact": sweeper.contact,
            "along": along, "lateral": lateral, "yaw": math.degrees(yaw_error),
            "seconds": time.monotonic() - started,
            "drift": f"{drift_m:.2f} m {drift_deg:+.0f} deg",
            # On the way to the start: a collision with something.
            "transit_hit": transit_hit,
            # During the return: pressing the station's body at the end of
            # the approach also counts — that one is the station stopping
            # the robot, as it is built to.
            "dock_hit": sweeper.collided,
            # ⚠️ A jammed robot's wheels keep counting (A3c trap 3), so after
            # any collision the EKF can be metres off and every later case
            # inherits it. Flagged, so the table says which rows to trust.
            "after_hit": hit_before,
        })
        hit_before = hit_before or transit_hit
        # Off the dock (or out of SAFE_STOP) for the next case.
        if sweeper.phase in ("CHARGING", "SAFE_STOP"):
            sweeper.call(sweeper.resume)
            sweeper.spin_until(lambda: sweeper.phase in ("NORMAL", "SAFE_STOP"), 90.0)
            if sweeper.phase == "SAFE_STOP":
                sweeper.call(sweeper.resume)
                sweeper.spin_until(lambda: sweeper.phase == "NORMAL", 10.0)

    table = "\n".join(
        f"{r['case']:>14}  start {r['start']}  nav {r['nav']!s:>7}  {r['result']:<10} "
        f"contact {r['contact']!s:<5}  along {r['along']:+.3f}  lateral {r['lateral']:+.3f}  "
        f"yaw {r['yaw']:+5.1f}  {r['seconds']:5.0f} s  ekf off {r['drift']}  "
        f"hit transit/dock {r['transit_hit']:d}/{r['dock_hit']:d}"
        f"{'  (after a hit: estimate suspect)' if r['after_hit'] else ''}"
        for r in rows)
    print("\nDOCKING SWEEP\n" + table)
    failed = [r["case"] for r in rows
              if r["transit_hit"] or not (r["result"] == "CHARGING" and r["contact"]
                                          and abs(r["lateral"]) <= CONTACT_LATERAL)]
    assert not failed, f"did not dock from {failed}:\n{table}"
