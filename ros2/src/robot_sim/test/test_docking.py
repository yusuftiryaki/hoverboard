"""SP3-SP5 acceptance: a mission, a battery that runs low, and the way home.

    /battery LOW ─► battery_supervisor ─► cancel mission ─► staging ─► face dock
                                                        ─► DockRobot (marker) ─► contacts ─► charging

Everything below the supervisor is the real stack: Nav2, opennav_docking's
docking_server, nav_cmd_gate, twist_mux, the bridge and its protocol, the
INA228 driver and the SoC estimator (behind a fake chip carrying the simulated
pack's current). The simulator supplies the world: the pack, the station body,
the contacts, and what the camera would make of the marker.

Ground truth is the judge, as everywhere here. "DockRobot succeeded" means the
robot BELIEVES it is charging; /dock_contact_truth says whether the contacts
actually touch.

⚠️ What this does not prove (design doc, "simin kanıtlamadıkları"): contact
mechanics, oxidation, the real charge curve, AprilTag detection in sunlight,
INA228 accuracy. The detection is ground truth plus noise in the right
convention; image_proc's detector itself never sees a pixel here.

Spawns processes, ~2 x 2.5 min. SKIPs without ROS.
"""

import math
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                        # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist    # noqa: E402
from nav2_msgs.action import NavigateToPose         # noqa: E402
from nav_msgs.msg import Odometry                   # noqa: E402
from rclpy.action import ActionClient               # noqa: E402
from rclpy.node import Node                         # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import BatteryState            # noqa: E402
from std_msgs.msg import Bool, String               # noqa: E402

# The station, truly: robot drives SOUTH into it, docked base_link at (0, -3).
DOCK_X, DOCK_Y, DOCK_YAW = 0.0, -3.0, -math.pi / 2
# The survey got it 0.3 m wrong sideways. Staging and turning use THIS; only
# the marker can put the robot on the real contacts.
SURVEY_ERROR_X = 0.3
SURVEYED_DOCK = (DOCK_X + SURVEY_ERROR_X, DOCK_Y, DOCK_YAW)

MISSION_X, MISSION_Y = 12.0, 0.0

# A 0.05 Ah pack, on both sides: the sim's truth and the monitor's belief.
# Idle draw off, so the SoC at the start of the mission does not depend on how
# long a loaded CI box took to bring Nav2 up. Measured from the tables:
# sim SoC 0.32 -> 37.32 V at rest -> estimator 0.33. At 0.5 m/s the pack loses
# ~1.5 %/s, so the 0.30 threshold falls a few metres into the mission, and the
# trip home costs ~10 % — far from the 0.05 critical line.
CAPACITY_AH = 0.05
SIM_ARGS_COMMON = (
    "--ros-args",
    "-p", f"dock_pose:=[{DOCK_X}, {DOCK_Y}, {DOCK_YAW}]",
    "-p", f"battery_capacity_ah:={CAPACITY_AH}",
    "-p", "battery_initial_soc:=0.32",
    "-p", "battery_idle_current_a:=0.0",
)
MONITOR = {"capacity_ah": CAPACITY_AH}
SUPERVISOR = {"return_soc": 0.30, "critical_soc": 0.05}

LATCHED = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Observer(Node):
    def __init__(self):
        super().__init__("docking_test_observer")
        self.truth = None
        self.states = []
        self.contact = False
        self.lock = None
        self.battery = None
        self.create_subscription(Odometry, "/ground_truth",
                                 lambda m: setattr(self, "truth", m), 10)
        self.create_subscription(String, "/battery_supervisor/state",
                                 lambda m: self.states.append(m.data), LATCHED)
        self.create_subscription(Bool, "/dock_contact_truth",
                                 lambda m: setattr(self, "contact", m.data), 10)
        self.create_subscription(Bool, "/nav_lock",
                                 lambda m: setattr(self, "lock", m.data), 10)
        self.create_subscription(BatteryState, "/battery",
                                 lambda m: setattr(self, "battery", m), 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def pose(self):
        p, q = self.truth.pose.pose.position, self.truth.pose.pose.orientation
        return p.x, p.y, 2.0 * math.atan2(q.z, q.w)

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert predicate(), f"{what} within {timeout:.0f} s (supervisor states: {self.states})"

    def start_mission(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = MISSION_X
        goal.pose.pose.position.y = MISSION_Y
        goal.pose.pose.orientation.w = 1.0
        assert self.nav.wait_for_server(timeout_sec=20.0)
        deadline = time.monotonic() + 45.0
        while True:
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            future = self.nav.send_goal_async(goal)
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.result().accepted:
                return future.result().get_result_async()
            assert time.monotonic() < deadline, "Nav2 kept rejecting the mission"
            self.spin(1.0)


@pytest.fixture
def observer(ros):
    node = Observer()
    yield node
    node.destroy_node()


def in_dock_frame(x, y, dock):
    """(along, lateral) of a point relative to a dock pose."""
    dx, dy = x - dock[0], y - dock[1]
    c, s = math.cos(dock[2]), math.sin(dock[2])
    return c * dx + s * dy, -s * dx + c * dy


def resume():
    return subprocess.run(
        ["ros2", "service", "call", "/battery_supervisor/resume", "std_srvs/srv/Trigger"],
        capture_output=True, text=True, timeout=30).stdout


def test_low_battery_abandons_the_mission_and_docks_on_the_real_contacts(
        dock_stack, observer):
    """The design's acceptance tests 1, 2 and 4 in one drive:
      1. SoC under the return threshold cancels the mission and goes home;
      2. the approach converges onto the REAL station although the survey
         put it 0.3 m to the side — the marker did that, nothing else knew;
      4. current flowing into the pack makes it CHARGING, and twist_mux's
         Nav2 input is locked while it sits there.
    Then a person resumes: the lock comes off FIRST (the undock drives on the
    Nav2 input) and the robot backs off the contacts."""
    dock_stack(SIM_ARGS_COMMON, monitor=MONITOR, supervisor=SUPERVISOR,
               dock_pose=SURVEYED_DOCK)
    observer.wait_for(lambda: observer.truth is not None and observer.battery is not None
                      and math.isfinite(observer.battery.percentage), 60.0,
                      "no ground truth or no SoC on /battery")
    start_soc = observer.battery.percentage
    assert start_soc > SUPERVISOR["return_soc"], (
        f"the estimator starts at SoC {start_soc:.3f}, already under the return "
        "threshold — the test cannot show a mission being abandoned")

    mission = observer.start_mission()
    observer.wait_for(lambda: "RETURNING" in observer.states, 120.0,
                      "the battery never sent the robot home")
    observer.wait_for(mission.done, 10.0, "the mission was not cancelled")
    abandoned_at = observer.pose
    assert mission.result().status == 5, (
        f"mission status {mission.result().status}, expected 5 (CANCELED)")
    assert abandoned_at[0] > 1.0, f"the mission never really started: {abandoned_at}"

    observer.wait_for(lambda: observer.states[-1] in ("CHARGING", "SAFE_STOP"), 180.0,
                      "the robot never finished docking")
    observer.spin(2.0)
    x, y, yaw = observer.pose
    along, lateral = in_dock_frame(x, y, (DOCK_X, DOCK_Y, DOCK_YAW))
    _, surveyed_lateral = in_dock_frame(x, y, SURVEYED_DOCK)
    assert observer.states[-1] == "CHARGING", (
        f"docking ended in {observer.states[-1]}; robot at ({x:.3f}, {y:.3f}) is "
        f"{along:+.3f} m along / {lateral:+.3f} m beside the real dock")
    assert "DOCKING" in observer.states
    assert observer.contact, (
        f"the supervisor says CHARGING but the contacts do not touch: "
        f"{along:+.3f} m along / {lateral:+.3f} m beside the real dock")
    assert abs(lateral) < 0.06 < abs(surveyed_lateral), (
        f"lateral {lateral:+.3f} m from the real dock, {surveyed_lateral:+.3f} m "
        "from the surveyed one — the marker should have pulled it onto the real one")
    assert observer.battery.current > 0.5, (
        f"/battery current {observer.battery.current:.2f} A — nothing flowing in")
    observer.wait_for(lambda: observer.lock is True, 3.0, "nav_lock is not held on the dock")

    reply = resume()
    assert "success=True" in reply, reply
    observer.wait_for(lambda: observer.states[-1] in ("NORMAL", "SAFE_STOP"), 60.0,
                      "undocking never finished")
    assert observer.states[-1] == "NORMAL", f"undocking ended in {observer.states[-1]}"
    assert observer.lock is False
    observer.spin(1.0)
    along_after, _ = in_dock_frame(*observer.pose[:2], (DOCK_X, DOCK_Y, DOCK_YAW))
    assert not observer.contact and along_after < -1.0, (
        f"undocked, but the robot is only {along_after:+.2f} m from the dock")


def test_a_marker_it_cannot_see_ends_in_a_safe_stop_that_teleop_can_still_drive(
        dock_stack, observer):
    """The failure path: same trip home, but the camera never decodes the
    marker (dirt, glare, night — design doc "İşaretçi görünmüyor"). The dock
    server gives up, the supervisor stops the robot and locks Nav2 out — and a
    person can still drive it, because teleop outranks the lock."""
    dock_stack((*SIM_ARGS_COMMON, "-p", "dock_tag_visible:=false"),
               monitor=MONITOR, supervisor=SUPERVISOR, dock_pose=SURVEYED_DOCK)
    observer.wait_for(lambda: observer.truth is not None and observer.battery is not None
                      and math.isfinite(observer.battery.percentage), 60.0,
                      "no ground truth or no SoC on /battery")
    observer.start_mission()
    observer.wait_for(lambda: "DOCKING" in observer.states, 180.0,
                      "the robot never got as far as docking")
    observer.wait_for(lambda: observer.states[-1] == "SAFE_STOP", 60.0,
                      "an invisible marker did not end in SAFE_STOP")
    observer.wait_for(lambda: observer.lock is True, 3.0, "SAFE_STOP without nav_lock")
    assert not observer.contact

    observer.spin(1.0)
    before = observer.pose
    observer.spin(3.0)
    after = observer.pose
    drift = math.hypot(after[0] - before[0], after[1] - before[1])
    assert drift < 0.02, f"the robot moved {drift:.3f} m in SAFE_STOP"

    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        observer.teleop.publish(Twist(angular=type(Twist().angular)(z=0.5)))
        observer.spin(0.05)
    turned = abs(math.atan2(math.sin(observer.pose[2] - after[2]),
                            math.cos(observer.pose[2] - after[2])))
    assert turned > 0.3, (
        f"teleop turned the robot only {turned:.2f} rad — the battery lock must "
        "not take manual control away")
