"""A3's acceptance test: /cmd_vel -> real physics -> /odom, against ground truth.

This is what the whole Gazebo backend exists to prove, and what the kinematic
world can never say. KinematicWorld's wheels turn exactly as commanded and the
robot goes exactly where perfect differential-drive kinematics says it goes; the
hall sensors and the truth agree by construction. Here they do not, and the gap
has a name: SLIP.

    /cmd_vel ─► hoverboard_bridge ─pty─► Esp32Sim ─► GazeboBackend ─► Gazebo
                      ▲                                                  │
                      └── halls ── DiffDrive dead reckoning ◄────────────┤
                                                                         │
                      /ground_truth ◄── OdometryPublisher (true pose) ◄───┘

⚠️ THE TWO ODOMETRIES MUST COME FROM DIFFERENT PLUGINS. DiffDrive's odometry is
dead reckoning from the wheel joint angles — the hall sensors' analogue, which
keeps counting while the wheels spin on the spot. Ground truth is a separate
OdometryPublisher differentiating the model's real pose. Sourcing both from
DiffDrive would make every number below zero by construction.

⚠️ SAMPLE BOTH AT THE SAME INSTANT, never "the newest message on each topic".
conftest's PoseTrack does the interpolation and explains what it costs to skip.

Spawns Gazebo and the full stack, ~3 min for the file. SKIPs without ROS or gz.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                             # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry         # noqa: E402
from rclpy.node import Node                              # noqa: E402

# How much slower than real time the sim may run before drive() calls it
# dead. Gazebo is configured for RTF 1.0; this is slack for a busy box.
SIM_TIME_STALL_FACTOR = 6.0
from rclpy.qos import DurabilityPolicy, QoSProfile       # noqa: E402
from std_msgs.msg import Bool                            # noqa: E402

# hoverbot.sdf's stated wheel friction. The bounds below are derived from it by
# hand, so a change there must be made here too — deliberately not imported
# from the SDF: a test that reads the same number the model reads proves the
# two files agree, not that the physics obeys either of them.
WHEEL_MU = 0.5
STANDARD_GRAVITY = 9.80665

# The chassis box is 0.7 m long, so its front face is this far ahead of the
# model origin — which sits between the wheel contact patches.
CHASSIS_HALF_LENGTH = 0.35


def slip_floor(speed):
    """The least slip physics allows when stepping to `speed` from rest.

    Hand derivation, and the reason it is an inequality rather than a formula:

      * DiffDrive velocity-controls the wheel joints, so a step command puts the
        contact patch at `speed` while the body is still at rest. The contact
        slides, and the body can only be pushed by friction.
      * Friction cannot accelerate the body faster than a = mu * g, and that is
        the case where the driven wheels carry ALL the weight. Ours do not —
        accelerating throws load onto the rear caster — so the real limit is
        lower and the real slip is larger. `mu * g` is what makes this a floor
        and not a fit.
      * Slipping ends when the body reaches `speed`, at t = speed / a. The
        wheels report speed * t while the body has covered speed * t / 2, so
        the wheels over-report by speed^2 / (2 a).

    Measured through the full stack: 1.31x this floor at 0.25 m/s, 1.37x at
    0.5, 1.41x at 1.0 — the same ratio at every speed, which is what a genuine
    load-transfer deficit looks like.
    """
    return speed ** 2 / (2.0 * WHEEL_MU * STANDARD_GRAVITY)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Probe(Node):
    """Drives /cmd_vel and reads the two odometries at matched stamps."""

    def __init__(self, new_track):
        # ⚠️ Sim time, and set at construction rather than afterwards. The
        # stamps this node compares are Gazebo's and drive() now counts in
        # Gazebo's seconds, so the node's clock has to BE that clock from its
        # first tick — not from whenever a later set_parameters takes effect.
        super().__init__("gazebo_physics_probe", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = new_track()
        self.truth = new_track()
        self.collisions = []
        self.obstacle_map = None
        self.command = (0.0, 0.0)
        self.create_subscription(Odometry, "/odom", self.odom.add, 50)
        self.create_subscription(Odometry, "/ground_truth", self.truth.add, 50)
        self.create_subscription(Bool, "/collision_truth",
                                 lambda m: self.collisions.append(m.data), 50)
        self.create_subscription(
            OccupancyGrid, "/obstacle_map",
            lambda m: setattr(self, "obstacle_map", m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        # 20 Hz, comfortably inside hoverboard_bridge's 0.5 s cmd_timeout and
        # the simulated ESP32's 0.2 s watchdog. Falling below either would stop
        # the wheels mid-measurement, which reads exactly like a physics fault.
        self.create_timer(0.05, self._republish)

    def _republish(self):
        message = Twist()
        message.linear.x, message.angular.z = self.command
        self._publisher.publish(message)

    # ---- Driving ------------------------------------------------------------
    def drive(self, v, w, seconds):
        """Hold a command for `seconds` of SIM time, not wall time.

        ⚠️ The difference is not pedantry. The robot moves on Gazebo's clock,
        so a leg timed by the wall clock is a different manoeuvre on a loaded
        machine than on an idle one — and the whole suite runs Gazebo, Nav2 and
        a full ROS stack on eight cores. Timed by the wall clock, this test's
        square measured 0.095 m of EKF error alone and 0.505 m inside the full
        suite: five times the effect being measured, coming from nothing but
        how busy the box was.

        The wall-clock cap is a deadlock guard, not the timer: if /clock stops
        (Gazebo died) sim time freezes and this would otherwise never return.
        """
        self.command = (float(v), float(w))
        start = self.get_clock().now()
        wall_cap = time.monotonic() + seconds * SIM_TIME_STALL_FACTOR + 10.0
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)
            assert time.monotonic() < wall_cap, (
                f"sim time advanced less than {seconds} s in "
                f"{seconds * SIM_TIME_STALL_FACTOR + 10.0:.0f} s of wall time — "
                "Gazebo has stopped publishing /clock")

    def settle(self, seconds=2.0, timeout_s=30.0):
        """Wait for both odometries to arrive, THEN sit still.

        ⚠️ Not a sleep. A fresh node joining a graph this size (Gazebo, the
        bridge, the whole robot stack) can take several seconds to discover
        /odom, and a fixed 2 s wait passed on an idle machine and failed under
        load with "no /odom — is hoverboard_bridge up?", which reads like a
        crashed bridge rather than the race it is. Same lesson as A3c's
        wait_for_server: a sleep is not a readiness check.
        """
        deadline = time.monotonic() + timeout_s
        self.command = (0.0, 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom.newest is not None and self.truth.newest is not None:
                break
        assert self.odom.newest is not None, "no /odom — is hoverboard_bridge up?"
        assert self.truth.newest is not None, "no /ground_truth — is sim_node up?"
        self.rest(seconds)

    def rest(self, seconds=3.0):
        """Come to a full stop AND let the decelerating skid finish.

        Braking slips too, in the opposite direction: the wheels are commanded
        to zero while the body is still moving, so they under-report by the same
        v^2/(2a). A measurement window that spans a stop would net the two out
        to nearly nothing and hide the effect entirely. Every window below runs
        from rest to cruise and stops there.
        """
        self.drive(0.0, 0.0, seconds)

    def drive_until(self, v, w, predicate, limit_s=60.0):
        self.command = (float(v), float(w))
        end = time.monotonic() + limit_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return True
        return False

    # ---- Reading ------------------------------------------------------------
    def sim_now(self):
        """The newest stamp BOTH tracks can answer for."""
        assert self.odom.newest is not None, "no /odom — is hoverboard_bridge up?"
        assert self.truth.newest is not None, "no /ground_truth — is sim_node up?"
        return min(self.odom.newest, self.truth.newest)

    def sample(self):
        moment = self.sim_now()
        odom, truth = self.odom.at(moment), self.truth.at(moment)
        assert odom is not None and truth is not None
        return odom, truth

    def travelled(self, v, w, metres, ramp_s=0.0):
        """Drive `metres` of TRUE distance and return (odometry, truth) deltas.

        Distance rather than time on purpose. It is what separates slip from an
        odometry scale error: over a fixed distance a scale error contributes
        the SAME extra metres at every speed, while friction slip is spent
        entirely on getting up to speed and so grows as v^2. Timing the runs
        instead would make both grow with v and the two explanations would be
        much harder to tell apart.
        """
        self.rest()
        start = self.sim_now()
        odom0, truth0 = self.odom.at(start), self.truth.at(start)
        if ramp_s:
            steps = max(1, int(ramp_s / 0.05))
            for step in range(steps):
                self.drive(v * (step + 1) / steps, w, 0.05)
        reached = self.drive_until(
            v, w,
            lambda: (lambda t: t is not None and distance(t, truth0) >= metres)(
                self.truth.at(self.sim_now())))
        assert reached, f"the robot never covered {metres} m at {v} m/s"
        # Hold briefly so both tracks bracket the final instant.
        self.drive(v, w, 0.3)
        end = self.sim_now()
        odom1, truth1 = self.odom.at(end), self.truth.at(end)
        return distance(odom1, odom0), distance(truth1, truth0)


@pytest.fixture
def probe(ros, pose_track):
    node = Probe(pose_track)
    yield node
    node.command = (0.0, 0.0)
    node.destroy_node()


# --------------------------------------------------------------------------
# These four share ONE obstacle-free world — see conftest's `gazebo` fixture.
# Each measures a delta between two rest states, so order does not matter.
# --------------------------------------------------------------------------

def test_the_physics_world_actually_moves_the_robot(gazebo, probe):
    """The regression guard for A3's other silent failure.

    Both wheels used to sit at the model origin with no <pose>, buried inside
    the chassis box and touching nothing. Gazebo velocity-controlled them
    anyway and DiffDrive dead-reckoned odometry from them anyway, so
    /odometry climbed to 5.47 m while the true pose read 3.8e-9 m. Every
    surface sign — the sim runs, the model spawns, odometry counts up — looked
    like a working robot. Only the true pose knew.
    """
    gazebo()
    probe.settle()
    odom_delta, truth_delta = probe.travelled(0.4, 0.0, 2.0)

    assert truth_delta > 1.0, (
        f"ground truth moved {truth_delta:.4f} m while the wheels reported "
        f"{odom_delta:.4f} m — the wheels are turning without driving anything")
    # 2 m of true travel, so the wheels may only over-report by the slip.
    assert odom_delta == pytest.approx(truth_delta, abs=0.1)


def test_the_stack_runs_on_gazebos_clock(gazebo, probe):
    """use_sim_time, end to end, on the topic the EKF actually consumes.

    /clock was not bridged and nothing set use_sim_time — two halves of one
    bug, and because BOTH were missing neither could show: with no /clock a
    node set to sim time would simply hang, which someone would have noticed.
    Sim time starts near zero and the wall clock is a Unix epoch, so the two
    are impossible to confuse by accident.
    """
    gazebo()
    probe.settle()
    _, truth = probe.sample()
    assert truth is not None

    stamp = probe.odom.newest
    wall = time.time()
    assert stamp < wall - 3.15e8, (
        f"/odom is stamped {stamp:.1f}, which is wall-clock time, not Gazebo's "
        "— hoverboard_bridge is not running with use_sim_time:=true")
    # And it must be MOVING, not a frozen zero: a node waiting for a /clock
    # that never arrives also fails the check above.
    before = probe.odom.newest
    probe.rest(1.5)
    assert probe.odom.newest > before, "sim time is not advancing"


def test_the_robot_goes_where_it_points_at_every_heading(gazebo, probe):
    """Sweep the whole circle; never test a direction at one point.

    The rule A6 wrote in blood. A mirrored axis, a 90-degree mounting offset and
    a swapped pair of wheels all look perfect at exactly one heading. Two claims
    are checked at each of eight headings around the full turn: the robot
    translates along its own nose, and a positive commanded yaw rate turns it
    counter-clockwise (REP-103). A model with its wheels swapped left-for-right
    passes neither.
    """
    gazebo()
    probe.settle()
    worst_course = 0.0
    turns = []
    for _ in range(8):
        _, before = probe.sample()
        probe.drive(0.0, 0.7, 1.2)          # ~48 deg to the next heading
        probe.rest(1.0)
        _, heading = probe.sample()
        turns.append(math.atan2(math.sin(heading[2] - before[2]),
                                math.cos(heading[2] - before[2])))

        probe.drive(0.35, 0.0, 3.0)         # ~1.05 m straight
        probe.rest(1.0)
        _, after = probe.sample()
        assert distance(after, heading) > 0.5, "the robot did not move at all"
        course = math.atan2(after[1] - heading[1], after[0] - heading[0])
        worst_course = max(worst_course, angle_diff(course, heading[2]))

    # Measured: 0.02 deg at every heading. This is a geometry check, so the
    # threshold is loose enough to survive settling and nowhere near loose
    # enough to survive a mirrored or swapped axis, which costs 90 or 180.
    assert math.degrees(worst_course) < 5.0, (
        f"the robot's course differs from its heading by "
        f"{math.degrees(worst_course):.2f} deg — it is not driving along its "
        "own nose, so the model's forward axis is wrong")
    assert all(turn > 0.0 for turn in turns), (
        f"a positive yaw rate turned the robot clockwise: {turns} — the wheels "
        "are swapped left for right")
    # A full circle, so no heading is untested: 8 x ~48 deg is a bit over 360.
    assert sum(turns) > 2.0 * math.pi, (
        f"the sweep only covered {math.degrees(sum(turns)):.0f} deg")


def test_the_wheels_over_report_distance_because_they_slip(gazebo, probe):
    """A3's headline claim: the thing the kinematic world does not have.

    Over a FIXED true distance, at three speeds, the hall-derived odometry the
    robot believes must exceed the distance actually covered — by at least the
    hand-derived friction floor, and by an amount that grows as the SQUARE of
    the speed it was stepped to.

    That second half is the counter-test, and it is what makes this an argument
    rather than an observation. Over a fixed distance the rival explanations
    each have their own signature:

        a wheel-radius or units-per-RPM scale error -> the same extra metres
            at every speed
        friction slip during spin-up               -> v^2

    Measured through the full stack over 2.5 m: 0.0084 / 0.0350 / 0.1434 m at
    0.25 / 0.5 / 1.0 m/s. Ratios 4.17 and 4.10 against 4.0 predicted; a scale
    error would have given 1.0 twice. Spread across three repeats: under 2 mm.
    """
    gazebo()
    probe.settle()
    slips = {}
    for speed in (0.25, 0.5, 1.0):
        odom_delta, truth_delta = probe.travelled(speed, 0.0, 2.5)
        slips[speed] = odom_delta - truth_delta

        assert slips[speed] > slip_floor(speed), (
            f"at {speed} m/s the wheels over-reported by only "
            f"{slips[speed]:.4f} m; friction cannot let the body reach that "
            f"speed in less than {slip_floor(speed):.4f} m of slip. Either the "
            "two odometries are secretly the same source, or the wheels are "
            "not carrying the robot")
        # The floor is a floor, not a target. Measured ~1.4x; 4x means
        # something is wrong in a way this test should not wave through.
        assert slips[speed] < 4.0 * slip_floor(speed), (
            f"{slips[speed]:.4f} m of slip at {speed} m/s is far beyond "
            f"{slip_floor(speed):.4f} m — the wheels are barely gripping")

    assert slips[0.5] > 3.0 * slips[0.25], (
        f"halving the speed should quarter the slip over the same distance; "
        f"got {slips[0.25]:.4f} -> {slips[0.5]:.4f}. A constant ratio near 1 "
        "means this is an odometry SCALE error, not slip")
    assert slips[1.0] > 3.0 * slips[0.5], (
        f"got {slips[0.5]:.4f} -> {slips[1.0]:.4f}, expected roughly x4")


def test_a_gentle_ramp_to_the_same_speed_and_distance_barely_slips(gazebo, probe):
    """The other half of the counter-test: remove the CAUSE, lose the effect.

    Same top speed, same 2.5 m, same everything — only the step command becomes
    a four-second ramp, so the tyres are never asked for more force than they
    have. If the slip above were a scale error, a bias, or a bug in how the two
    odometries are compared, it would be just as large here.

    Measured: -0.0024 m ramped against +0.1434 m stepped. Sixty times smaller,
    and the wrong sign for slip — which is the honest noise floor of the whole
    /cmd_vel-to-/odom chain, hall quantisation included.
    """
    gazebo()
    probe.settle()
    stepped_odom, stepped_truth = probe.travelled(1.0, 0.0, 2.5)
    ramped_odom, ramped_truth = probe.travelled(1.0, 0.0, 2.5, ramp_s=4.0)

    stepped = stepped_odom - stepped_truth
    ramped = ramped_odom - ramped_truth
    assert stepped > slip_floor(1.0), "the stepped control case did not slip"
    assert abs(ramped) < 0.02, (
        f"ramping to the same speed over the same distance still moved the two "
        f"odometries {ramped:+.4f} m apart, so the gap is not spin-up slip")
    assert abs(ramped) < 0.25 * stepped, (
        f"ramped {ramped:+.4f} m vs stepped {stepped:+.4f} m — not a big "
        "enough difference to blame acceleration for the stepped slip")


# --------------------------------------------------------------------------
# Its own world: it spawns obstacles and it leaves the robot jammed. Asking for
# different arguments tears the shared one down first, so only one gz ever runs.
# --------------------------------------------------------------------------

def test_a_spawned_obstacle_stops_the_robot_where_the_map_says_it_is(gazebo, probe):
    """Obstacles reach Gazebo AND the map, from one list.

    sim_node used to REFUSE this combination outright, because the backend
    could not put a ROS parameter into Gazebo and publishing the map anyway
    would have drawn obstacles the physics drove straight through — a phantom
    obstacle with a warning in front of it. Both are now rendered from the one
    parsed tuple (gazebo_obstacles.py beside obstacle_map.py), so this asserts
    the two agree about a single obstacle in three independent ways: the grid
    marks it, the chassis physically stops at it, and the contact sensor says
    so.

    ⚠️ And then the part nobody wants: A3c's trap 3 reproduced by real physics
    rather than argued for. Jammed against the obstacle the wheels keep
    turning, the halls keep reporting speed, and /odom keeps integrating
    distance the robot is not covering. Measured: ground truth moved 0.0000 m
    while /odom advanced 2.97 m in six seconds. This is not a simulator
    artefact — hub motors braced against a rock report a healthy 0.5 m/s, and
    it is why Nav2 calls a jammed robot SUCCEEDED.
    """
    gazebo("-p", "obstacle_centers:=[3.0, 0.0]", "-p", "obstacle_radii:=[0.5]")
    probe.settle(3.0)

    # 1. The map has it, at the coordinates it was asked for.
    assert probe.obstacle_map is not None, "no latched /obstacle_map"
    info = probe.obstacle_map.info
    column = round((3.0 - info.origin.position.x) / info.resolution)
    row = round((0.0 - info.origin.position.y) / info.resolution)
    assert probe.obstacle_map.data[row * info.width + column] == 100
    away = round((4.0 - info.origin.position.x) / info.resolution)
    assert probe.obstacle_map.data[row * info.width + away] == 0

    # 2. The physics has it, at the same place, to the centimetre. The chassis
    #    box is what makes contact, so the stopping point is worked out by hand
    #    from three lengths and nothing else: 3.0 - 0.5 - 0.35.
    probe.collisions.clear()
    contact_x = 3.0 - 0.5 - CHASSIS_HALF_LENGTH
    probe.drive(0.5, 0.0, 16.0)
    odom_jammed, truth_jammed = probe.sample()
    assert truth_jammed[0] == pytest.approx(contact_x, abs=0.05), (
        f"the robot stopped at x={truth_jammed[0]:.4f}, not the {contact_x} m "
        "the map's obstacle and the chassis geometry predict")

    # 3. The collision oracle agrees. Nothing in the robot stack may read this.
    assert any(probe.collisions), (
        "the chassis contact sensor never fired — /collision_truth is blind, "
        "so `gz-sim-contact-system` or bridge.yaml's sensor topic is wrong")

    # 4. And the robot's own odometry does not know any of it.
    probe.drive(0.5, 0.0, 6.0)
    odom_after, truth_after = probe.sample()
    assert distance(truth_after, truth_jammed) < 0.05, "it broke through"
    assert distance(odom_after, odom_jammed) > 1.5, (
        "jammed against an obstacle at 0.5 m/s, /odom should still be counting "
        f"distance — it advanced only "
        f"{distance(odom_after, odom_jammed):.3f} m. If this now fails because "
        "the stack DETECTED the jam, that is the fix A3c asked for: delete "
        "this assertion and celebrate")
