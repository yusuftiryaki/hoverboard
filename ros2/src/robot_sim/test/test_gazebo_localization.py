"""How much does WHEEL SLIP cost the estimate? The question A3 opened and left.

test_localization.py measures the EKF against ground truth in the kinematic
world, where the wheels turn exactly as commanded and slip does not exist. It
says so itself, and it has always carried the caveat that its 0.083 m of error
over an 8 m square proves the maths is wired up and nothing about outdoors.
This file drives THE SAME SQUARE on real physics and puts a number on the gap.

    kinematic world (no slip)   0.080-0.082 m position error, 4.2-4.3 deg yaw
    physics world   (slip)      0.073-0.079 m position error, 3.2-3.4 deg yaw  <- here
    (2026-10-15: gz-sim 8.15.0, accelerometer out of the EKF; 5 and 6 runs)

⚠️ THE ANSWER IS "ALMOST NOTHING", AND THAT IS NOT THE REASSURANCE IT LOOKS
LIKE. Spin-up slip is real and large — stepping to 1 m/s the wheels over-report
by 137 mm (test_gazebo_physics.py) — but braking gives 91% of it straight back,
because a wheel commanded to zero while the body still moves under-reports by
the same mechanism in the opposite direction. Over any manoeuvre that starts
and stops, it very nearly cancels. The second test here isolates exactly that,
because a result this convenient has to be explained rather than banked.

What does NOT cancel is one-directional slip: a wheel that keeps turning
without carrying the robot anywhere. That is the jam of A3c trap 3, it is
measured in test_gazebo_physics.py at +2.97 m of imaginary travel in six
seconds, and nothing in this stack can see it.

⚠️ Its own Gazebo world, on purpose. The module-scoped `gazebo` fixture gives
one per test module, and this measurement needs a stack that has not already
been driven around for minutes: the EKF's yaw is unobservable, so whatever
drift it has accumulated rotates every displacement measured afterwards. Run
on a shared, already-exercised world the same square read 18 deg of yaw error
in one run and 3.3 in the next.

Spawns Gazebo and the full stack, ~2 min. SKIPs without ROS or gz.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                             # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from sensor_msgs.msg import Imu                          # noqa: E402
from rclpy.node import Node                              # noqa: E402

# How much slower than real time the sim may run before drive() calls it
# dead. Gazebo is configured for RTF 1.0; this is slack for a busy box.
SIM_TIME_STALL_FACTOR = 6.0

# What the same square costs in the kinematic world, from test_localization.py.
# Quoted, not imported: this is a number to compare against, and reaching into
# the other module for it would make the two move together silently.
KINEMATIC_SQUARE_ERROR_M = 0.081   # 0.080-0.082 m over 5 runs, 2026-10-15


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Probe(Node):
    """Ground truth, the EKF estimate and raw wheel odometry, at matched stamps.

    All three, because the interesting comparison is not just "how wrong is the
    EKF" but "is it wronger than the wheels it is built on". Measured here: it
    is not better. With no magnetometer nothing observes absolute heading, so
    fusing a biased gyro with biased wheels lands between the two rather than
    above them — the same conclusion A1 reached, now with slip in the picture.
    """

    def __init__(self, new_track):
        # Sim time, set at construction: drive() counts in Gazebo's seconds.
        super().__init__("gazebo_localization_probe", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.truth = new_track()
        self.ekf = new_track()
        self.odom = new_track()
        self.command = (0.0, 0.0)
        self.create_subscription(Odometry, "/ground_truth", self.truth.add, 50)
        self.create_subscription(Odometry, "/odometry/filtered/local", self.ekf.add, 50)
        self.create_subscription(Odometry, "/odom", self.odom.add, 50)
        # Rate only — the EKF's other input. When the estimate is worse than
        # the wheels it is built on, this is the input to suspect.
        self.imu_stamps = []
        # Gazebo's raw ground truth, before sim_node touches it. The largest
        # |yaw rate| it reports is checked by the square test: gz-sim 8.11.0
        # published ~628 rad/s here (upstream #3834), and since the simulated
        # gyro now saturates like the real chip, the EKF no longer shows it.
        self.max_truth_rate = 0.0
        self.create_subscription(
            Odometry, "/gazebo/ground_truth",
            lambda m: setattr(self, "max_truth_rate",
                              max(self.max_truth_rate, abs(m.twist.twist.angular.z))),
            50)
        self.create_subscription(
            Imu, "/imu/data_raw",
            lambda m: self.imu_stamps.append(
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 100)
        # 20 Hz: inside hoverboard_bridge's 0.5 s cmd_timeout and the simulated
        # ESP32's 0.2 s watchdog, either of which would stop the wheels
        # mid-measurement in a way that reads like a physics fault.
        self.create_timer(0.05, self._republish)

    def _republish(self):
        message = Twist()
        message.linear.x, message.angular.z = self.command
        self._publisher.publish(message)

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
        """Wait for all three topics, THEN sit still. Not a sleep — a fresh node
        joining a graph this size takes seconds to discover them, and a fixed
        wait fails under load with a message that reads like a crashed node."""
        deadline = time.monotonic() + timeout_s
        self.command = (0.0, 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(t.newest is not None for t in (self.truth, self.ekf, self.odom)):
                break
        for name, track in (("/ground_truth", self.truth),
                            ("/odometry/filtered/local", self.ekf),
                            ("/odom", self.odom)):
            assert track.newest is not None, f"nothing on {name}"
        self.drive(0.0, 0.0, seconds)

    def sample(self):
        """(truth, ekf, odom) poses, all read at one sim-time instant."""
        moment = min(t.newest for t in (self.truth, self.ekf, self.odom))
        poses = tuple(t.at(moment) for t in (self.truth, self.ekf, self.odom))
        assert all(pose is not None for pose in poses)
        return poses

    def health(self, since_sim, since_wall, since_counts):
        """What the simulator actually delivered, as a one-line diagnosis.

        ⚠️ Bolted onto every assertion below because "the EKF is 1.37 m off"
        does not say WHY. That exact reading happened on gz-sim 8.11.0 with
        every rate nominal — which is what pointed AWAY from load and lost
        messages and, eventually, at the simulator itself (see the square test).
        Rates and RTF separate "the estimate is bad" from "its inputs never
        arrived" without another hour of bisecting.
        """
        span = max(self.get_clock().now().nanoseconds * 1e-9 - since_sim, 1e-9)
        wall = max(time.monotonic() - since_wall, 1e-9)
        rates = " ".join(
            f"{name} {(len(track.stamps) - count) / span:.1f}"
            for (name, track), count in zip(
                (("truth", self.truth), ("ekf", self.ekf), ("odom", self.odom)),
                since_counts[:3])
        )
        imu = (len(self.imu_stamps) - since_counts[3]) / span
        return (f"[sim {span:.1f} s / wall {wall:.1f} s, RTF {span / wall:.2f}; "
                f"Hz: {rates} imu {imu:.1f} "
                f"(nominal truth/odom 50, ekf 30, imu 100)]")

    def counts(self):
        return tuple(len(t.stamps) for t in (self.truth, self.ekf, self.odom)) + (
            len(self.imu_stamps),)


@pytest.fixture
def probe(ros, pose_track):
    node = Probe(pose_track)
    yield node
    node.command = (0.0, 0.0)
    node.destroy_node()


def displacement_error(estimate_start, estimate_end, truth_start, truth_end):
    """How far an estimate's DISPLACEMENT is from the true displacement.

    Displacements rather than absolute poses, so a constant offset between the
    two frames — `odom` starts wherever the EKF was initialised, `sim_world` at
    the model's spawn — does not get counted as estimation error.
    """
    moved = (estimate_end[0] - estimate_start[0], estimate_end[1] - estimate_start[1])
    truly = (truth_end[0] - truth_start[0], truth_end[1] - truth_start[1])
    return distance(moved, truly)


def test_the_ekf_tracks_the_truth_around_a_square_even_with_real_slip(gazebo, probe):
    """The same 8 m square test_localization.py drives, on real physics.

    Identical command profile on purpose — four ~2 m straights and four ~90 deg
    turns, no pauses — so the two numbers are comparable. Measured 2026-10-15
    (gz-sim 8.15.0, accelerometer out of the EKF), four fresh stacks: EKF
    0.073 / 0.074 / 0.075 / 0.078 m and 3.22 / 3.36 / 3.34 / 3.38 deg; wheel
    odometry 0.066-0.085 m. Kinematic world, same day: 0.080-0.082 m, 4.2-4.3 deg.

    ⚠️ This EKF bound was 2.5 m for a while — a divergence check, not an
    accuracy one — because on 2026-09-08 the same code also produced 0.487,
    1.368 and 1.522 m with the wheels at 0.080 m, RTF 1.00 and every rate
    nominal, and nobody knew why. STRONGEST CANDIDATE, not proven (nothing was
    recorded in those runs): gz-sim 8.11.0, which those runs used (dpkg log),
    and its OdometryPublisher bug (upstream #3834, fixed in 8.15.0). The pose
    quaternion can flip sign between 1 ms physics steps; the finite difference
    reads that as a full turn in one step (~6283 rad/s), and the 10-sample
    rolling mean publishes ~628 rad/s. On this square the flip falls near yaw
    240 deg, in the third turn. sim_node turned that omega into the gyro, and
    ekf_local trusts the gyro ~500x over the wheels for yaw rate: a heading
    step of whatever the spike integrates to, mod 2 pi — wheels untouched.
    SUFFICIENCY was shown with a stand-in (one extra, out-of-phase -314 rad/s
    message on /gazebo/ground_truth in turn 2): 0.076 / 5.63 / 0.41 / 0.076 m
    over four runs, wheels 0.07 m every time — the A3e signature.
    Two guards since: the simulated gyro saturates like the real chip (4.37
    rad/s at 250 dps), so such a spike can no longer reach the EKF at full
    strength (same stand-in: 0.050-0.080 m); and this test asserts directly
    that Gazebo never reports an impossible yaw rate.

    So slip costs roughly a centimetre over eight metres here, and the yaw error
    is unchanged — it is 0.1 deg/s of residual gyro bias integrated over the
    ~35 s the square takes, exactly as in the slip-free world. The estimate is
    still bias-dominated, not slip-dominated. test_slip_from_speeding_up_is_
    given_back_when_slowing_down below is why, and it is the load-bearing half
    of this result.
    """
    gazebo()
    probe.settle()
    truth0, ekf0, odom0 = probe.sample()
    sim0 = probe.get_clock().now().nanoseconds * 1e-9
    wall0 = time.monotonic()
    counts0 = probe.counts()

    for _ in range(4):
        probe.drive(0.4, 0.0, 5.0)      # ~2 m straight
        probe.drive(0.0, 0.5, 3.14)     # ~90 deg turn
    probe.drive(0.0, 0.0, 1.0)

    truth1, ekf1, odom1 = probe.sample()
    closure = distance(truth1, truth0)
    assert closure < 1.5, (
        f"the square did not close — the robot ended {closure:.2f} m from where "
        "it started, so it did not drive the shape this test is measuring")

    ekf_error = displacement_error(ekf0, ekf1, truth0, truth1)
    odom_error = displacement_error(odom0, odom1, truth0, truth1)
    ekf_yaw = math.degrees(angle_diff(ekf1[2] - ekf0[2], truth1[2] - truth0[2]))

    health = probe.health(sim0, wall0, counts0)

    # Slip is a wheel-odometry effect, and wheel odometry over this square is
    # steady to a few millimetres: 0.066-0.085 m across every run, glitched or
    # not. So this is where A3's question actually gets answered
    # — 8 m of driving with real slip costs the odometry under 0.1 m, and the
    # kinematic world's slip-free ~0.08 m was not a flattering number.
    assert odom_error < 0.2, (
        f"raw wheel odometry is {odom_error:.3f} m off after ~8 m with slip, "
        f"far past the ~0.08 m measured — slip has stopped cancelling, or the "
        f"wheel geometry has changed {health}")

    # The simulator itself first: if Gazebo reports a yaw rate no robot here can
    # have, every number below is measuring the simulator. 8.11.0 published
    # ~628 rad/s (docstring); the bridge caps commands at 1.5 rad/s.
    assert probe.max_truth_rate < 10.0, (
        f"Gazebo reported a yaw rate of {probe.max_truth_rate:.1f} rad/s on "
        f"/gazebo/ground_truth — impossible for this robot. A gz-sim "
        f"OdometryPublisher regression (upstream #3834)? {health}")

    # An ACCURACY bound: ~3x the measured 0.073-0.079 m (6 runs). Run-to-run
    # spread is ~0.002 m, so only a real change trips it.
    assert ekf_error < 0.25, (
        f"the EKF is {ekf_error:.3f} m off after ~8 m (measured 0.073-0.079), "
        f"with wheel odometry at {odom_error:.3f} m and yaw {ekf_yaw:.2f} deg. "
        f"Wheels fine but EKF not: suspect the yaw rate reaching the gyro "
        f"(see the docstring) {health}")
    # The residual gyro bias integrated: a FIXED 0.1 deg/s (sim_node, seeded
    # RNG) over the ~35 s this square takes — measured 3.2-3.4 deg (kinematic
    # world 4.3). 8 deg is ~2.4x that, room for timing jitter on a slow runner.
    assert ekf_yaw < 8.0, (
        f"the EKF is {ekf_yaw:.1f} deg off after ~8 m (measured 3.2-3.4) — more "
        f"than the residual gyro bias can integrate to {health}")


def test_slip_from_speeding_up_is_given_back_when_slowing_down(gazebo, probe):
    """Why the square above barely notices slip. THE SAME DRIVE, TWO WINDOWS.

    Stepping from rest the contact patch jumps to the commanded speed while the
    body is still still, so the wheels over-report. Stopping is the mirror
    image: the wheels are commanded to zero while the body is still moving, so
    they under-report by the same mechanism. Nothing here changes the driving —
    both cases step to 1 m/s and hold for four seconds — only whether the
    braking phase falls inside the measured window.

    Measured, three repeats each: +0.137 m ending at cruise, +0.013 m ending
    after the stop. Braking returns 91% of it.

    ⚠️ Read this as a warning, not an all-clear. It says spin-up slip is not a
    CUMULATIVE odometry error, which is good news for a robot that keeps
    starting and stopping. It says nothing about slip that only goes one way —
    a wheel spinning on loose ground, a robot pushing something, or the jam in
    test_gazebo_physics.py, where /odom invented 2.97 m in six seconds and no
    part of this stack could tell.
    """
    gazebo()
    probe.settle()

    def slip(include_the_stop):
        probe.drive(0.0, 0.0, 3.0)
        truth0, _, odom0 = probe.sample()
        probe.drive(1.0, 0.0, 4.0)                  # identical in both cases
        if include_the_stop:
            probe.drive(0.0, 0.0, 3.0)              # the braking skid, inside
        truth1, _, odom1 = probe.sample()
        if not include_the_stop:
            probe.drive(0.0, 0.0, 3.0)              # stop anyway, but outside
        return distance(odom1, odom0) - distance(truth1, truth0)

    accelerating = slip(include_the_stop=False)
    whole_cycle = slip(include_the_stop=True)

    assert accelerating > 0.10, (
        f"the accelerating window slipped only {accelerating:.4f} m; "
        "test_gazebo_physics.py measures 0.137 m for this exact command, so "
        "something upstream has changed")
    assert whole_cycle < 0.25 * accelerating, (
        f"braking gave back only {100 * (1 - whole_cycle / accelerating):.0f}% "
        f"of the {accelerating:.4f} m spin-up slip ({whole_cycle:.4f} m left "
        "over). If that is now real, spin-up slip has become a cumulative "
        "odometry error and the square test above should have got worse too")
    # Still positive, so the cancellation is close but not free.
    assert whole_cycle > 0.0, (
        f"a whole start-stop cycle ended {whole_cycle:.4f} m to the wheels' "
        "credit — braking is over-correcting, which no friction model does")
