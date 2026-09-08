"""ROS adapter for the Gazebo Sim physics model.

The ESP32 simulator still owns safety and command mixing; this backend only
translates the wheel targets it produces into Gazebo Twist commands, and reads
back the two things Gazebo knows that KinematicWorld cannot invent:

    /gazebo/wheel_odometry   what the WHEELS turned  -> hall feedback (_meas_l/r)
    /gazebo/ground_truth     where the robot ACTUALLY is -> pose, v, omega
    /gazebo/contacts         whether the chassis is touching something

⚠️ THOSE FIRST TWO ARE DIFFERENT NUMBERS AND THAT DIFFERENCE IS THE POINT.
DiffDrive's odometry is dead reckoning from the wheel joint angles — exactly
what the hall sensors give the real board — so it keeps counting distance while
the wheels spin on the spot. Ground truth comes from a separate
OdometryPublisher differentiating the model's true pose. Feeding the wheel
number into `pose` would make /ground_truth agree with /odom by construction
and quietly turn every localization measurement in this world into a tautology.
Measured on the pre-A3 model, whose wheels were buried inside the chassis and
touched nothing: wheel odometry 5.47 m, true pose 3.8e-9 m.

Slip measured through this backend (v = 0.5 m/s stepped from rest, wheel
friction mu = 0.5): the wheels over-report by 0.041 m, all of it during
spin-up, and it scales as v^2. test_gazebo_physics.py pins that.
"""

from __future__ import annotations

import math
import shutil
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_sim.gazebo_obstacles import obstacle_model_sdf

# The spawn goes through Gazebo's own UserCommands service rather than
# `ros2 run ros_gz_sim create`, which is a whole ROS node spun up and torn down
# per call. Both end at the same gz service.
SPAWN_TIMEOUT_S = 30.0


class GazeboBackend:
    """Backend compatible with ``hoverboard_bridge.esp32_sim.Backend``."""

    def __init__(self, node, wheel_radius=0.0825, wheel_separation=0.5,
                 board_units_per_rpm=1.0, command_topic="/gazebo/cmd_vel",
                 wheel_odometry_topic="/gazebo/wheel_odometry",
                 ground_truth_topic="/gazebo/ground_truth",
                 contacts_topic="/gazebo/contacts",
                 obstacles=(), world="empty") -> None:
        self._node = node
        self._radius = float(wheel_radius)
        self._separation = float(wheel_separation)
        self._units_per_rpm = float(board_units_per_rpm)
        self._publisher = node.create_publisher(Twist, command_topic, 10)
        node.create_subscription(Odometry, wheel_odometry_topic, self._on_wheel_odometry, 10)
        node.create_subscription(Odometry, ground_truth_topic, self._on_ground_truth, 10)
        self._lock = threading.Lock()
        self._meas_l = 0.0
        self._meas_r = 0.0
        self.pose = type("Pose", (), {"x": 0.0, "y": 0.0, "yaw": 0.0})()
        self.v = 0.0
        self.omega = 0.0
        self.accel_x = 0.0
        self.collision = False
        self._truth_seen = False
        self._last_truth_t = None

        # ros_gz_interfaces is only needed for the collision oracle, and only
        # the gazebo backend ever needs it. Importing it at module scope would
        # make the ROS-free unit tests of this file's maths depend on it.
        from ros_gz_interfaces.msg import Contacts
        node.create_subscription(Contacts, contacts_topic, self._on_contacts, 10)

        if obstacles:
            self._spawn_obstacles(obstacles, world)

    # ---- World setup ---------------------------------------------------------
    def _spawn_obstacles(self, obstacles, world: str) -> None:
        """Put the parsed obstacles into Gazebo, or fail loudly.

        Same list the OccupancyGrid is drawn from — see gazebo_obstacles.py.
        Raising rather than warning on failure is deliberate and it is the same
        rule this backend used to enforce by refusing obstacles outright: a run
        that publishes a map of obstacles the physics does not have is the
        phantom-obstacle bug, and it looks exactly like a working run.
        """
        if shutil.which("gz") is None:
            raise RuntimeError(
                "obstacles were requested for the gazebo backend but the `gz` "
                "CLI is not on PATH, so they cannot be spawned"
            )
        sdf = obstacle_model_sdf(obstacles)
        request = f"sdf: {sdf!r}, name: 'obstacles', allow_renaming: false"
        deadline = time.monotonic() + SPAWN_TIMEOUT_S
        last = ""
        while time.monotonic() < deadline:
            # Gazebo may not be up yet — sim_node and `gz sim` start together.
            result = subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "3000", "--req", request],
                capture_output=True, text=True,
            )
            last = (result.stdout + result.stderr).strip()
            if "data: true" in result.stdout:
                self._node.get_logger().info(
                    f"{len(obstacles)} engel Gazebo'ya spawn edildi")
                return
            time.sleep(1.0)
        raise RuntimeError(
            f"could not spawn obstacles into Gazebo world '{world}' within "
            f"{SPAWN_TIMEOUT_S:.0f} s — last reply: {last!r}"
        )

    # ---- Backend interface ---------------------------------------------------
    def step(self, target_l: float, target_r: float, dt: float):
        left_mps = self._units_to_mps(target_l)
        right_mps = self._units_to_mps(target_r)
        command = Twist()
        command.linear.x = 0.5 * (left_mps + right_mps)
        command.angular.z = (right_mps - left_mps) / self._separation
        self._publisher.publish(command)
        with self._lock:
            return self._meas_l, self._meas_r

    # ---- What the wheels turned ---------------------------------------------
    def _on_wheel_odometry(self, message: Odometry) -> None:
        """DiffDrive's dead reckoning -> the hall sensors' raw board units.

        Only the twist is used. Its pose is the wheels' own idea of where the
        robot is, which is what hoverboard_bridge is there to compute for
        itself out of these very numbers; taking it from here would hand the
        bridge the answer instead of testing it.
        """
        v = message.twist.twist.linear.x
        omega = message.twist.twist.angular.z
        left_mps = v - omega * self._separation * 0.5
        right_mps = v + omega * self._separation * 0.5
        with self._lock:
            self._meas_l = self._mps_to_units(left_mps)
            self._meas_r = self._mps_to_units(right_mps)

    # ---- Where the robot actually is ----------------------------------------
    def _on_ground_truth(self, message: Odometry) -> None:
        stamp = message.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        v = message.twist.twist.linear.x
        with self._lock:
            # The fake IMU reads accel_x, and it is not published by Gazebo
            # here, so it is differentiated from the true forward speed.
            # Guarded against a repeated or non-advancing stamp: dividing by a
            # zero dt would put an inf into /imu/data_raw and take the EKF with
            # it.
            if self._last_truth_t is not None and now > self._last_truth_t:
                self.accel_x = (v - self.v) / (now - self._last_truth_t)
            self._last_truth_t = now
            self.v = v
            self.omega = message.twist.twist.angular.z
            self.pose.x = message.pose.pose.position.x
            self.pose.y = message.pose.pose.position.y
            quaternion = message.pose.pose.orientation
            self.pose.yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
            )
            self._truth_seen = True

    def _on_contacts(self, message) -> None:
        """The chassis contact sensor: the honest source for /collision_truth.

        The sensor watches the chassis box ONLY. The wheels and the casters are
        in contact with the ground on every step of every run, so a sensor that
        included them would read "colliding" forever; the box has 9 cm of
        clearance, so anything it touches is something the robot ran into.
        """
        with self._lock:
            self.collision = len(message.contacts) > 0

    @property
    def ground_truth_seen(self) -> bool:
        """False until Gazebo has sent a pose.

        Without this, `pose` reads a flat (0, 0, 0) whether Gazebo is running,
        misconfigured, or bridged to the wrong topic — and a robot that never
        moves looks identical to one that has not started yet.
        """
        with self._lock:
            return self._truth_seen

    # ---- Units ---------------------------------------------------------------
    def _units_to_mps(self, units: float) -> float:
        rpm = units / self._units_per_rpm
        return rpm * 2.0 * math.pi * self._radius / 60.0

    def _mps_to_units(self, speed: float) -> float:
        rpm = speed * 60.0 / (2.0 * math.pi * self._radius)
        return rpm * self._units_per_rpm
