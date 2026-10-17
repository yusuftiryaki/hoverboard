"""The simulated world as a ROS node: ESP32 + wheels + IMU + GPS, no hardware.

    /cmd_vel ─► hoverboard_bridge ─pty─► Esp32Sim ─►  backend  ─► ground truth
                      ▲                                    │
                      └────────── EspFeedback ◄────────────┘
                                                           ├─► /ground_truth  (the answer key)
                                                           ├─► /imu/data      (truth + bias + noise)
                                                           └─► /gps/fix       (truth + drift + noise)

Run it, point the bridge at /tmp/fake_esp32, and the whole localization stack has
a world to move in — with an answer key. That is the thing the real robot can
never give us: /ground_truth is what the EKF is TRYING to estimate, so the error
between them is measurable instead of a matter of opinion.

    ros2 run robot_sim sim_node
    ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
        use_localization:=true

⚠️ The IMU driver is bypassed here: this node publishes /imu/data itself rather
than driving mpu6050_driver. That driver's value is its register-level maths,
which fake_bus already unit-tests; re-testing it through a simulated I2C bus
would add nothing. hoverboard_bridge is NOT bypassed — the real bridge and the
real 0xABCD protocol stay in the loop, which is the whole reason Esp32Sim exists.

⚠️ TWO backends, chosen with `backend:=`:

    kinematic (default)  KinematicWorld — fast, deterministic, exact ground
                         truth, and NO physics: no slip, no mass, no tipping.
                         The wheels turn exactly as commanded and the robot
                         goes exactly where the maths says. See world.py.
    gazebo               GazeboBackend — real physics under the same simulated
                         ESP32: mass, friction, wheel slip, obstacles it
                         actually collides with. Needs `gz sim` running; see
                         robot_bringup/launch/gazebo.launch.py.

Everything below the backend line — the fake IMU, magnetometer and GPS, the
ground truth publisher, the obstacle map — is shared, and reads the backend's
`pose`, `v` and `omega`. In the Gazebo world those come from the model's TRUE
pose, never from its wheel odometry; conflating the two would make
/ground_truth agree with /odom by construction (see gazebo_backend.py).
"""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from hoverboard_bridge.esp32_sim import TX_PERIOD_S, Esp32Sim, PtyLink, step_once
from mpu6050_driver.mpu6050 import GYRO_LSB_PER_DPS, gyro_full_scale_rad_s
from qmc5883l_driver import earth_field
from robot_sim.battery_model import PackModel
from robot_sim.dock_model import (CameraMount, DockStation, mat_mul, matrix_from_quaternion,
                                  quaternion_from_matrix)
from robot_sim.gazebo_backend import GazeboBackend
from robot_sim.obstacle_map import DEFAULT_GRID, build_obstacle_grid, parse_obstacle_params
from robot_sim.world import bumper_pressed
from robot_sim.world import KinematicWorld

EARTH_RADIUS_M = 6378137.0
STANDARD_GRAVITY = 9.80665


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class SimNode(Node):
    # **kwargs so tests can construct the node in-process with
    # parameter_overrides instead of shelling out to `ros2 run`; the obstacle
    # wiring is checked that way in test_obstacle_node.py.
    def __init__(self, **kwargs) -> None:
        super().__init__("sim_node", **kwargs)

        self.declare_parameter("link", "/tmp/fake_esp32")
        self.declare_parameter("estop", False)
        self.declare_parameter("bump", False)
        self.declare_parameter("bump_file", "/tmp/fake_esp32.bump")
        # Determinism: a test that fails one run in ten is a test nobody trusts.
        self.declare_parameter("seed", 0)

        # ---- The world's TRUE geometry ---------------------------------------
        # These are what the robot IS. hoverboard_bridge's identically-named
        # params are what the Pi BELIEVES. Making them differ simulates a
        # miscalibrated robot — the honest state of things until step B4.
        self.declare_parameter("wheel_radius", 0.0825)
        self.declare_parameter("wheel_separation", 0.5)
        self.declare_parameter("board_units_per_rpm", 1.0)
        self.declare_parameter("slip_factor", 0.0)
        self.declare_parameter("backend", "kinematic")
        # Which Gazebo world to spawn obstacles into; must match the world name
        # in empty.sdf and the -world argument `ros_gz_sim create` is given.
        self.declare_parameter("gazebo_world", "empty")
        # Obstacles as flat [x0, y0, x1, y1, ...] + [r0, r1, ...]: ROS
        # parameters cannot hold a list of tuples.
        # ⚠️ Declared BY TYPE with no default, not as `[]`. An empty list has no
        # inferable type: rclpy reads it as a BYTE_ARRAY and then REJECTS the
        # double array you actually pass — the same untyped-empty-list trap as
        # nav2.yaml's `plugins: []`. Unset means an obstacle-free world, which
        # is what every pre-A3c test expects.
        self.declare_parameter("obstacle_centers", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("obstacle_radii", Parameter.Type.DOUBLE_ARRAY)
        # UNSURVEYED obstacles: physically there, absent from the published map.
        # This is not the phantom-obstacle bug — that was the opposite, a map
        # the physics did not share. This is the real world: the tree nobody
        # entered into the survey. The costmaps cannot see it (nothing observes
        # obstacles at runtime, nav2.yaml explains why), so Nav2 drives into it.
        self.declare_parameter("unsurveyed_obstacle_centers", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("unsurveyed_obstacle_radii", Parameter.Type.DOUBLE_ARRAY)
        # Keep in step with nav2.yaml's costmap robot_radius — see world.py.
        self.declare_parameter("robot_radius", 0.4)
        # false = the robot's own site_map_server publishes /obstacle_map from
        # a site file instead (robot.launch.py use_site_map). There must be
        # exactly one publisher: two latched maps on one topic do not error,
        # the costmap just keeps whichever arrived last.
        self.declare_parameter("publish_obstacle_map", True)
        # true = the front bumper presses when the robot touches an obstacle
        # ahead (world.bumper_pressed), so the ESP32's veto and everything
        # downstream of /bumper see real contacts. false (default) keeps the
        # old behaviour — bump / bump_file only — that the jam test documents.
        # ⚠️ The station body never presses it here: whether the real bumper
        # meets the station when docking is an SP6 design question.
        self.declare_parameter("bumper_from_obstacles", False)
        # A cheap rendered camera on camera/image_raw (sim_camera.py) for the
        # image pipeline — the bumper's snapshot, the web page's live view.
        # 0 = off (the default: the tests that do not look at images should
        # not pay for rendering). NOT for anything vision is judged on.
        self.declare_parameter("camera_rate_hz", 0.0)
        self.declare_parameter("camera_width", 320)
        self.declare_parameter("camera_height", 240)

        # ---- Battery (SP5) -----------------------------------------------------
        # The pack the robot really has. battery_monitor's capacity_ah is what
        # the Pi BELIEVES; a test that wants a fast-draining pack sets both.
        self.declare_parameter("battery_capacity_ah", 4.4)
        self.declare_parameter("battery_initial_soc", 0.8)
        # The mainboard's standby draw (battery_model.py). 0.0 in tests that
        # need the SoC at mission start not to depend on how long Nav2 took.
        self.declare_parameter("battery_idle_current_a", 0.2)

        # ---- Charging station (SP3/SP4) ------------------------------------------
        # [x, y, yaw] in sim_world: where base_link sits when docked. Unset = no
        # station. Declared by type for the same reason as the obstacles.
        self.declare_parameter("dock_pose", Parameter.Type.DOUBLE_ARRAY)
        # false = the marker is there but the camera never decodes it (dirt,
        # glare, night): the failure path the design names.
        self.declare_parameter("dock_tag_visible", True)
        # The station is a fixed structure, so it belongs on the survey map
        # like any surveyed obstacle — drawn where the SURVEY says it is
        # (docks.yaml's pose; unset = surveyed exactly), while the physics puts
        # it at dock_pose. false = nobody mapped it: measured, a robot coming
        # from behind then drives into it (test_docking_sweep.py).
        self.declare_parameter("dock_in_survey", True)
        self.declare_parameter("dock_surveyed_pose", Parameter.Type.DOUBLE_ARRAY)
        # What image_proc's track_marker_node manages on a Pi 4 at 640x480.
        self.declare_parameter("dock_detection_rate_hz", 15.0)
        # Position noise grows with range (pixels per metre shrink); the angle
        # noise is the marker's yaw, the term that steers the final approach.
        self.declare_parameter("dock_detection_noise_per_m", 0.01)
        self.declare_parameter("dock_detection_angle_noise", 0.02)

        # ---- Fake IMU --------------------------------------------------------
        # This publishes what mpu6050_driver WOULD PUBLISH, not what the chip
        # emits: the driver is bypassed here, so its output contract is what has
        # to be modelled. The chip's raw few-deg/s bias is the driver's problem
        # and fake_bus already tests that it gets removed.
        #
        # What survives calibration is a small RESIDUAL — averaging error plus
        # temperature drift over a session — and it matters enormously: with no
        # magnetometer, nothing observes absolute heading, so any residual
        # integrates forever. 0.1 deg/s is ~3.5 deg of drift per minute of
        # driving. That is not a sim artefact; that is exactly why handoff
        # decision 4 calls the magnetometer the highest-value purchase.
        self.declare_parameter("imu_gyro_residual_bias_dps", 0.1)
        self.declare_parameter("imu_gyro_noise_dps", 0.05)
        self.declare_parameter("imu_accel_noise", 0.05)
        self.declare_parameter("imu_rate_hz", 100.0)
        # The real chip's range (robot_bringup/config/mpu6050.yaml). The fake
        # gyro saturates there exactly as the int16 registers do.
        self.declare_parameter("imu_gyro_range_dps", 250)

        # ---- Fake magnetometer -----------------------------------------------
        # Publishes what qmc5883l_driver WOULD PUBLISH: the field AFTER hard/soft
        # iron correction. Same reasoning as the gyro bias above — the driver is
        # bypassed here, so its output contract is what gets modelled. Emitting
        # the chip's raw hard-iron offset with nothing to remove it is exactly
        # the mistake that drifted the estimate 100 degrees in A1.
        self.declare_parameter("mag_rate_hz", 50.0)
        # What survives calibration: an imperfect fit, plus the robot's own field
        # changing with motor current. It biases the heading, and unlike the gyro
        # it does NOT accumulate — a compass error stays an error.
        self.declare_parameter("mag_residual_hard_iron_ut", [0.5, -0.3, 0.2])
        self.declare_parameter("mag_noise_ut", 0.2)

        # ---- Fake GPS --------------------------------------------------------
        self.declare_parameter("gps_rate_hz", 5.0)      # NEO-6M default
        self.declare_parameter("datum_lat", 41.0)       # TODO: your actual site
        self.declare_parameter("datum_lon", 29.0)
        # Real GPS error is NOT white noise — it wanders, correlated over minutes
        # (ionosphere, multipath, satellite geometry). White noise would average
        # out beautifully in the EKF and make our localization look far better
        # than it will be. So: a slow Ornstein-Uhlenbeck wander plus a little
        # white noise on top. handoff decision 5: NEO-6M is 2.5-5 m.
        self.declare_parameter("gps_wander_sigma", 2.0)   # steady-state std, m
        self.declare_parameter("gps_wander_tau", 60.0)    # correlation time, s
        self.declare_parameter("gps_noise_sigma", 1.0)    # white, m

        p = self.get_parameter
        self._rng = random.Random(int(p("seed").value))
        # Parsed ONCE and shared: the world collides against these objects and
        # the published map draws these same objects. Parsing twice would be
        # two models of one world, which is precisely how A6 happened.
        self._obstacles = parse_obstacle_params(
            self._unset_as_empty("obstacle_centers"),
            self._unset_as_empty("obstacle_radii"),
        )
        self._unsurveyed = parse_obstacle_params(
            self._unset_as_empty("unsurveyed_obstacle_centers"),
            self._unset_as_empty("unsurveyed_obstacle_radii"),
        )
        # What the bumper can meet: every obstacle, surveyed or not, but not
        # the station body (appended to _unsurveyed below).
        self._bumpable = self._obstacles + self._unsurveyed
        dock = self._unset_as_empty("dock_pose")
        self._station = None
        # Drawn on the published map, never collided with: the survey's idea
        # of an obstacle whose physical twin lives in _unsurveyed.
        self._map_only = ()
        if dock:
            if len(dock) != 3:
                raise ValueError(f"dock_pose must be [x, y, yaw], got {dock}")
            self._station = DockStation(*dock)
            robot_radius = p("robot_radius").value
            # The body physically stops a robot that drives in too far.
            self._unsurveyed = self._unsurveyed + (
                self._station.stop_obstacle(robot_radius),)
            if p("dock_in_survey").value:
                surveyed = self._unset_as_empty("dock_surveyed_pose") or dock
                if len(surveyed) != 3:
                    raise ValueError(f"dock_surveyed_pose must be [x, y, yaw], got {surveyed}")
                self._map_only = (DockStation(*surveyed).stop_obstacle(robot_radius),)
        if p("backend").value == "gazebo":
            if p("slip_factor").value:
                # Slip is what this backend is FOR — it comes out of friction,
                # mass and load transfer, and hoverbot.sdf's wheel mu is the
                # knob. Silently ignoring a slip_factor here would let a test
                # think it had configured slip when the number went nowhere.
                raise ValueError(
                    "slip_factor is a KinematicWorld knob and does nothing "
                    "under the gazebo backend, where slip comes from the wheel "
                    "friction in hoverbot.sdf"
                )
            # ⚠️ The obstacles go to GAZEBO, from the same tuple the map is
            # drawn from. This used to raise instead: the backend could not
            # honour the parameter, and warning-and-carrying-on would have
            # published a map of obstacles the physics drove straight through —
            # the phantom-obstacle bug with a warning in front of it that
            # nobody reads. Rendering both from one list (gazebo_obstacles.py)
            # is what finally makes the map and the physics unable to disagree,
            # which is what the refusal was standing in for.
            self._world = GazeboBackend(
                self,
                wheel_radius=p("wheel_radius").value,
                wheel_separation=p("wheel_separation").value,
                board_units_per_rpm=p("board_units_per_rpm").value,
                # Both kinds are physically there; only the map tells them
                # apart. Same split as the kinematic world below.
                obstacles=self._obstacles + self._unsurveyed,
                world=p("gazebo_world").value,
            )
            self.get_logger().info("Gazebo fizik backend'i seçildi")
        else:
            self._world = KinematicWorld(
                wheel_radius=p("wheel_radius").value,
                wheel_separation=p("wheel_separation").value,
                board_units_per_rpm=p("board_units_per_rpm").value,
                slip_factor=p("slip_factor").value,
                # Both kinds collide identically; only the map tells them apart.
                obstacles=self._obstacles + self._unsurveyed,
                robot_radius=p("robot_radius").value,
            )
        # Validated BEFORE the pty exists: raising after it would leave the
        # link symlink dangling (main() never gets a node to destroy).
        gyro_range = int(p("imu_gyro_range_dps").value)
        if gyro_range not in GYRO_LSB_PER_DPS:
            raise ValueError(f"imu_gyro_range_dps must be one of "
                             f"{sorted(GYRO_LSB_PER_DPS)}, got {gyro_range}")
        self._gyro_full_scale = gyro_full_scale_rad_s(gyro_range)
        self._link = PtyLink(p("link").value)
        self._esp = Esp32Sim(
            estop=p("estop").value,
            bump=p("bump").value,
            bump_file=p("bump_file").value,
        )
        if p("bumper_from_obstacles").value:
            radius = p("robot_radius").value
            self._esp.bump_source = lambda: bumper_pressed(
                self._world.pose, radius, self._bumpable)
        self.get_logger().info(
            f"simulated ESP32 on {self._link.link_path} — point the bridge at it: "
            f"-p port:={self._link.link_path}"
        )

        self._gyro_bias = math.radians(p("imu_gyro_residual_bias_dps").value)
        self._gyro_noise = math.radians(p("imu_gyro_noise_dps").value)
        self._accel_noise = p("imu_accel_noise").value
        self._gps_wander_sigma = p("gps_wander_sigma").value
        self._gps_wander_tau = p("gps_wander_tau").value
        self._gps_noise_sigma = p("gps_noise_sigma").value
        self._datum_lat = p("datum_lat").value
        self._datum_lon = p("datum_lon").value
        self._gps_bias_x = 0.0
        self._gps_bias_y = 0.0

        self._mag_residual = [v * 1e-6 for v in p("mag_residual_hard_iron_ut").value]
        self._mag_noise = p("mag_noise_ut").value * 1e-6

        self._pack = PackModel(capacity_ah=p("battery_capacity_ah").value,
                               soc=p("battery_initial_soc").value,
                               idle_current_a=p("battery_idle_current_a").value)
        self._esp.bat_voltage = round(self._pack.open_circuit_voltage * 100.0)
        self._tag_visible = bool(p("dock_tag_visible").value)
        self._tag_noise = p("dock_detection_noise_per_m").value
        self._tag_angle_noise = p("dock_detection_angle_noise").value
        self._camera: CameraMount | None = None
        self._gps_offset: tuple[float, float] | None = None

        self._truth_pub = self.create_publisher(Odometry, "ground_truth", 10)
        # ⚠️ An ORACLE, like /ground_truth: the real robot has nothing that can
        # publish this. Nothing in the robot stack may subscribe to it — it
        # exists so tests can ask "did it actually hit something?" instead of
        # inferring it from a frozen pose. Named _truth for that reason.
        self._collision_pub = self.create_publisher(Bool, "collision_truth", 10)
        # imu/data_raw, matching mpu6050_driver: no orientation here.
        # imu_filter_madgwick fuses this with imu/mag into imu/data.
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)
        self._gps_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        # TRANSIENT_LOCAL: the map is published once and latched, so a costmap
        # that subscribes later still receives it.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._obstacle_map_pub = self.create_publisher(OccupancyGrid, "obstacle_map", map_qos)
        self._tf = TransformBroadcaster(self)
        # Pack current, for battery_monitor's fake INA228 (fake_battery:=true).
        self._current_pub = self.create_publisher(Float64, "sim/battery_current", 10)
        # What image_proc's track_marker_node would publish (nav2.launch.py
        # wires that node's tracked_pose to the same name on the robot).
        self._dock_pose_pub = self.create_publisher(PoseStamped, "detected_dock_pose", 10)
        # ⚠️ An ORACLE like /collision_truth: are the contacts touching.
        self._contact_pub = self.create_publisher(Bool, "dock_contact_truth", 10)
        # The camera mount is read off tf rather than copied: robot.urdf.xacro
        # is the one description of where the camera is.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ⚠️ ALWAYS, even with no obstacles. Nav2's StaticLayer waits for a map
        # before it will let its costmap become usable, so a simulator that
        # stays quiet does not mean "no obstacles" — it means the planner never
        # starts and every goal aborts. Measured: status 6 on every goal until
        # this became unconditional. An empty grid is the honest a priori map
        # of a field with nothing surveyed in it, which is what the costmaps
        # already assumed before A3c.
        if self.get_parameter("publish_obstacle_map").value:
            self._publish_obstacle_map()

        self._last_tick = None
        self.create_timer(TX_PERIOD_S, self._tick)
        rate = self.get_parameter("camera_rate_hz").value
        if rate > 0.0:
            self._image_pub = self.create_publisher(Image, "camera/image_raw", 2)
            self.create_timer(1.0 / rate, self._publish_camera)
        self.create_timer(1.0 / p("imu_rate_hz").value, self._publish_imu)
        self.create_timer(1.0 / p("mag_rate_hz").value, self._publish_mag)
        self.create_timer(1.0 / p("gps_rate_hz").value, self._publish_gps)
        if self._station is not None:
            self.create_timer(1.0 / p("dock_detection_rate_hz").value, self._publish_dock)

    # ---- The world -----------------------------------------------------------
    def _unset_as_empty(self, name: str) -> list:
        """Read a type-declared array parameter that may never have been set.

        Reading an uninitialized one raises rather than returning [], so "no
        obstacles given" gets said once, here, instead of at each call site.
        """
        try:
            return list(self.get_parameter(name).value)
        except ParameterUninitializedException:
            return []

    def _publish_obstacle_map(self) -> None:
        """Latch the world's obstacles as an OccupancyGrid for Nav2's costmaps.

        ⚠️ frame_id is `map`, and that is a DECISION rather than a relabel. The
        coordinates here are ground truth, which lives in `sim_world`; `map` is
        anchored to the navsat datum and carries its GPS error. Publishing them
        as `map` asserts that these obstacles were SURVEYED IN THE SAME DATUM
        the robot navigates in — a field's known rocks, entered in the same
        coordinates the GPS reports. That is how a prior map is actually made,
        and it makes the datum error common-mode: it shifts the map and the
        robot's estimate of itself together, so it cancels rather than showing
        up as survey error.

        What this does NOT model is a map the robot builds from its own
        sensors, where the error is the live localization error and does NOT
        cancel. That needs a simulated range sensor, and the inventory has none
        (wiring-map section 6). So Nav2 gets an a priori map — which is what
        the A3c slice asks for, and no more.
        """
        spec = DEFAULT_GRID
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = spec.resolution
        message.info.width = spec.width
        message.info.height = spec.height
        message.info.origin.position.x = spec.origin_x
        message.info.origin.position.y = spec.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = build_obstacle_grid(self._obstacles + self._map_only, spec)
        self._obstacle_map_pub.publish(message)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = TX_PERIOD_S if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        step_once(self._esp, self._world, self._link, now, dt)
        self._step_battery(dt)
        self._publish_truth()
        # Both backends carry a real `collision`: KinematicWorld sets it when
        # the circle test refuses a step, GazeboBackend from hoverbot.sdf's
        # chassis contact sensor. The getattr default is only for a backend
        # that has yet to grow one — it is a fallback, not the gazebo case.
        self._collision_pub.publish(Bool(data=getattr(self._world, "collision", False)))

    def _publish_truth(self) -> None:
        # ⚠️ Say nothing rather than say (0, 0, 0). The gazebo backend's pose is
        # whatever Gazebo last sent, and before the first message that is the
        # origin — indistinguishable from a robot that is up and not moving.
        # An answer key that reads "at the origin, stationary" while the stack
        # is still connecting is worse than no answer key: every localization
        # measurement taken against it is measured against a constant. The
        # kinematic world has no such gap and reports True.
        if not getattr(self._world, "ground_truth_seen", True):
            return

        stamp = self.get_clock().now().to_msg()
        pose = self._world.pose

        msg = Odometry()
        msg.header.stamp = stamp
        # ⚠️ `sim_world`, NOT `map`. They are different frames and conflating them
        # silently poisons every measurement taken with GPS on: navsat_transform's
        # `map` is anchored at the FIRST GPS FIX, so it sits a couple of metres
        # from the sim's true origin — that offset IS the GPS's absolute error.
        # This published "map" until it was caught comparing ground truth against
        # ekf_global and finding metres of disagreement that were really just two
        # different origins wearing the same name.
        msg.header.frame_id = "sim_world"
        msg.child_frame_id = "base_link_truth"
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation = yaw_to_quaternion(pose.yaw)
        msg.twist.twist.linear.x = self._world.v
        msg.twist.twist.angular.z = self._world.omega
        self._truth_pub.publish(msg)

        # A separate frame, never base_link: publishing the true pose as
        # map->base_link would fight the EKF for the same transform and quietly
        # make a broken filter look perfect in RViz. sim_world is likewise
        # disconnected from the robot's tf tree on purpose — the answer key must
        # not be reachable from the frames the robot reasons in.
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "sim_world"
        tf.child_frame_id = "base_link_truth"
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.rotation = yaw_to_quaternion(pose.yaw)
        self._tf.sendTransform(tf)

    def _step_battery(self, dt: float) -> None:
        contact = (self._station is not None
                   and getattr(self._world, "ground_truth_seen", True)
                   and self._station.in_contact(self._world.pose))
        voltage, current = self._pack.step(dt, self._world.v, self._world.omega, contact)
        # Reported on the next feedback frame — the board samples, the ESP32
        # forwards, a 20 ms lag nobody downstream can see.
        self._esp.bat_voltage = round(voltage * 100.0)
        self._current_pub.publish(Float64(data=current))
        if self._station is not None:
            self._contact_pub.publish(Bool(data=contact))

    def _gps_mount(self) -> tuple[float, float] | None:
        """Where the antenna sits in base_link (x, y), off tf like the camera.

        ⚠️ The fix must be the ANTENNA's position, not base_link's. The real
        NEO-6M measures where it is — on the mast, 0.15 m behind base_link
        (robot.urdf.xacro) — and navsat_transform subtracts that offset to
        recover base_link. Reporting base_link here (as this sim did until the
        site-map stack test caught it) made navsat_transform subtract an offset
        that was never there: `map` came out shifted 0.15 m along the robot's
        heading at the first fix, and every later fix was off by 0.15 m in a
        direction that turned with the robot.
        """
        if self._gps_offset is None:
            try:
                tf = self._tf_buffer.lookup_transform("base_link", "gps_link", Time())
            except TransformException:
                return None            # robot_state_publisher not up yet
            self._gps_offset = (tf.transform.translation.x, tf.transform.translation.y)
        return self._gps_offset

    def _publish_camera(self) -> None:
        from robot_sim.sim_camera import render   # numpy only when the camera is on
        mount = self._camera_mount()
        if mount is None:
            return
        p = self.get_parameter
        width, height = int(p("camera_width").value), int(p("camera_height").value)
        # Heights are guesses: small things knee-high, wide ones tree-trunk tall.
        solids = [(o.x, o.y, o.radius, 0.4 if o.radius < 0.4 else 1.5)
                  for o in self._obstacles + self._unsurveyed]
        pose = self._world.pose
        pixels = render(width, height, mount.hfov, mount.translation, mount.rotation,
                        (pose.x, pose.y, pose.yaw), solids)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_link"
        msg.height, msg.width = height, width
        msg.encoding = "rgb8"
        msg.step = width * 3
        msg.data = pixels.tobytes()
        self._image_pub.publish(msg)

    def _camera_mount(self) -> CameraMount | None:
        if self._camera is None:
            try:
                tf = self._tf_buffer.lookup_transform("base_link", "camera_optical_link",
                                                      Time())
            except TransformException:
                return None            # robot_state_publisher not up yet
            t, q = tf.transform.translation, tf.transform.rotation
            self._camera = CameraMount(translation=(t.x, t.y, t.z),
                                       rotation=matrix_from_quaternion(q.x, q.y, q.z, q.w))
        return self._camera

    def _publish_dock(self) -> None:
        """The marker's pose in the camera's optical frame, when it is in view.

        ⚠️ Computed from the TRUE pose. That is the point: it is a relative
        measurement, so docking_server turns it into odom through the EKF's
        transform and the dock lands where the robot's own estimate says —
        which is how the survey error of the dock database gets corrected."""
        if not self._tag_visible or not getattr(self._world, "ground_truth_seen", True):
            return
        mount = self._camera_mount()
        if mount is None:
            return
        seen = self._station.tag_in_camera(self._world.pose, mount)
        if seen is None:
            return
        (x, y, z), (qx, qy, qz, qw) = seen
        distance = math.sqrt(x * x + y * y + z * z)
        sigma = self._tag_noise * distance
        rotation = matrix_from_quaternion(qx, qy, qz, qw)
        # Yaw error of the marker about the camera's vertical (optical y).
        a = self._rng.gauss(0.0, self._tag_angle_noise)
        about_y = ((math.cos(a), 0.0, math.sin(a)), (0.0, 1.0, 0.0),
                   (-math.sin(a), 0.0, math.cos(a)))
        qx, qy, qz, qw = quaternion_from_matrix(mat_mul(about_y, rotation))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_link"
        msg.pose.position.x = x + self._rng.gauss(0.0, sigma)
        msg.pose.position.y = y + self._rng.gauss(0.0, sigma)
        msg.pose.position.z = z + self._rng.gauss(0.0, sigma)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self._dock_pose_pub.publish(msg)

    # ---- Fake sensors --------------------------------------------------------
    def _publish_imu(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        # Same contract as the real driver: 6-axis, so no orientation at all.
        msg.orientation_covariance[0] = -1.0
        rate = self._world.omega + self._gyro_bias + self._rng.gauss(0.0, self._gyro_noise)
        # Saturate like the chip. Not cosmetic: gz-sim 8.11.0 could feed ~628
        # rad/s through here for one sample, and ekf_local, trusting the gyro
        # ~500x over the wheels, keeps the heading step that integrates to — a
        # stand-in spike left up to 161 deg (A3e).
        msg.angular_velocity.z = max(-self._gyro_full_scale,
                                     min(self._gyro_full_scale, rate))
        msg.linear_acceleration.x = self._world.accel_x + self._rng.gauss(0.0, self._accel_noise)
        msg.linear_acceleration.z = STANDARD_GRAVITY + self._rng.gauss(0.0, self._accel_noise)
        for axis in range(3):
            msg.angular_velocity_covariance[axis * 4] = max(self._gyro_noise ** 2, 1e-4)
            msg.linear_acceleration_covariance[axis * 4] = max(self._accel_noise ** 2, 1e-2)
        self._imu_pub.publish(msg)

    def _publish_mag(self) -> None:
        """The earth's field in the robot's frame, post-calibration.

        REP-103: yaw 0 = facing east, so the earth's horizontal field (pointing
        north) lies along +y. Turning the robot by yaw rotates the field by -yaw
        in the body frame — which is precisely the signal that makes the heading
        observable, and the thing the 6-axis IMU could never provide.
        """
        yaw = self._world.pose.yaw
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mag_link"
        # ⚠️ The field is NOT recomputed here. This node used to carry its own
        # copy of the rotation, qmc5883l_driver's fake_bus carried another, and
        # both had the same sign flipped — the mirrored earth of A6, which cost
        # weeks of blaming Nav2. One definition, in earth_field; this node only
        # adds the residual hard iron and the noise on top.
        truth = earth_field.field_in_body_frame(yaw)
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = (
            component + residual + self._rng.gauss(0.0, self._mag_noise)
            for component, residual in zip(truth, self._mag_residual)
        )
        msg.magnetic_field_covariance[0] = -1.0
        self._mag_pub.publish(msg)

    def _publish_gps(self) -> None:
        mount = self._gps_mount()
        if mount is None:
            return                     # no fix yet rather than a wrong one
        dt = 1.0 / self.get_parameter("gps_rate_hz").value
        # Ornstein-Uhlenbeck: pulls back toward zero over gps_wander_tau, with
        # the driving noise scaled so the steady-state std lands on wander_sigma.
        drive = self._gps_wander_sigma * math.sqrt(2.0 / self._gps_wander_tau)
        for attr in ("_gps_bias_x", "_gps_bias_y"):
            bias = getattr(self, attr)
            bias += -bias / self._gps_wander_tau * dt + drive * math.sqrt(dt) * self._rng.gauss(0, 1)
            setattr(self, attr, bias)

        pose = self._world.pose
        antenna_x = pose.x + mount[0] * math.cos(pose.yaw) - mount[1] * math.sin(pose.yaw)
        antenna_y = pose.y + mount[0] * math.sin(pose.yaw) + mount[1] * math.cos(pose.yaw)
        east = antenna_x + self._gps_bias_x + self._rng.gauss(0, self._gps_noise_sigma)
        north = antenna_y + self._gps_bias_y + self._rng.gauss(0, self._gps_noise_sigma)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        # Flat-earth around the datum. Fine over the tens of metres this robot
        # will ever cover; navsat_transform does the real projection anyway.
        msg.latitude = self._datum_lat + math.degrees(north / EARTH_RADIUS_M)
        msg.longitude = self._datum_lon + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(self._datum_lat)))
        )
        msg.altitude = 0.0
        # Report the white noise only. A receiver cannot see its own wander —
        # claiming the true total error here would hand the EKF information the
        # real NEO-6M never provides.
        var = self._gps_noise_sigma ** 2
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var * 4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SimNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
