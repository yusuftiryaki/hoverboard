"""The kinematic world as a ROS node: ESP32 + wheels + IMU + GPS, no hardware.

    /cmd_vel ─► hoverboard_bridge ─pty─► Esp32Sim ─► KinematicWorld ─► ground truth
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

⚠️ NOT physics. No slip, no mass, no tipping. See world.py.
"""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

from hoverboard_bridge.esp32_sim import TX_PERIOD_S, Esp32Sim, PtyLink, step_once
from robot_sim.world import KinematicWorld

EARTH_RADIUS_M = 6378137.0
STANDARD_GRAVITY = 9.80665

# Istanbul-ish, matching qmc5883l_driver's fake bus. The horizontal component is
# what gives a heading; the vertical only matters once the robot tilts, which
# this flat kinematic world never does.
EARTH_NORTH_T = 26e-6
EARTH_DOWN_T = 36e-6


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_node")

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
        self._world = KinematicWorld(
            wheel_radius=p("wheel_radius").value,
            wheel_separation=p("wheel_separation").value,
            board_units_per_rpm=p("board_units_per_rpm").value,
        )
        self._link = PtyLink(p("link").value)
        self._esp = Esp32Sim(
            estop=p("estop").value,
            bump=p("bump").value,
            bump_file=p("bump_file").value,
        )
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

        self._truth_pub = self.create_publisher(Odometry, "ground_truth", 10)
        # imu/data_raw, matching mpu6050_driver: no orientation here.
        # imu_filter_madgwick fuses this with imu/mag into imu/data.
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)
        self._gps_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self._tf = TransformBroadcaster(self)

        self._last_tick = None
        self.create_timer(TX_PERIOD_S, self._tick)
        self.create_timer(1.0 / p("imu_rate_hz").value, self._publish_imu)
        self.create_timer(1.0 / p("mag_rate_hz").value, self._publish_mag)
        self.create_timer(1.0 / p("gps_rate_hz").value, self._publish_gps)

    # ---- The world -----------------------------------------------------------
    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = TX_PERIOD_S if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        step_once(self._esp, self._world, self._link, now, dt)
        self._publish_truth()

    def _publish_truth(self) -> None:
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

    # ---- Fake sensors --------------------------------------------------------
    def _publish_imu(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        # Same contract as the real driver: 6-axis, so no orientation at all.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.z = (
            self._world.omega + self._gyro_bias + self._rng.gauss(0.0, self._gyro_noise)
        )
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
        msg.magnetic_field.x = (
            -EARTH_NORTH_T * math.sin(yaw) + self._mag_residual[0]
            + self._rng.gauss(0.0, self._mag_noise)
        )
        msg.magnetic_field.y = (
            EARTH_NORTH_T * math.cos(yaw) + self._mag_residual[1]
            + self._rng.gauss(0.0, self._mag_noise)
        )
        msg.magnetic_field.z = (
            -EARTH_DOWN_T + self._mag_residual[2]
            + self._rng.gauss(0.0, self._mag_noise)
        )
        msg.magnetic_field_covariance[0] = -1.0
        self._mag_pub.publish(msg)

    def _publish_gps(self) -> None:
        dt = 1.0 / self.get_parameter("gps_rate_hz").value
        # Ornstein-Uhlenbeck: pulls back toward zero over gps_wander_tau, with
        # the driving noise scaled so the steady-state std lands on wander_sigma.
        drive = self._gps_wander_sigma * math.sqrt(2.0 / self._gps_wander_tau)
        for attr in ("_gps_bias_x", "_gps_bias_y"):
            bias = getattr(self, attr)
            bias += -bias / self._gps_wander_tau * dt + drive * math.sqrt(dt) * self._rng.gauss(0, 1)
            setattr(self, attr, bias)

        east = self._world.pose.x + self._gps_bias_x + self._rng.gauss(0, self._gps_noise_sigma)
        north = self._world.pose.y + self._gps_bias_y + self._rng.gauss(0, self._gps_noise_sigma)

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
