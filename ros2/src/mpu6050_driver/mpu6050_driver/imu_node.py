"""MPU6050 -> /imu/data, for the local EKF (roadmap step 5).

There is no MPU6050 driver in the Jazzy apt repo, so this is ours. It exists to
feed robot_localization's ekf_local a trustworthy yaw RATE — without it the EKF
runs on wheel odometry alone and the heading drifts away within a minute.

What this node deliberately does NOT publish: an absolute orientation. The chip
is 6-axis (docs/handoff.md decision 4) — it has gyros and accelerometers but no
magnetometer, so it has no heading reference at all. Integrating the gyro into a
yaw here and publishing it as `orientation` would be inventing information: it
would look plausible, the EKF would fuse it as an absolute measurement, and the
robot would confidently drive in the wrong direction. Instead
orientation_covariance[0] is set to -1, which is sensor_msgs/Imu's way of saying
"no orientation in this message" and is exactly what robot_localization checks.
The QMC5883L on the mast (not bought yet) is what will fill that gap.

Axes: samples are published raw, in the chip's own frame, as `imu_link`. The
mounting orientation belongs in the URDF's imu_joint rpy — robot_localization
rotates the data into base_link using tf. Do not "fix" axes here as well, or the
rotation gets applied twice.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

from mpu6050_driver.mpu6050 import MPU6050, WHO_AM_I_VALUE


class Mpu6050Node(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050")

        # ---- Parameters ------------------------------------------------------
        self.declare_parameter("i2c_bus", 1)          # Pi's GPIO2/3 bus
        self.declare_parameter("address", 0x68)       # 0x68 with AD0 low
        self.declare_parameter("frame_id", "imu_link")
        # 100 Hz is plenty for a 30 Hz EKF and leaves headroom for averaging.
        self.declare_parameter("rate_hz", 100.0)

        # +-4 g rather than the more sensitive +-2 g: this is an outdoor robot on
        # rough ground with rigid hub motors, and a bump that clips the range
        # produces a wrong reading rather than a noisy one. Drop to 2 if the
        # accelerometer turns out to be quiet in the field.
        self.declare_parameter("accel_range_g", 4)
        # The robot turns slowly; the most sensitive range is the right one.
        self.declare_parameter("gyro_range_dps", 250)
        # DLPF 3 = 44 Hz accel / 42 Hz gyro. Filters hub-motor vibration at the
        # cost of ~5 ms delay, which the EKF can absorb.
        self.declare_parameter("dlpf", 3)

        # Gyro bias is a few deg/s on every one of these chips and it drifts with
        # temperature, so it is measured at every startup rather than stored.
        self.declare_parameter("calibrate_on_start", True)
        self.declare_parameter("calibration_samples", 500)
        self.declare_parameter("gyro_bias", [0.0, 0.0, 0.0])

        # Covariance floors. The variance measured while standing still badly
        # UNDERSTATES the noise while driving (vibration, bumps), and a filter
        # that trusts the IMU too much is worse than one that trusts it too
        # little. Raise these if the EKF's output looks jumpy on rough ground.
        self.declare_parameter("gyro_variance_floor", 1e-4)
        self.declare_parameter("accel_variance_floor", 1e-2)

        self.declare_parameter("publish_temperature", True)
        # Dev aid: run against fake_bus.py with no hardware at all.
        self.declare_parameter("use_fake_bus", False)

        p = self.get_parameter
        self._frame_id = p("frame_id").value
        self._gyro_floor = p("gyro_variance_floor").value
        self._accel_floor = p("accel_variance_floor").value
        self._publish_temp = p("publish_temperature").value
        address = int(p("address").value)

        # ---- Bus -------------------------------------------------------------
        if p("use_fake_bus").value:
            from mpu6050_driver.fake_bus import FakeMPU6050Bus

            self.get_logger().warn(
                "using the SIMULATED I2C bus — this is a dev aid, not a sensor"
            )
            self._bus = FakeMPU6050Bus(address=address)
        else:
            # Imported lazily so the package still builds, tests and runs on a
            # machine with no I2C library installed.
            import smbus2

            self._bus = smbus2.SMBus(int(p("i2c_bus").value))

        self._imu = MPU6050(
            self._bus,
            address=address,
            accel_range_g=int(p("accel_range_g").value),
            gyro_range_dps=int(p("gyro_range_dps").value),
            dlpf=int(p("dlpf").value),
            sample_rate_hz=float(p("rate_hz").value),
        )

        self._probe_and_configure(address)

        # ---- State -----------------------------------------------------------
        self._gyro_bias = list(p("gyro_bias").value)
        self._gyro_var = [self._gyro_floor] * 3
        self._accel_var = [self._accel_floor] * 3
        self._read_errors = 0
        self._reads_ok = 0
        self._last_temp = float("nan")

        if p("calibrate_on_start").value:
            self._calibrate(int(p("calibration_samples").value))
        else:
            self.get_logger().warn(
                f"calibration skipped; using gyro_bias={self._gyro_bias} rad/s"
            )

        # ---- ROS interfaces --------------------------------------------------
        self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self._temp_pub = self.create_publisher(Temperature, "imu/temperature", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        rate = float(p("rate_hz").value)
        actual = self._imu.actual_sample_rate_hz()
        if abs(actual - rate) > 0.5:
            # SMPLRT_DIV is an integer divider of 1 kHz, so not every rate is
            # reachable. Say so rather than quietly sampling at a different rate.
            self.get_logger().warn(
                f"requested {rate} Hz, the chip will produce {actual:.1f} Hz "
                "(SMPLRT_DIV is an integer divider of 1 kHz)"
            )
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._diag_tick)
        self.get_logger().info(f"MPU6050 streaming at {actual:.1f} Hz on {self._frame_id}")

    # ---- Setup ---------------------------------------------------------------
    def _probe_and_configure(self, address: int) -> None:
        try:
            who = self._imu.who_am_i()
        except OSError as exc:
            self.get_logger().fatal(
                f"no I2C response at 0x{address:02x}: {exc}. Check wiring and "
                "`i2cdetect -y 1` (docs/deployment.md step 5)."
            )
            raise
        if who != WHO_AM_I_VALUE:
            # Clone modules do exist that report something else; refuse rather
            # than stream numbers from an unknown chip whose scale factors we
            # would be guessing.
            raise RuntimeError(
                f"WHO_AM_I is 0x{who:02x}, expected 0x{WHO_AM_I_VALUE:02x} — "
                "this is not an MPU6050 (or AD0 is wired to the other address)"
            )
        self._imu.reset()
        time_to_settle = 0.1
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=time_to_settle))
        self._imu.configure()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))

    def _calibrate(self, samples: int) -> None:
        """Average the gyro while stationary; also measure the noise."""
        self.get_logger().info(
            f"calibrating gyro bias over {samples} samples — THE ROBOT MUST BE "
            "COMPLETELY STILL"
        )
        gyro: List[List[float]] = [[], [], []]
        accel: List[List[float]] = [[], [], []]
        failures = 0
        while len(gyro[0]) < samples:
            try:
                sample = self._imu.read()
            except OSError:
                failures += 1
                if failures > samples // 10:
                    raise RuntimeError("too many I2C errors while calibrating")
                continue
            for axis in range(3):
                gyro[axis].append(sample.gyro[axis])
                accel[axis].append(sample.accel[axis])

        self._gyro_bias = [_mean(axis) for axis in gyro]
        self._gyro_var = [max(_variance(axis), self._gyro_floor) for axis in gyro]
        self._accel_var = [max(_variance(axis), self._accel_floor) for axis in accel]

        bias_dps = [b * 180.0 / math.pi for b in self._gyro_bias]
        self.get_logger().info(
            "gyro bias: "
            f"[{bias_dps[0]:+.2f}, {bias_dps[1]:+.2f}, {bias_dps[2]:+.2f}] deg/s"
        )

        # If the robot moved during calibration the bias is wrong, and a wrong
        # bias is worse than none: the EKF will steadily rotate the estimate.
        # Catch it here rather than three hours later in a field.
        worst = max(math.sqrt(v) for v in self._gyro_var) * 180.0 / math.pi
        if worst > 1.0:
            self.get_logger().error(
                f"gyro noise during calibration was {worst:.2f} deg/s — the robot "
                "was probably moving. The bias is unreliable; restart the node "
                "with the robot still."
            )

    # ---- Runtime -------------------------------------------------------------
    def _tick(self) -> None:
        try:
            sample = self._imu.read()
        except OSError as exc:
            self._read_errors += 1
            # A loose Dupont wire on a vibrating robot gives bursts of these.
            # Log, count, carry on — the EKF handles a gap; a dead node does not.
            self.get_logger().warn(f"I2C read failed: {exc}", throttle_duration_sec=5.0)
            return
        self._reads_ok += 1
        self._last_temp = sample.temperature

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # -1 in the first element = "this message carries no orientation".
        # robot_localization checks exactly this before trusting the field.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = sample.gyro[0] - self._gyro_bias[0]
        msg.angular_velocity.y = sample.gyro[1] - self._gyro_bias[1]
        msg.angular_velocity.z = sample.gyro[2] - self._gyro_bias[2]
        msg.linear_acceleration.x = sample.accel[0]
        msg.linear_acceleration.y = sample.accel[1]
        msg.linear_acceleration.z = sample.accel[2]

        for axis in range(3):
            msg.angular_velocity_covariance[axis * 4] = self._gyro_var[axis]
            msg.linear_acceleration_covariance[axis * 4] = self._accel_var[axis]
        self._imu_pub.publish(msg)

        if self._publish_temp:
            temp = Temperature()
            temp.header = msg.header
            temp.temperature = sample.temperature
            temp.variance = 0.0
            self._temp_pub.publish(temp)

    def _diag_tick(self) -> None:
        status = DiagnosticStatus(name="mpu6050: IMU", hardware_id="mpu6050")
        if self._reads_ok == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = "no successful reads"
        elif self._read_errors > 0:
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._read_errors} I2C read errors"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "streaming"
        status.values = [
            KeyValue(key="reads_ok", value=str(self._reads_ok)),
            KeyValue(key="read_errors", value=str(self._read_errors)),
            KeyValue(key="die_temp_c", value=f"{self._last_temp:.1f}"),
            KeyValue(
                key="gyro_bias_dps",
                value=", ".join(f"{b * 180.0 / math.pi:+.2f}" for b in self._gyro_bias),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> bool:
        try:
            self._bus.close()
        except Exception:
            pass
        return super().destroy_node()


def _mean(values: List[float]) -> float:
    return sum(values) / len(values)


def _variance(values: List[float]) -> float:
    mean = _mean(values)
    return sum((v - mean) ** 2 for v in values) / max(1, len(values) - 1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[Mpu6050Node] = None
    try:
        node = Mpu6050Node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
