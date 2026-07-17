"""QMC5883L -> /imu/mag, the robot's only absolute heading reference.

Why this package exists: the MPU6050 is 6-axis. Gyros measure rotation RATE, so
integrating them gives a heading that drifts forever with no way to correct it —
A1 measured that drift, and A2 found it breaks GPS waypoint following outright
(ekf_global's yaw is unobservable, so GPS position updates yank it around, and it
publishes map->odom, so every Nav2 goal rotates with it). This chip is the fix.

It does NOT publish an orientation. It publishes the magnetic field vector;
imu_filter_madgwick fuses that with the MPU6050's gyro and accelerometer into
/imu/data with a real orientation. Tilt compensation lives there, which is why
this node does not attempt a heading of its own: a bare compass reading is only a
heading when the robot is level, and this one drives on grass.

⚠️ CALIBRATION IS NOT OPTIONAL. Uncalibrated, this chip measures the robot's own
steel and motor currents at least as much as the earth (handoff decision 4: mount
it on the mast, >30 cm up, away from the motor cables). An uncalibrated
magnetometer is worse than none: it is confidently wrong in a direction that
rotates with the robot. See docs/mag-calibration.md.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from qmc5883l_driver.qmc5883l import CHIP_ID_VALUE, QMC5883L


class Qmc5883lNode(Node):
    def __init__(self) -> None:
        super().__init__("qmc5883l")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x0D)
        # The mast frame from the URDF. Its rpy there describes how the chip is
        # really mounted; do not re-fix axes here or the rotation lands twice.
        self.declare_parameter("frame_id", "mag_link")
        self.declare_parameter("rate_hz", 50.0)

        # 2 Gauss: the earth is ~0.5 G, so this keeps the resolution. If the
        # OVL flag starts firing (watch /diagnostics), the mount is picking up
        # too much of the robot — move it before reaching for the 8 G range,
        # because a saturating compass is a mounting problem, not a range one.
        self.declare_parameter("range_gauss", 2)
        self.declare_parameter("odr_hz", 100)
        self.declare_parameter("osr", 512)

        # ---- Calibration -----------------------------------------------------
        # HARD IRON: a constant field the robot carries with it (magnetised
        # steel, DC in nearby wiring). It shifts the centre of the sphere the
        # readings trace out. In tesla, subtracted from every sample.
        self.declare_parameter("hard_iron", [0.0, 0.0, 0.0])
        # SOFT IRON: nearby ferrous material distorts the sphere into an
        # ellipsoid. This is the diagonal scale that pulls it back — the full
        # correction is a 3x3 matrix, but for a mast-mounted sensor the diagonal
        # is the part that matters and is far easier to measure.
        self.declare_parameter("soft_iron_scale", [1.0, 1.0, 1.0])

        self.declare_parameter("use_fake_bus", False)

        p = self.get_parameter
        self._frame_id = p("frame_id").value
        self._hard_iron = list(p("hard_iron").value)
        self._soft_iron = list(p("soft_iron_scale").value)
        address = int(p("address").value)

        if self._hard_iron == [0.0, 0.0, 0.0]:
            self.get_logger().warn(
                "hard_iron is all zeros — this magnetometer is UNCALIBRATED. The "
                "heading it produces will be wrong by a rotating amount. Run the "
                "procedure in docs/mag-calibration.md before trusting any of it."
            )

        if p("use_fake_bus").value:
            from qmc5883l_driver.fake_bus import FakeQMC5883LBus

            self.get_logger().warn(
                "using the SIMULATED I2C bus — this is a dev aid, not a sensor"
            )
            self._bus = FakeQMC5883LBus(address=address)
        else:
            import smbus2

            self._bus = smbus2.SMBus(int(p("i2c_bus").value))

        self._mag = QMC5883L(
            self._bus,
            address=address,
            range_gauss=int(p("range_gauss").value),
            odr_hz=int(p("odr_hz").value),
            osr=int(p("osr").value),
        )
        self._probe_and_configure(address)

        self._reads_ok = 0
        self._read_errors = 0
        self._overflows = 0
        self._not_ready = 0
        self._last: Optional[tuple] = None

        self._pub = self.create_publisher(MagneticField, "imu/mag", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / p("rate_hz").value, self._tick)
        self.create_timer(1.0, self._diag_tick)
        self.get_logger().info(f"QMC5883L streaming on {self._frame_id}")

    def _probe_and_configure(self, address: int) -> None:
        try:
            chip_id = self._mag.chip_id()
        except OSError as exc:
            self.get_logger().fatal(
                f"no I2C response at 0x{address:02x}: {exc}. Check the mast wiring "
                "and `i2cdetect -y 1` (docs/deployment.md step 5)."
            )
            raise
        if chip_id != CHIP_ID_VALUE:
            raise RuntimeError(
                f"chip ID is 0x{chip_id:02x}, expected 0x{CHIP_ID_VALUE:02x} — this "
                "is not a QMC5883L. HMC5883L breakouts look identical and are a "
                "different chip with a different register map."
            )
        self._mag.reset()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        self._mag.configure()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.05))

    def _calibrated(self, field: tuple) -> tuple:
        return tuple(
            (field[i] - self._hard_iron[i]) * self._soft_iron[i] for i in range(3)
        )

    def _tick(self) -> None:
        try:
            sample = self._mag.read()
        except OSError as exc:
            self._read_errors += 1
            self.get_logger().warn(f"I2C read failed: {exc}", throttle_duration_sec=5.0)
            return
        if sample is None:
            # Polling faster than the chip's ODR; harmless, but worth counting in
            # case rate_hz and odr_hz drift apart.
            self._not_ready += 1
            return

        if sample.overflow:
            self._overflows += 1
            # Publishing a saturated sample would feed the filter a wrong
            # heading that looks like a real one. Drop it.
            self.get_logger().warn(
                "magnetometer SATURATED — the reading is dominated by something "
                "other than the earth. Check the mast clearance from the motors.",
                throttle_duration_sec=5.0,
            )
            return

        self._reads_ok += 1
        field = self._calibrated(sample.field)
        self._last = field

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = field
        # Unknown covariance: the first element is set to -1 by convention when a
        # driver cannot characterise its own noise, and madgwick does not use it.
        msg.magnetic_field_covariance[0] = -1.0
        self._pub.publish(msg)

    def _diag_tick(self) -> None:
        status = DiagnosticStatus(name="qmc5883l: magnetometer", hardware_id="qmc5883l")
        if self._reads_ok == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = "no successful reads"
        elif self._overflows > 0:
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._overflows} saturated samples dropped"
        elif self._hard_iron == [0.0, 0.0, 0.0]:
            status.level = DiagnosticStatus.WARN
            status.message = "UNCALIBRATED — heading is not trustworthy"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "streaming"

        values = [
            KeyValue(key="reads_ok", value=str(self._reads_ok)),
            KeyValue(key="read_errors", value=str(self._read_errors)),
            KeyValue(key="overflows", value=str(self._overflows)),
        ]
        if self._last is not None:
            magnitude = math.sqrt(sum(v * v for v in self._last))
            values.append(KeyValue(key="field_uT", value=f"{magnitude * 1e6:.1f}"))
            # The earth's total field is 25-65 uT everywhere on the planet. A
            # calibrated sensor reading far outside that is measuring the robot.
            if self._reads_ok > 10 and not (15e-6 < magnitude < 80e-6):
                status.level = max(status.level, DiagnosticStatus.WARN)
                status.message = (
                    f"field is {magnitude * 1e6:.0f} uT — the earth's is 25-65. "
                    "Calibration is off, or the mount is picking up the robot."
                )
        status.values = values

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> bool:
        try:
            self._mag.standby()
            self._bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Qmc5883lNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
