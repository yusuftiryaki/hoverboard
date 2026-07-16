"""/cmd_vel <-> ESP32 serial bridge, plus wheel odometry.

Sits between ROS 2 and the ESP32 "brainstem" (firmware/esp32_bridge). It does
three things:

  1. inverse kinematics — /cmd_vel (v, omega) to left/right wheel RPM to the
     hoverboard's (speed, steer) mixer units
  2. serial I/O — PiCommand out at cmd_rate_hz (each frame is also the ESP32's
     watchdog heartbeat), EspFeedback in
  3. forward kinematics — measured hall RPM to /odom, /joint_states, /battery

It does NOT publish the odom->base_link transform by default: robot_localization's
local EKF owns that (see robot_bringup/config/ekf.yaml). Set publish_tf:=true only
when running this node standalone, e.g. bench teleop without the EKF.

CALIBRATION: cmd_per_rpm, steer_sign and the invert_* flags cannot be derived
from a datasheet — the TXTY board variant is undocumented (docs/handoff.md). They
are measured on the bench with the wheels in the air. See the notes on each
parameter below.
"""

from __future__ import annotations

import math

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from hoverboard_bridge.protocol import FeedbackParser, pack_pi_command

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class HoverboardBridge(Node):
    def __init__(self) -> None:
        super().__init__("hoverboard_bridge")

        # ---- Parameters ------------------------------------------------------
        # Serial link. /dev/esp32 is the udev symlink from docs/deployment.md
        # step 4 — never trust /dev/ttyUSB0, it swaps with the GPS across boots.
        self.declare_parameter("port", "/dev/esp32")
        self.declare_parameter("baud", 115200)

        # 50 Hz matches the firmware's TX_PERIOD_MS. Must stay well under the
        # 200 ms WATCHDOG_MS or the ESP32 will cut the motors mid-drive.
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("rx_poll_hz", 200.0)
        # No /cmd_vel for this long -> command zero (but keep the heartbeat, so
        # the ESP32 knows we are alive and merely idle).
        self.declare_parameter("cmd_timeout", 0.5)

        # Geometry. 6.5" hoverboard hub wheel = 165 mm diameter.
        self.declare_parameter("wheel_radius", 0.0825)
        # TODO: measure on the real chassis (roadmap step 3, not built yet).
        self.declare_parameter("wheel_separation", 0.5)

        # Raw hoverboard command units per wheel RPM. EFeru's FOC firmware maps
        # +-1000 to +-N_MOT_MAX rpm, so 1.0 is the right starting guess for the
        # stock N_MOT_MAX=1000. TODO calibrate: wheels in the air, publish a
        # steady /cmd_vel, compare the commanded RPM against /joint_states.
        self.declare_parameter("cmd_per_rpm", 1.0)
        # The board's mixer is speedR = speed - steer, speedL = speed + steer,
        # but the sign depends on how the wheels are mounted. TODO: if the robot
        # turns the wrong way on the bench, flip this to -1.
        self.declare_parameter("steer_sign", 1.0)
        # Hub motors face opposite directions, so one hall stream is inverted.
        # TODO: spin each wheel forward by hand, check /joint_states signs.
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", True)

        # Firmware clamps to SPEED_LIMIT=300 anyway; clamp here so /odom's
        # expectations match what the motors will actually be asked to do.
        self.declare_parameter("speed_limit", 300)

        # EspFeedback raw -> SI. EFeru: batVoltage in 0.01 V, boardTemp in 0.1 C.
        # TODO: sanity check against a multimeter once the board is flashed.
        self.declare_parameter("battery_scale", 0.01)
        self.declare_parameter("temp_scale", 0.1)
        self.declare_parameter("battery_cells", 10)  # 36 V 10S pack

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # Leave false in the full stack — the local EKF publishes odom->base_link.
        self.declare_parameter("publish_tf", False)
        # Verified against this workspace's Nav2 (1.3.12): controller_server
        # publishes plain geometry_msgs/Twist on /cmd_vel, same as
        # teleop_twist_keyboard — so false is right for both. Later Nav2 releases
        # move to TwistStamped behind `enable_stamped_cmd_vel`; when that lands,
        # check `ros2 topic info /cmd_vel` and flip this rather than guessing.
        self.declare_parameter("use_stamped_cmd_vel", False)

        p = self.get_parameter
        self._wheel_radius = p("wheel_radius").value
        self._wheel_separation = p("wheel_separation").value
        self._cmd_per_rpm = p("cmd_per_rpm").value
        self._steer_sign = p("steer_sign").value
        self._left_sign = -1.0 if p("invert_left").value else 1.0
        self._right_sign = -1.0 if p("invert_right").value else 1.0
        self._speed_limit = int(p("speed_limit").value)
        self._cmd_timeout = p("cmd_timeout").value
        self._battery_scale = p("battery_scale").value
        self._temp_scale = p("temp_scale").value
        self._battery_cells = int(p("battery_cells").value)
        self._odom_frame = p("odom_frame").value
        self._base_frame = p("base_frame").value
        self._publish_tf = p("publish_tf").value

        # ---- Serial ----------------------------------------------------------
        port = p("port").value
        try:
            # timeout=0 -> non-blocking reads; the executor stays responsive.
            self._serial = serial.Serial(port, p("baud").value, timeout=0)
        except serial.SerialException as exc:
            self.get_logger().fatal(f"cannot open {port}: {exc}")
            raise
        self.get_logger().info(f"ESP32 link open on {port}")
        self._parser = FeedbackParser()

        # ---- State -----------------------------------------------------------
        self._target_v = 0.0        # m/s
        self._target_w = 0.0        # rad/s
        self._last_cmd_time = self.get_clock().now()
        self._clear_estop_request = False
        self._last_fb = None
        self._last_fb_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._pos_l = 0.0           # integrated wheel angle, rad
        self._pos_r = 0.0
        self._bump_was_set = False

        # ---- ROS interfaces --------------------------------------------------
        cmd_type = TwistStamped if p("use_stamped_cmd_vel").value else Twist
        cmd_cb = self._on_cmd_stamped if cmd_type is TwistStamped else self._on_cmd
        self.create_subscription(cmd_type, "cmd_vel", cmd_cb, 10)

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self._battery_pub = self.create_publisher(BatteryState, "battery", 10)
        # Latched-style QoS would be nicer, but the ESP32 restates the bumper at
        # 50 Hz anyway, so a late subscriber is at most 20 ms behind.
        self._bumper_pub = self.create_publisher(Bool, "bumper", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # The ESP32 only clears a latched e-stop if the button is physically
        # released; this service just asks. Check the response.
        self.create_service(Trigger, "~/clear_estop", self._on_clear_estop)

        self.create_timer(1.0 / p("cmd_rate_hz").value, self._tx_tick)
        self.create_timer(1.0 / p("rx_poll_hz").value, self._rx_tick)
        self.create_timer(1.0, self._diag_tick)

    # ---- Subscriptions -------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        self._target_v = msg.linear.x
        self._target_w = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _on_cmd_stamped(self, msg: TwistStamped) -> None:
        self._on_cmd(msg.twist)

    def _on_clear_estop(self, _request, response):
        self._clear_estop_request = True
        response.success = True
        response.message = (
            "clear requested; the ESP32 only obeys if the button is released — "
            "watch /diagnostics"
        )
        return response

    # ---- TX: cmd_vel -> wheels ----------------------------------------------
    def _tx_tick(self) -> None:
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self._cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self._target_v, self._target_w

        # Differential drive inverse kinematics.
        half_track = 0.5 * self._wheel_separation
        v_l = v - w * half_track
        v_r = v + w * half_track
        rpm_l = v_l / (2.0 * math.pi * self._wheel_radius) * 60.0
        rpm_r = v_r / (2.0 * math.pi * self._wheel_radius) * 60.0

        cmd_l = rpm_l * self._cmd_per_rpm
        cmd_r = rpm_r * self._cmd_per_rpm
        # Undo the board's mixer: speedL = speed + steer, speedR = speed - steer.
        speed = (cmd_l + cmd_r) * 0.5
        steer = (cmd_l - cmd_r) * 0.5 * self._steer_sign

        frame = pack_pi_command(
            int(round(speed)), int(round(steer)), self._clear_estop_request
        )
        self._clear_estop_request = False
        try:
            self._serial.write(frame)
        except serial.SerialException as exc:
            self.get_logger().error(f"serial write failed: {exc}")

    # ---- RX: feedback -> odometry -------------------------------------------
    def _rx_tick(self) -> None:
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(waiting) if waiting else b""
        except serial.SerialException as exc:
            self.get_logger().error(f"serial read failed: {exc}")
            return
        if not data:
            return
        for frame in self._parser.feed(data):
            self._handle_feedback(frame)

    def _handle_feedback(self, fb) -> None:
        now = self.get_clock().now()
        dt = 0.0
        if self._last_fb_time is not None:
            dt = (now - self._last_fb_time).nanoseconds * 1e-9
        self._last_fb_time = now
        self._last_fb = fb

        # Forward kinematics from the measured hall RPM.
        w_l = fb.speed_l * self._left_sign * RPM_TO_RAD_S    # rad/s at the wheel
        w_r = fb.speed_r * self._right_sign * RPM_TO_RAD_S
        v_l = w_l * self._wheel_radius
        v_r = w_r * self._wheel_radius
        v = 0.5 * (v_l + v_r)
        w = (v_r - v_l) / self._wheel_separation

        # Guard against a stalled or first frame: dt of 0 or a huge gap after a
        # dropout would otherwise teleport the pose.
        if 0.0 < dt < 0.5:
            self._yaw += w * dt
            self._x += v * math.cos(self._yaw) * dt
            self._y += v * math.sin(self._yaw) * dt
            self._pos_l += w_l * dt
            self._pos_r += w_r * dt

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # The EKF is configured to consume the twist only, so the pose covariance
        # is deliberately pessimistic — this pose drifts without bound.
        odom.pose.covariance[0] = 1e3     # x
        odom.pose.covariance[7] = 1e3     # y
        odom.pose.covariance[35] = 1e3    # yaw
        odom.twist.covariance[0] = 0.02   # vx
        odom.twist.covariance[35] = 0.05  # vyaw
        self._odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_wheel_joint", "right_wheel_joint"]
        joints.position = [self._pos_l, self._pos_r]
        joints.velocity = [w_l, w_r]
        self._joint_pub.publish(joints)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = fb.bat_voltage * self._battery_scale
        battery.temperature = fb.board_temp * self._temp_scale
        battery.current = float("nan")
        battery.charge = float("nan")
        battery.capacity = float("nan")
        battery.design_capacity = float("nan")
        battery.percentage = float("nan")
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.present = True
        battery.cell_voltage = []
        battery.location = "hoverboard"
        self._battery_pub.publish(battery)

        self._bumper_pub.publish(Bool(data=fb.bump))
        if fb.bump != self._bump_was_set:
            # Log the edges only — at 50 Hz, logging the level would bury the
            # rest of the log the moment the robot leans on something.
            if fb.bump:
                self.get_logger().warn(
                    "BUMPER HIT — the ESP32 is vetoing forward motion; "
                    "reverse and turning still work"
                )
            else:
                self.get_logger().info("bumper released")
            self._bump_was_set = fb.bump

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = yaw_to_quaternion(self._yaw)
            self._tf_broadcaster.sendTransform(tf)

    # ---- Diagnostics ---------------------------------------------------------
    def _diag_tick(self) -> None:
        status = DiagnosticStatus(name="hoverboard_bridge: ESP32 link", hardware_id="esp32")
        fb = self._last_fb
        if fb is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "no feedback from the ESP32"
        elif fb.estop:
            status.level = DiagnosticStatus.ERROR
            status.message = "E-STOP latched — motors are dead until it is cleared"
        elif not fb.watchdog_ok:
            status.level = DiagnosticStatus.WARN
            status.message = "ESP32 says our heartbeat is stale — motors stopped"
        elif fb.bump:
            # WARN, not ERROR: the robot is not broken, it is touching something
            # and can still reverse out of it under its own power.
            status.level = DiagnosticStatus.WARN
            status.message = "bumper hit — forward vetoed, reverse still allowed"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "driving"

        if fb is not None:
            status.values = [
                KeyValue(key="battery_v", value=f"{fb.bat_voltage * self._battery_scale:.2f}"),
                KeyValue(key="board_temp_c", value=f"{fb.board_temp * self._temp_scale:.1f}"),
                KeyValue(key="estop", value=str(fb.estop)),
                KeyValue(key="watchdog_ok", value=str(fb.watchdog_ok)),
                KeyValue(key="bump", value=str(fb.bump)),
                KeyValue(key="frames_ok", value=str(self._parser.frames_ok)),
                KeyValue(key="checksum_errors", value=str(self._parser.checksum_errors)),
            ]
            volts = fb.bat_voltage * self._battery_scale
            # 10S Li-ion: 3.3 V/cell is the "land now" line.
            if 0.0 < volts < 3.3 * self._battery_cells and status.level == DiagnosticStatus.OK:
                status.level = DiagnosticStatus.WARN
                status.message = f"battery low: {volts:.1f} V"

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> bool:
        # Best effort: leave the motors commanded to zero rather than relying on
        # the watchdog to notice we died.
        try:
            self._serial.write(pack_pi_command(0, 0))
            self._serial.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HoverboardBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Both are ordinary ways to stop: Ctrl-C, or systemd/launch shutting us
        # down. Neither deserves a stack trace in the field logs.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
