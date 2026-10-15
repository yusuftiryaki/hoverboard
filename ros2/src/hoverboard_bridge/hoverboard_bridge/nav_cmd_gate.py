"""Nav2's velocity commands reach /cmd_vel only while the local EKF is sane.

    controller_server ─┐
                       ├─► /cmd_vel_nav ─► nav_cmd_gate ─► /cmd_vel ─► hoverboard_bridge
    behavior_server  ──┘                        ▲
                       /odometry/filtered/local ┘

Why: Nav2 steers by odom->base_link, which ekf_local publishes, and nothing in
the stack checked that estimate. In CI ekf_local went to 1e148 m within 33 ms of
startup (robot_localization 3.8.3, see ekf.yaml) and the robot was stopped only
by ACCIDENT — the planner's worldToMap overflowed. A diverged estimate that
stays finite and plausible-looking would have been driven on.

So every Nav2 command is checked against the latest EKF output. If that output
is missing, stale, non-finite or physically impossible, the gate sends ZERO
instead and says why in /diagnostics. Nav2 then sees no progress and aborts.

What it cannot see: an estimate that is wrong but plausible (a few metres off,
a slow heading drift). That needs a second opinion — GPS, INA228 current, the
bumper (docs/handoff.md, A3c trap 3). This gate catches the broken filter, not
the merely inaccurate one.

Teleop does NOT go through here, on purpose: driving by hand needs no
localization, and a broken EKF must not take manual control away with it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class HealthLimits:
    max_age: float = 0.5                  # s since the last EKF message
    # Bounds for "physically impossible", not for "inaccurate": wide enough
    # that a healthy filter never trips them over a day's driving, tight
    # enough that any blow-up does at once (CI's was 1e148 m, 1e281 m^2).
    max_position: float = 1.0e4           # m from the odom origin
    max_position_variance: float = 1.0e6  # m^2 (a 1 km sigma)
    max_speed: float = 5.0                # m/s; the board tops out near 2.6
    max_yaw_rate: float = 10.0            # rad/s


def estimate_problem(x: float, y: float, yaw_z: float, yaw_w: float,
                     vx: float, wz: float, var_x: float, var_y: float,
                     limits: HealthLimits) -> Optional[str]:
    """Why this EKF output must not be driven on, or None if it may be.

    yaw_z/yaw_w are the quaternion's z and w (two_d_mode); var_* are the pose
    covariance diagonal.
    """
    values = (x, y, yaw_z, yaw_w, vx, wz, var_x, var_y)
    if not all(math.isfinite(v) for v in values):
        return "EKF output is not finite"
    if max(abs(x), abs(y)) > limits.max_position:
        return f"EKF position ({x:.3g}, {y:.3g}) m is beyond {limits.max_position:g} m"
    if max(var_x, var_y) > limits.max_position_variance:
        return (f"EKF position variance {max(var_x, var_y):.3g} m^2 is beyond "
                f"{limits.max_position_variance:g}")
    if abs(vx) > limits.max_speed:
        return f"EKF speed {vx:.3g} m/s is beyond {limits.max_speed:g}"
    if abs(wz) > limits.max_yaw_rate:
        return f"EKF yaw rate {wz:.3g} rad/s is beyond {limits.max_yaw_rate:g}"
    return None


def main(args=None) -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class NavCmdGate(Node):
        def __init__(self) -> None:
            super().__init__("nav_cmd_gate")
            defaults = HealthLimits()
            for name in ("max_age", "max_position", "max_position_variance",
                         "max_speed", "max_yaw_rate"):
                self.declare_parameter(name, getattr(defaults, name))
            p = self.get_parameter
            self._limits = HealthLimits(*(float(p(n).value) for n in (
                "max_age", "max_position", "max_position_variance",
                "max_speed", "max_yaw_rate")))

            self._last_odom_time = None
            self._odom_problem = "no EKF output yet"
            self._blocked = 0          # commands replaced by zero, since start
            self._forwarded = 0
            self._was_blocking = False

            self._pub = self.create_publisher(Twist, "cmd_vel", 10)
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_subscription(Odometry, "odometry/filtered/local",
                                     self._on_odom, 10)
            self.create_subscription(Twist, "cmd_vel_nav", self._on_cmd, 10)
            self.create_timer(1.0, self._diag_tick)

        def _on_odom(self, msg) -> None:
            pose, twist, cov = msg.pose.pose, msg.twist.twist, msg.pose.covariance
            self._odom_problem = estimate_problem(
                pose.position.x, pose.position.y,
                pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.angular.z, cov[0], cov[7], self._limits)
            self._last_odom_time = self.get_clock().now()

        def _problem(self) -> Optional[str]:
            if self._last_odom_time is None:
                return "no EKF output yet"
            age = (self.get_clock().now() - self._last_odom_time).nanoseconds * 1e-9
            if age > self._limits.max_age:
                return f"EKF output is {age:.2f} s old (limit {self._limits.max_age:g} s)"
            return self._odom_problem

        def _on_cmd(self, msg) -> None:
            problem = self._problem()
            if problem is None:
                self._pub.publish(msg)
                self._forwarded += 1
                if self._was_blocking:
                    self.get_logger().info("EKF healthy again — forwarding Nav2")
                self._was_blocking = False
                return
            self._pub.publish(Twist())
            self._blocked += 1
            if not self._was_blocking:
                self.get_logger().error(f"STOPPING Nav2: {problem}")
            self._was_blocking = True

        def _diag_tick(self) -> None:
            problem = self._problem()
            status = DiagnosticStatus(name="nav_cmd_gate: localization health",
                                      hardware_id="ekf_local")
            if problem is None:
                status.level = DiagnosticStatus.OK
                status.message = "EKF healthy — Nav2 commands forwarded"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = f"Nav2 commands replaced by zero: {problem}"
            status.values = [
                KeyValue(key="forwarded", value=str(self._forwarded)),
                KeyValue(key="blocked", value=str(self._blocked)),
            ]
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [status]
            self._diag_pub.publish(array)

    rclpy.init(args=args)
    node = NavCmdGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
