"""ROS adapter for the Gazebo Sim diff-drive model.

The ESP32 simulator still owns safety and command mixing. This backend only
translates the resulting wheel targets into Gazebo Twist commands and converts
Gazebo odometry back into measured wheel units.
"""

from __future__ import annotations

import math
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GazeboBackend:
    """Backend compatible with ``hoverboard_bridge.esp32_sim.Backend``."""

    def __init__(self, node, wheel_radius=0.0825, wheel_separation=0.5,
                 board_units_per_rpm=1.0, command_topic="/cmd_vel_gazebo",
                 odometry_topic="/odom_gazebo") -> None:
        self._node = node
        self._radius = float(wheel_radius)
        self._separation = float(wheel_separation)
        self._units_per_rpm = float(board_units_per_rpm)
        self._publisher = node.create_publisher(Twist, command_topic, 10)
        node.create_subscription(Odometry, odometry_topic, self._on_odometry, 10)
        self._lock = threading.Lock()
        self._meas_l = 0.0
        self._meas_r = 0.0
        self.pose = type("Pose", (), {"x": 0.0, "y": 0.0, "yaw": 0.0})()
        self.v = 0.0
        self.omega = 0.0
        self.accel_x = 0.0

    def step(self, target_l: float, target_r: float, dt: float):
        left_mps = self._units_to_mps(target_l)
        right_mps = self._units_to_mps(target_r)
        command = Twist()
        command.linear.x = 0.5 * (left_mps + right_mps)
        command.angular.z = (right_mps - left_mps) / self._separation
        self._publisher.publish(command)
        with self._lock:
            return self._meas_l, self._meas_r

    def _on_odometry(self, message: Odometry) -> None:
        v = message.twist.twist.linear.x
        omega = message.twist.twist.angular.z
        left_mps = v - omega * self._separation * 0.5
        right_mps = v + omega * self._separation * 0.5
        with self._lock:
            self._meas_l = self._mps_to_units(left_mps)
            self._meas_r = self._mps_to_units(right_mps)
            self.v = v
            self.omega = omega
            self.pose.x = message.pose.pose.position.x
            self.pose.y = message.pose.pose.position.y
            quaternion = message.pose.pose.orientation
            self.pose.yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
            )

    def _units_to_mps(self, units: float) -> float:
        rpm = units / self._units_per_rpm
        return rpm * 2.0 * math.pi * self._radius / 60.0

    def _mps_to_units(self, speed: float) -> float:
        rpm = speed * 60.0 / (2.0 * math.pi * self._radius)
        return rpm * self._units_per_rpm