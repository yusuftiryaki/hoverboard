"""Kinematic ground truth. ROS-free, deterministic, no physics.

Plugs in behind hoverboard_bridge.esp32_sim as a Backend: it takes the wheel
commands the simulated ESP32 decided on, returns what the hall sensors would
measure, and — the part that matters — integrates where the robot actually IS.

That pose is the whole point of this milestone. On the real robot there is no
ground truth: you can never ask "how far off is the EKF?", only "does it look
smooth?". Here the question has a number.

WHAT THIS IS NOT: physics. There is no mass, no friction, no slip, no tipping,
no motor torque curve. Wheels turn exactly as commanded (through a lag) and the
robot goes exactly where perfect differential-drive kinematics says. Slip alone
is the single biggest source of real odometry error, so a good result here proves
the maths is wired up correctly — NOT that the robot will localize well outdoors.
That question needs A3 (Gazebo) and, more honestly, dirt.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Pose:
    x: float = 0.0      # metres east
    y: float = 0.0      # metres north
    yaw: float = 0.0    # rad, CCW from east (REP-103)


@dataclass(frozen=True)
class CircularObstacle:
    x: float
    y: float
    radius: float


class KinematicWorld:
    """Differential drive kinematics with a first-order wheel lag."""

    def __init__(
        self,
        wheel_radius: float = 0.0825,
        wheel_separation: float = 0.5,
        # The board's REAL units-per-RPM. Deliberately separate from
        # hoverboard_bridge's cmd_per_rpm param, which is only what the Pi
        # BELIEVES. Setting them to different values simulates a miscalibrated
        # robot — which is the honest default state until roadmap step B4.
        board_units_per_rpm: float = 1.0,
        # Wheel spin-up time constant. A guess: hub motors under a ~15 kg robot.
        tau: float = 0.09,
        # Slip factor: fraction of commanded wheel speed lost to friction/slip.
        # 0.0 = no slip (perfect grip), 0.1 = 10% slip loss, etc.
        # Realistically 0.05-0.15 on varied terrain; 0.0 here means "ideal road".
        slip_factor: float = 0.0,
        obstacles: Tuple[CircularObstacle, ...] = (),
        # ⚠️ Must match nav2.yaml's costmap robot_radius. There is one robot
        # and it has one radius; two numbers for it would let Nav2 plan a gap
        # the physics refuses (or worse, the reverse). Both are guesses until
        # the chassis exists (roadmap B3) — guessed once, in two places that
        # agree, rather than twice.
        robot_radius: float = 0.4,
    ) -> None:
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.board_units_per_rpm = board_units_per_rpm
        self._tau = tau
        self._slip_factor = slip_factor
        self._obstacles = tuple(obstacles)
        self.robot_radius = robot_radius
        self.collision = False

        self.pose = Pose()
        self.v = 0.0          # body forward velocity, m/s
        self.omega = 0.0      # body yaw rate, rad/s
        self.accel_x = 0.0    # body forward acceleration, m/s^2 (for the fake IMU)
        self._meas_l = 0.0    # raw board units
        self._meas_r = 0.0

    # ---- Backend interface (see hoverboard_bridge.esp32_sim.Backend) ---------
    def step(self, target_l: float, target_r: float, dt: float) -> Tuple[float, float]:
        if dt <= 0.0:
            return self._meas_l, self._meas_r

        alpha = 1.0 - math.exp(-dt / self._tau) if self._tau > 0 else 1.0
        self._meas_l += (target_l - self._meas_l) * alpha
        self._meas_r += (target_r - self._meas_r) * alpha

        # Apply slip to ground truth only. Hall feedback remains the wheel's
        # measured spin; this exposes the odometry error caused by wheel slip.
        slipped_l = self._meas_l * (1.0 - self._slip_factor)
        slipped_r = self._meas_r * (1.0 - self._slip_factor)

        rpm_l = slipped_l / self.board_units_per_rpm
        rpm_r = slipped_r / self.board_units_per_rpm
        v_l = rpm_l / 60.0 * 2.0 * math.pi * self.wheel_radius
        v_r = rpm_r / 60.0 * 2.0 * math.pi * self.wheel_radius

        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / self.wheel_separation
        self.accel_x = (v - self.v) / dt
        self.v, self.omega = v, omega

        # Exact arc integration rather than the Euler step the bridge's odometry
        # uses. Using the same approximation in both would hide the bridge's
        # discretisation error from the very comparison meant to expose it.
        yaw0 = self.pose.yaw
        next_x, next_y, next_yaw = self.pose.x, self.pose.y, yaw0
        if abs(omega) < 1e-9:
            next_x += v * math.cos(yaw0) * dt
            next_y += v * math.sin(yaw0) * dt
        else:
            radius = v / omega
            yaw1 = yaw0 + omega * dt
            next_x += radius * (math.sin(yaw1) - math.sin(yaw0))
            next_y -= radius * (math.cos(yaw1) - math.cos(yaw0))
            next_yaw = math.atan2(math.sin(yaw1), math.cos(yaw1))

        if self._collides(next_x, next_y):
            self.collision = True
        else:
            self.collision = False
            self.pose.x, self.pose.y, self.pose.yaw = next_x, next_y, next_yaw

        return self._meas_l, self._meas_r

    def _collides(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - obstacle.x, y - obstacle.y)
            <= self.robot_radius + obstacle.radius
            for obstacle in self._obstacles
        )
