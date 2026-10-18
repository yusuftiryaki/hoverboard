"""edge_expert — the privileged edge-following demonstrator (docs/learning.md, 6a).

    /ground_truth (in)              sim_world pose: privileged, sim only
    expert/cmd_vel (out, 20 Hz)     the expert's label, ALWAYS while running —
                                    recorded, it is DAgger's label (6d)
    cmd_vel_teleop (out)            the same command, when drive:=true: the
                                    expert drives like a person at the page
    ~/start, ~/stop (Trigger)
    ~/status (String JSON, latched) idle | running | done, progress

The lawn is an `area` from the site file, put into sim_world with the exact
inverse of sim_node's GPS formula (flat earth about the datum, sim_node's
EARTH_RADIUS_M) — the simulator's own truth about where the lawn is, not
navsat_transform's estimate of it. Privileged on purpose (edge_expert.py).
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shapely.geometry import Polygon
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_sim.edge_expert import EdgeExpert, ExpertConfig
from robot_sim.site_world import areas_in_world


def lawn_in_sim_world(site_file: str, area: str, datum_lat: float, datum_lon: float) -> Polygon:
    areas = areas_in_world(site_file, datum_lat, datum_lon, area)
    if not areas:
        raise ValueError(f"no area {area!r} in {site_file}")
    return areas[0]


class EdgeExpertNode(Node):
    def __init__(self, **kwargs):
        super().__init__("edge_expert", **kwargs)
        self.declare_parameter("site_file", "")
        self.declare_parameter("area", "")
        self.declare_parameter("datum_lat", 41.0)        # sim_node's
        self.declare_parameter("datum_lon", 29.0)
        self.declare_parameter("offset_m", 0.4)
        self.declare_parameter("speed", 0.3)
        self.declare_parameter("direction", "ccw")
        self.declare_parameter("laps", 1.0)
        self.declare_parameter("drive", True)
        p = self.get_parameter
        self.lawn = lawn_in_sim_world(p("site_file").value, p("area").value,
                                      p("datum_lat").value, p("datum_lon").value)
        self.config = ExpertConfig(offset_m=p("offset_m").value, speed=p("speed").value)
        self.expert = None
        self.state = "idle"
        self.pose = None
        self._ticks = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._label_pub = self.create_publisher(Twist, "expert/cmd_vel", 10)
        self._drive_pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)
        self._status_pub = self.create_publisher(String, "~/status", latched)
        self.create_subscription(Odometry, "ground_truth", self._on_truth, 10)
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/stop", self._on_stop)
        self.create_timer(0.05, self._tick)
        self._publish_status()

    def _on_truth(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _on_start(self, request, response):
        self.expert = EdgeExpert(self.lawn, self.config, self.get_parameter("direction").value,
                                 self.get_parameter("laps").value)
        self.state = "running"
        response.success, response.message = True, f"edge lap of {self.expert.length:.1f} m"
        self._publish_status()
        return response

    def _on_stop(self, request, response):
        self._finish("idle")
        response.success, response.message = True, "stopped"
        return response

    def _finish(self, state: str) -> None:
        if self.state == "running":
            self._send(0.0, 0.0)
        self.state = state
        self._publish_status()

    def _send(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self._label_pub.publish(msg)
        if self.get_parameter("drive").value:
            self._drive_pub.publish(msg)

    def _tick(self) -> None:
        if self.state != "running" or self.pose is None:
            return
        v, w = self.expert.step(*self.pose)
        if self.expert.done:
            self._finish("done")
            return
        self._send(v, w)
        self._ticks += 1
        if self._ticks % 20 == 0:                      # 1 Hz progress
            self._publish_status()

    def _publish_status(self) -> None:
        out = {"state": self.state}
        if self.expert is not None:
            out.update(progress_m=round(self.expert.progress_m, 2),
                       length_m=round(self.expert.length, 2))
        self._status_pub.publish(String(data=json.dumps(out)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EdgeExpertNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
