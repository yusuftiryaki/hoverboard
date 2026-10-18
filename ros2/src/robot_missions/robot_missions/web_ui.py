"""web_ui — the robot's web page (docs/missions.md, 4. aşama).

    http://<robot>:8080/   draw the site, start missions, watch, confirm detections

web_api.py is the HTTP side and knows no ROS; this node is its backend:

    /gps/filtered (navsat_transform)       robot lat/lon
    map -> base_link (tf)                  heading
    mission_runner/status, battery_supervisor/state, /battery
    mission_runner/start|pause|resume|abort
    site_map_server/reload, battery_supervisor/reload_site   after a site edit
    camera/image_raw                       the live view — subscribed only while
                                           a page is watching, encoded <= 5 Hz
    /cmd_vel_teleop (out, 20 Hz)           driving from the page (teleop.py):
                                           only while a page holds the lease
    teleop/state (out, 10 Hz)              who is driving — for the episode
                                           recorder (docs/learning.md)

The executor spins in its own thread; HTTP handlers (one thread each) send a
service request and wait on its future, bounded by call_timeout_s.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Image, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_missions.camera import jpeg_from_image
from robot_missions.teleop import TeleopGate
from robot_missions.web_api import WebApi, default_static_dir, make_server
from robot_missions_interfaces.srv import StartMission


class WebUi(Node):
    def __init__(self, **kwargs):
        super().__init__("web_ui", **kwargs)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("site_file", "")
        self.declare_parameter("detected_file",
                               os.path.expanduser("~/.robot/detected_obstacles.json"))
        self.declare_parameter("call_timeout_s", 5.0)
        self.declare_parameter("static_dir", "")
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("camera_fps", 5.0)
        # Driving from the page (docs/missions.md 5a). Below the bridge's own
        # max_linear_vel / max_angular_vel (1.0 / 1.5), which still apply.
        self.declare_parameter("use_teleop", True)
        self.declare_parameter("teleop_max_linear", 0.6)
        self.declare_parameter("teleop_max_angular", 1.0)
        self.declare_parameter("teleop_deadman_s", 0.3)
        self.declare_parameter("teleop_lease_s", 2.0)

        self._lock = threading.Lock()
        self._fix: Optional[Tuple[float, float, float]] = None   # lat, lon, stamp
        self._mission: Optional[Dict[str, Any]] = None
        self._supervisor: Optional[str] = None
        self._battery: Optional[Dict[str, Any]] = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavSatFix, "gps/filtered", self._on_fix, 10)
        self.create_subscription(String, "mission_runner/status", self._on_mission, latched)
        self.create_subscription(String, "battery_supervisor/state", self._on_supervisor, latched)
        self.create_subscription(BatteryState, "battery", self._on_battery, 10)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._start = self.create_client(StartMission, "mission_runner/start")
        self._commands = {name: self.create_client(Trigger, f"mission_runner/{name}")
                          for name in ("pause", "resume", "abort")}
        self._reloads = [self.create_client(Trigger, "site_map_server/reload"),
                         self.create_client(Trigger, "battery_supervisor/reload_site")]

        # The live view: nothing is subscribed (or encoded) until a page asks.
        self._camera = threading.Condition()
        self._jpeg: Optional[Tuple[int, bytes, float]] = None     # seq, data, time
        self._camera_wanted_until = 0.0
        self._camera_sub = None
        self._seq = 0
        self._last_encode = 0.0
        self.create_timer(0.5, self._manage_camera)

        p = self.get_parameter
        self.teleop = None
        if p("use_teleop").value:
            self.teleop = TeleopGate(p("teleop_max_linear").value, p("teleop_max_angular").value,
                                     p("teleop_deadman_s").value, p("teleop_lease_s").value)
            self._teleop_pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)
            self._teleop_state_pub = self.create_publisher(String, "teleop/state", 10)
            # Wall time on purpose: the deadman is about the browser, and a
            # sim-time timer would stop ticking with /clock — leaving the last
            # command to the bridge's deadman instead of this one's zero.
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self._ticks = 0
            self.create_timer(0.05, self._tick_teleop, clock=steady)
        self.api = WebApi(p("site_file").value, p("detected_file").value, self, self.teleop)
        static = p("static_dir").value or default_static_dir()
        self.server = make_server(self.api, static, p("host").value, int(p("port").value))
        host, port = self.server.server_address[:2]
        self.get_logger().info(f"web page on http://{host}:{port}/ (static: {static})")

    def _tick_teleop(self) -> None:
        out = self.teleop.output()
        if out is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = out
            self._teleop_pub.publish(msg)
        self._ticks += 1
        if self._ticks % 2 == 0:
            self._teleop_state_pub.publish(String(data=json.dumps(self.teleop.state())))

    # --- subscriptions --------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            with self._lock:
                self._fix = (msg.latitude, msg.longitude, time.time())

    def _on_mission(self, msg: String) -> None:
        with self._lock:
            self._mission = json.loads(msg.data)

    def _on_supervisor(self, msg: String) -> None:
        with self._lock:
            self._supervisor = msg.data

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._battery = {
                "percentage": msg.percentage if math.isfinite(msg.percentage) else None,
                "voltage": msg.voltage if math.isfinite(msg.voltage) else None}

    # --- WebApi backend -------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        with self._lock:
            fix, mission = self._fix, self._mission
            supervisor, battery = self._supervisor, self._battery
        robot = None
        if fix is not None:
            robot = {"lat": fix[0], "lon": fix[1], "age_s": round(time.time() - fix[2], 1),
                     "heading_deg": self._heading()}
        return {"robot": robot, "mission": mission, "supervisor": supervisor,
                "battery": battery}

    def _heading(self) -> Optional[float]:
        """Compass bearing (0 = north, clockwise) from map's yaw. `map`'s x
        is the IMU's east (navsat_transform, docs/missions.md)."""
        try:
            tf = self._tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return round((90.0 - math.degrees(yaw)) % 360.0, 1)

    def start_mission(self, spec: Dict[str, Any]) -> Tuple[bool, str]:
        request = StartMission.Request()
        request.mission = json.dumps(spec)
        response = self._call(self._start, request)
        if response is None:
            return False, "mission_runner did not answer (is it running?)"
        return response.accepted, response.message

    def command(self, name: str) -> Tuple[bool, str]:
        response = self._call(self._commands[name], Trigger.Request())
        if response is None:
            return False, "mission_runner did not answer (is it running?)"
        return response.success, response.message

    def reload_site(self) -> Tuple[bool, str]:
        """Ask everyone holding the site to read it again. A node that is not
        running (no Nav2, so no supervisor) is skipped, not a failure; one
        that answers no, or never answers, is."""
        messages, ok = [], True
        for client in self._reloads:
            if not client.wait_for_service(timeout_sec=1.0):
                messages.append(f"{client.srv_name}: not running")
                continue
            response = self._call(client, Trigger.Request())
            if response is None:
                ok = False
                messages.append(f"{client.srv_name}: no answer")
            else:
                ok = ok and response.success
                messages.append(f"{client.srv_name}: {response.message}")
        return ok, "; ".join(messages)

    def camera_jpeg(self, wait_s: float = 3.0, after: Optional[int] = None):
        """(seq, JPEG) newer than `after` (any fresh one if None), or None."""
        deadline = time.monotonic() + wait_s
        with self._camera:
            self._camera_wanted_until = time.monotonic() + 5.0
            while True:
                frame = self._jpeg
                if frame is not None and (after is None and time.time() - frame[2] < 2.0
                                          or after is not None and frame[0] > after):
                    return frame[0], frame[1]
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._camera.wait(timeout=left)

    def _manage_camera(self) -> None:
        wanted = time.monotonic() < self._camera_wanted_until
        topic = self.get_parameter("camera_topic").value
        if wanted and self._camera_sub is None and topic:
            self._camera_sub = self.create_subscription(Image, topic, self._on_image,
                                                        qos_profile_sensor_data)
        elif not wanted and self._camera_sub is not None:
            self.destroy_subscription(self._camera_sub)
            self._camera_sub = None
            with self._camera:
                self._jpeg = None

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_encode < 1.0 / max(0.5, self.get_parameter("camera_fps").value):
            return
        self._last_encode = now
        data = jpeg_from_image(msg.encoding, msg.width, msg.height, msg.step, bytes(msg.data),
                               quality=70)
        if data is None:
            self.get_logger().warn(f"camera: cannot encode {msg.encoding!r}",
                                   throttle_duration_sec=60.0)
            return
        with self._camera:
            self._seq += 1               # monotonic across re-subscriptions
            self._jpeg = (self._seq, data, time.time())
            self._camera.notify_all()

    def _call(self, client, request):
        timeout = self.get_parameter("call_timeout_s").value
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WebUi()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        node.server.serve_forever()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.server.shutdown()
            node.server.server_close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
