"""site_map_server in-process, against a stand-in /fromLLArray. Skips without ROS.

The stand-in answers exactly like robot_localization 3.8.3's does before its
transform exists — an EMPTY list, not an error — for the first few calls, then
converts flat-earth about (41 N, 29 E). The real navsat_transform is exercised
in robot_sim/test/test_site_map_stack.py.
"""

import json
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus   # noqa: E402
from geometry_msgs.msg import Point                                 # noqa: E402
from nav_msgs.msg import OccupancyGrid                              # noqa: E402
from rclpy.executors import SingleThreadedExecutor                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile                  # noqa: E402
from robot_localization.srv import FromLLArray, ToLL                # noqa: E402
from geographic_msgs.msg import GeoPoint                            # noqa: E402
from geometry_msgs.msg import TransformStamped                      # noqa: E402
from std_msgs.msg import Bool, String                               # noqa: E402
from sensor_msgs.msg import Image                                   # noqa: E402
from rclpy.qos import qos_profile_sensor_data                       # noqa: E402
from tf2_ros import StaticTransformBroadcaster                      # noqa: E402

from robot_missions.occupancy import OCCUPIED                       # noqa: E402
from robot_missions.site import SiteError                           # noqa: E402
from robot_missions.site_map_server import DIAG_NAME, SiteMapServer  # noqa: E402

LAT0, LON0 = 41.0, 29.0
DLAT_10M = 8.993216e-05
DLON_10M = 1.191613e-04
R = 6371000.0


@pytest.fixture
def context():
    own = not rclpy.ok()
    if own:
        rclpy.init()
    yield
    if own:
        rclpy.shutdown()


class FakeNavsat:
    """/fromLLArray that answers [] for `empty_answers` calls, then converts."""

    def __init__(self, empty_answers):
        self.node = rclpy.create_node("fake_navsat")
        self.calls = 0
        self._empty = empty_answers
        self.node.create_service(FromLLArray, "fromLLArray", self._answer)
        self.node.create_service(ToLL, "toLL", self._to_ll)

    def _to_ll(self, request, response):
        x, y = request.map_point.x, request.map_point.y
        response.ll_point = GeoPoint(
            latitude=LAT0 + math.degrees(y / R),
            longitude=LON0 + math.degrees(x / (R * math.cos(math.radians(LAT0)))))
        return response

    def _answer(self, request, response):
        self.calls += 1
        if self.calls <= self._empty:
            return response                     # what 3.8.3 does: no error, no points
        for p in request.ll_points:
            response.map_points.append(Point(
                x=math.radians(p.longitude - LON0) * R * math.cos(math.radians(LAT0)),
                y=math.radians(p.latitude - LAT0) * R))
        return response


def write_site(tmp_path, *features):
    path = tmp_path / "site.geojson"
    path.write_text(json.dumps({"type": "FeatureCollection", "features": list(features)}),
                    encoding="utf-8")
    return str(path)


def keepout_10m_east():
    lon, lat = LON0 + DLON_10M, LAT0
    ring = [[lon, lat], [lon + DLON_10M / 5, lat], [lon + DLON_10M / 5, lat + DLAT_10M / 5],
            [lon, lat + DLAT_10M / 5], [lon, lat]]       # 2 x 2 m, corner (10, 0)
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": "keepout", "name": "cicek"}}


def station_facing_north():
    return {"type": "Feature", "geometry": {"type": "Point", "coordinates": [LON0, LAT0]},
            "properties": {"kind": "station", "name": "home", "heading_deg": 0.0}}


def server(tmp_path, site_file, **overrides):
    overrides.setdefault("retry_period", 0.1)
    # Never the real ~/.robot file: a developer's detections would leak in.
    overrides.setdefault("detected_file", str(tmp_path / "detected.json"))
    return SiteMapServer(parameter_overrides=[
        rclpy.parameter.Parameter(name, value=value)
        for name, value in {"site_file": site_file, **overrides}.items()])


def listen(executor, topic, kind, qos, until, timeout_s=5.0):
    node = rclpy.create_node(f"listen_{abs(hash(topic)) % 10000}")
    got = []
    node.create_subscription(kind, topic, got.append, qos)
    executor.add_node(node)
    deadline = time.monotonic() + timeout_s
    while not until(got) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    executor.remove_node(node)
    node.destroy_node()
    return got


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def test_no_site_latches_an_all_free_map_at_once(context, tmp_path):
    node = server(tmp_path, "")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        (message,) = listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        # ⚠️ Silence would abort every goal (A3c trap 1): an empty site must
        # still be a map, all free, in `map`, covering the 60 m window.
        assert message.header.frame_id == "map"
        assert set(message.data) == {0}
        assert (message.info.width, message.info.height) == (600, 600)
        assert message.info.origin.position.x == pytest.approx(-30.0)
    finally:
        node.destroy_node()


def test_an_empty_answer_is_never_drawn_and_the_server_asks_again(context, tmp_path):
    navsat = FakeNavsat(empty_answers=3)
    node = server(tmp_path, write_site(tmp_path, keepout_10m_east()))
    executor = SingleThreadedExecutor()
    executor.add_node(navsat.node)
    executor.add_node(node)
    try:
        got = listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        assert got, "no map after the converter became ready"
        # Three empty answers, then the one that was drawn — no map from any
        # of the empty ones (there is exactly one message, built from call 4).
        assert navsat.calls == 4
        assert len(got) == 1
        message = got[0]

        def at(x, y):
            i = int(math.floor((x - message.info.origin.position.x) / message.info.resolution))
            j = int(math.floor((y - message.info.origin.position.y) / message.info.resolution))
            return message.data[j * message.info.width + i]

        assert at(11.0, 1.0) == OCCUPIED       # inside the keep-out
        assert at(9.5, 1.0) == 0               # half a metre west of it
        assert at(0.0, 0.0) == 0
    finally:
        node.destroy_node()
        navsat.node.destroy_node()


def test_a_broken_site_file_stops_the_node_with_the_reason(context, tmp_path):
    bad = keepout_10m_east()
    bad["properties"]["kind"] = "lawn"
    with pytest.raises(SiteError, match="kind must be one of"):
        server(tmp_path, write_site(tmp_path, bad))


def worst_status(messages):
    levels = [ord(s.level) if isinstance(s.level, bytes) else s.level
              for m in messages for s in m.status if s.name == DIAG_NAME]
    return max(levels) if levels else None


@pytest.mark.parametrize("site_has_station, level", [
    (True, DiagnosticStatus.OK),          # the site is home's one source
    (False, DiagnosticStatus.WARN),       # docks.yaml's station is not on the map
])
def test_a_station_only_docks_yaml_knows_is_flagged(context, tmp_path, site_has_station,
                                                     level):
    docks = tmp_path / "docks.yaml"
    docks.write_text(json.dumps({"docks": {"home": {
        "type": "charging_dock", "frame": "map", "pose": [0.0, 0.0, 0.0]}}}),
        encoding="utf-8")
    navsat = FakeNavsat(empty_answers=0)
    features = [station_facing_north()] if site_has_station else [keepout_10m_east()]
    node = server(tmp_path, write_site(tmp_path, *features), dock_database=str(docks))
    executor = SingleThreadedExecutor()
    executor.add_node(navsat.node)
    executor.add_node(node)
    try:
        assert listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        got = listen(executor, "/diagnostics", DiagnosticArray, 10,
                     lambda g: worst_status(g) is not None, timeout_s=3.0)
        expected = ord(level) if isinstance(level, bytes) else level
        assert worst_status(got) == expected
    finally:
        node.destroy_node()
        navsat.node.destroy_node()


def robot_at(node, x, y, yaw):
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "map", "base_link"
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(tf)
    return broadcaster


def test_a_bumper_press_puts_an_obstacle_on_the_map_and_on_disk(context, tmp_path):
    navsat = FakeNavsat(empty_answers=0)
    detected_file = tmp_path / "detected.json"
    node = server(tmp_path, write_site(tmp_path, keepout_10m_east()),
                  detected_file=str(detected_file))
    keep = robot_at(navsat.node, 2.0, 3.0, math.pi / 2)        # facing north
    bumper = navsat.node.create_publisher(Bool, "bumper", 10)
    camera = navsat.node.create_publisher(Image, "camera/image_raw", qos_profile_sensor_data)
    frame = Image(height=4, width=6, encoding="rgb8", step=18, data=bytes([200, 30, 30]) * 24)
    executor = SingleThreadedExecutor()
    executor.add_node(navsat.node)
    executor.add_node(node)
    try:
        assert listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        for _ in range(5):                              # the camera is running
            camera.publish(frame)
            listen(executor, "unused", String, 10, lambda g: False, timeout_s=0.1)
        for pressed in (True, True, True, False):      # one press, held: ONE record
            bumper.publish(Bool(data=pressed))
            listen(executor, "unused", String, 10, lambda g: False, timeout_s=0.2)

        def has_rock(got):
            return got and json.loads(got[-1].data)

        got = listen(executor, "site_map_server/detected", String, LATCHED, has_rock)
        (rock,) = json.loads(got[-1].data)
        # 0.45 m ahead of (2, 3) facing north: (2, 3.45).
        assert (rock["x"], rock["y"]) == (pytest.approx(2.0, abs=1e-6),
                                          pytest.approx(3.45, abs=1e-6))
        assert rock["count"] == 1 and rock["lat"] is not None

        maps = listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        message = maps[-1]

        def at(x, y):
            i = int(math.floor((x - message.info.origin.position.x) / message.info.resolution))
            j = int(math.floor((y - message.info.origin.position.y) / message.info.resolution))
            return message.data[j * message.info.width + i]
        assert at(2.0, 3.45) == OCCUPIED and at(2.0, 3.9) == 0

        saved = json.loads(detected_file.read_text())["detected"]
        assert len(saved) == 1 and saved[0]["lat"] == pytest.approx(rock["lat"])
        # The camera's frame from the press, kept beside the detections.
        photo = tmp_path / "detected_images" / saved[0]["image"]
        assert rock["image"] == saved[0]["image"] and photo.read_bytes()[:2] == b"\xff\xd8"
    finally:
        del keep
        node.destroy_node()
        navsat.node.destroy_node()


def test_detections_survive_a_restart(context, tmp_path):
    # A rock found earlier, written in lat/lon: 5 m east of the datum.
    detected_file = tmp_path / "detected.json"
    detected_file.write_text(json.dumps({"detected": [
        {"radius_m": 0.25, "count": 3, "first_seen": 0, "last_seen": 0,
         "lat": LAT0, "lon": LON0 + DLON_10M / 2}]}), encoding="utf-8")
    navsat = FakeNavsat(empty_answers=2)
    node = server(tmp_path, "", detected_file=str(detected_file))   # no site at all
    executor = SingleThreadedExecutor()
    executor.add_node(navsat.node)
    executor.add_node(node)
    try:
        # Not the instant all-free map: the detection needs converting first.
        message = listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)[-1]
        assert navsat.calls == 3

        def at(x, y):
            i = int(math.floor((x - message.info.origin.position.x) / message.info.resolution))
            j = int(math.floor((y - message.info.origin.position.y) / message.info.resolution))
            return message.data[j * message.info.width + i]
        assert at(5.0, 0.0) == OCCUPIED and at(5.4, 0.0) == 0
    finally:
        node.destroy_node()
        navsat.node.destroy_node()


def test_a_site_file_not_saved_yet_is_an_all_free_map(context, tmp_path):
    # The web page creates the file on its first save; until then the robot
    # must still have a survey map (or Nav2 aborts every goal).
    node = server(tmp_path, str(tmp_path / "not_yet.geojson"))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        (message,) = listen(executor, "obstacle_map", OccupancyGrid, LATCHED, bool)
        assert set(message.data) == {0}
    finally:
        node.destroy_node()
