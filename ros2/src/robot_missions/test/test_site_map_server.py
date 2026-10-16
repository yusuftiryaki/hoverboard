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
from robot_localization.srv import FromLLArray                      # noqa: E402

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


@pytest.mark.parametrize("dock_pose, level", [
    ([0.0, 0.0, math.pi / 2], DiagnosticStatus.OK),        # north = +90 deg yaw
    ([0.0, 0.0, 0.0], DiagnosticStatus.ERROR),             # docks.yaml's placeholder
])
def test_the_station_is_checked_against_docks_yaml(context, tmp_path, dock_pose, level):
    docks = tmp_path / "docks.yaml"
    docks.write_text(json.dumps({"docks": {"home": {
        "type": "charging_dock", "frame": "map", "pose": dock_pose}}}), encoding="utf-8")
    navsat = FakeNavsat(empty_answers=0)
    node = server(tmp_path, write_site(tmp_path, station_facing_north()),
                  dock_database=str(docks))
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
