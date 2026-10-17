"""The web page's backend on a running robot stack. Skips without ROS, ~1 min.

web_api's own tests use a stand-in robot; this is web_ui against the real
nodes: the robot's position and heading come back right, and a site edited
over HTTP reaches the costmaps' survey map through site_map_server's reload.
"""

import json
import math
import time
import urllib.request

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from nav_msgs.msg import OccupancyGrid                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402

from robot_missions.occupancy import OCCUPIED           # noqa: E402
from robot_sim.sim_node import EARTH_RADIUS_M           # noqa: E402

from conftest import WebStack                           # noqa: E402

DATUM_LAT, DATUM_LON = 41.0, 29.0
QUIET_GPS = ("-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")
BASE = f"http://127.0.0.1:{WebStack.WEB_PORT}"


def lonlat(east, north):
    return [DATUM_LON + math.degrees(
                east / (EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)))),
            DATUM_LAT + math.degrees(north / EARTH_RADIUS_M)]


def polygon(kind, name, corners):
    ring = [lonlat(*c) for c in corners] + [lonlat(*corners[0])]
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": kind, "name": name}}


LAWN = polygon("area", "cim", [(2, -1.5), (6, -1.5), (6, 1.5), (2, 1.5)])
BED = polygon("keepout", "cicek", [(3, 3), (5, 3), (5, 4), (3, 4)])


def http(method, path, body=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(BASE + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read())


def wait_http(predicate, timeout, what):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = http("GET", "/api/status")
            if predicate(last):
                return last
        except OSError:
            pass
        time.sleep(0.5)
    raise AssertionError(f"{what} within {timeout} s; last status {last}")


def latest_map(timeout=10.0):
    node = rclpy.create_node("web_map_listener")
    got = []
    node.create_subscription(OccupancyGrid, "/obstacle_map", got.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return got[-1] if got else None
    finally:
        node.destroy_node()


def cell(message, x, y):
    info = message.info
    i = int(math.floor((x - info.origin.position.x) / info.resolution))
    j = int(math.floor((y - info.origin.position.y) / info.resolution))
    return message.data[j * info.width + i]


def test_the_web_backend_sees_the_robot_and_edits_reach_the_map(web_stack):
    web_stack([LAWN], *QUIET_GPS)
    status = wait_http(lambda s: s["robot"] is not None, 60.0, "a robot position")
    # The robot starts at sim_world (0, 0) facing east: compass 90.
    assert status["robot"]["lat"] == pytest.approx(DATUM_LAT, abs=2e-6)
    assert status["robot"]["lon"] == pytest.approx(DATUM_LON, abs=2e-6)
    heading = status["robot"]["heading_deg"]
    assert heading is not None and abs((heading - 90.0 + 180) % 360 - 180) < 10.0

    assert http("GET", "/api/site")["features"] == [LAWN]
    before = latest_map()
    assert before is not None and cell(before, 4.0, 3.5) != OCCUPIED

    answer = http("PUT", "/api/site", {"type": "FeatureCollection", "features": [LAWN, BED]})
    # No Nav2 here, so no battery_supervisor: skipped, not a failure.
    assert answer["saved"] and answer["reloaded"], answer
    after = latest_map(timeout=15.0)
    assert cell(after, 4.0, 3.5) == OCCUPIED, "the keep-out drawn on the page is not on the map"
    assert cell(after, 4.0, 2.4) != OCCUPIED
