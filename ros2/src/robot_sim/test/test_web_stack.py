"""The web page's backend on a running robot stack. Skips without ROS, ~2 min.

web_api's own tests use a stand-in robot; this is web_ui against the real
nodes: the robot's position and heading come back right, a site edited over
HTTP reaches the costmaps' survey map through site_map_server's reload, and
driving from the page (docs/missions.md 5a) moves the robot through twist_mux
and the bridge — and stops it when the page goes quiet.
"""

import json
import math
import threading
import time
import urllib.request

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from geometry_msgs.msg import Twist                     # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry        # noqa: E402
from rclpy.executors import SingleThreadedExecutor      # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

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


class Watcher:
    """Records /cmd_vel, /cmd_vel_teleop, teleop/state and /ground_truth with
    wall-clock arrival times, spinning in its own thread while the test talks
    HTTP."""

    def __init__(self):
        self.node = rclpy.create_node("web_teleop_watcher")
        self.cmd, self.teleop, self.states, self.truth = [], [], [], []
        stamp = time.monotonic
        self.node.create_subscription(
            Twist, "/cmd_vel", lambda m: self.cmd.append((stamp(), m.linear.x, m.angular.z)), 50)
        self.node.create_subscription(
            Twist, "/cmd_vel_teleop", lambda m: self.teleop.append((stamp(), m.linear.x)), 50)
        self.node.create_subscription(
            String, "/teleop/state", lambda m: self.states.append((stamp(), json.loads(m.data))),
            50)
        self.node.create_subscription(
            Odometry, "/ground_truth",
            lambda m: self.truth.append((stamp(), m.pose.pose.position.x,
                                         m.pose.pose.position.y)), 50)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def truth_at(self, t):
        before = [p for p in self.truth if p[0] <= t]
        return before[-1] if before else None

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.node.destroy_node()


def test_driving_from_the_page_moves_the_robot_and_silence_stops_it(web_stack):
    web_stack([LAWN], *QUIET_GPS)
    wait_http(lambda s: s["robot"] is not None and s["teleop"] is not None, 60.0,
              "the web page with teleop")
    watch = Watcher()
    try:
        deadline = time.monotonic() + 15.0
        while not watch.truth and time.monotonic() < deadline:
            time.sleep(0.1)
        assert watch.truth, "no /ground_truth"
        lease = http("POST", "/api/teleop/acquire", {})
        assert lease["max_linear"] == pytest.approx(0.6)
        # 3 s at half stick, 10 Hz — what the page sends.
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            http("POST", "/api/teleop", {"token": lease["token"], "linear": 0.5, "angular": 0.0})
            last_sent = time.monotonic()      # the last command has arrived
            time.sleep(0.1)
        time.sleep(3.5)                       # past lease_s (2.0) with margin

        driving = [c for c in watch.cmd if start < c[0] < last_sent]
        assert driving, "nothing reached /cmd_vel: teleop did not go through twist_mux"
        assert max(c[1] for c in driving) == pytest.approx(0.3, abs=1e-6), \
            "the page's half stick must be half of max_linear, through the mux unchanged"

        # The robot faces east (+x) at the start.
        p0, p1 = watch.truth_at(start), watch.truth_at(last_sent)
        assert p0 is not None and p1 is not None, "no /ground_truth"
        assert 0.6 < p1[1] - p0[1] < 1.0 and abs(p1[2] - p0[2]) < 0.1, (p0, p1)

        # Silence: the first zero on /cmd_vel comes from web_ui's deadman
        # (0.3 s), not from the bridge's 0.5 s cmd_timeout behind it.
        zero = next((c for c in watch.cmd if c[0] > last_sent and c[1] == 0.0), None)
        assert zero is not None, "the robot was never told to stop"
        assert zero[0] - last_sent < 0.3 + 0.15, f"stop came {zero[0] - last_sent:.2f} s late"
        # 0.3 m/s × ~0.32 s ≈ 0.10 m, plus braking: measured 0.12 m (3 runs).
        # At the page's full 0.6 m/s a dropped Wi-Fi costs about twice that.
        rest = watch.truth_at(last_sent + 1.5)
        coast = math.hypot(rest[1] - p1[1], rest[2] - p1[2])
        assert coast < 0.15, f"the robot rolled {coast:.2f} m after the page went quiet"

        # Past lease_s web_ui publishes nothing: the mux would give the wheels
        # back to Nav2. And the state says so, for the episode recorder.
        late = [t for t in watch.teleop if t[0] > last_sent + 2.0 + 0.2]
        assert not late, f"/cmd_vel_teleop still published {len(late)} times after the lease"
        assert watch.states and watch.states[-1][1]["held"] is False
        assert any(s["active"] for _, s in watch.states), "teleop/state never saw driving"
    finally:
        watch.close()
