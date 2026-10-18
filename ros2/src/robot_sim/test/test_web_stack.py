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
import urllib.error
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
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        # The robot says WHY in the body; without it a failure reads "400".
        raise AssertionError(f"{method} {path} -> {exc.code}: {exc.read().decode()}") from None


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


def yaw_of(odometry):
    q = odometry.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def metres(lon, lat):
    """Inverse of lonlat(): sim_world east/north of the datum."""
    return (math.radians(lon - DATUM_LON) * EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)),
            math.radians(lat - DATUM_LAT) * EARTH_RADIUS_M)


def compass_error(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


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
                                         m.pose.pose.position.y, yaw_of(m))), 50)
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


class Driver:
    """Drives the robot from the page's API, closed-loop on /ground_truth —
    the test plays a careful person with perfect eyes."""

    def __init__(self, watch):
        self.watch = watch
        self.token = http("POST", "/api/teleop/acquire", {})["token"]

    def send(self, linear, angular):
        http("POST", "/api/teleop", {"token": self.token, "linear": linear, "angular": angular})
        time.sleep(0.1)

    def pose(self):
        return self.watch.truth[-1][1:]

    def to(self, x, y, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            px, py, yaw = self.pose()
            if math.hypot(x - px, y - py) < 0.15:
                return
            error = (math.atan2(y - py, x - px) - yaw + math.pi) % (2 * math.pi) - math.pi
            turn = max(-1.0, min(1.0, 2.0 * error))
            self.send(1.0 if abs(error) < 0.15 else 0.0, turn)
        raise AssertionError(f"did not reach ({x}, {y}); at {self.pose()}")

    def stop(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(0.0, 0.0)


def test_the_page_heading_is_the_true_compass_all_the_way_round(web_stack):
    """A6's lesson: a mirrored or offset heading looks perfect in ONE
    direction. Turn on the spot in eight 45-degree steps and, standing still
    at each, compare /api/status against /ground_truth.

    Standing still on purpose: sampled WHILE turning (0.5 rad/s) the status
    lags the truth by tf + HTTP, and under a loaded machine that read 11 deg
    of "error" that was only lag."""
    web_stack([LAWN], *QUIET_GPS)
    wait_http(lambda s: s["robot"] is not None and s["robot"]["heading_deg"] is not None,
              60.0, "a heading")
    watch = Watcher()
    try:
        deadline = time.monotonic() + 15.0
        while not watch.truth and time.monotonic() < deadline:
            time.sleep(0.1)
        driver = Driver(watch)
        errors = []
        for step in range(8):
            target = watch.truth[-1][3] + math.pi / 4
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                error = (target - watch.truth[-1][3] + math.pi) % (2 * math.pi) - math.pi
                if abs(error) < 0.03:
                    break
                driver.send(0.0, max(-1.0, min(1.0, 2.0 * error)))
            driver.stop(1.0)                            # settle: tf and EKF catch up
            heading = http("GET", "/api/status")["robot"]["heading_deg"]
            truth = (90.0 - math.degrees(watch.truth[-1][3])) % 360.0
            errors.append((truth, compass_error(heading, truth)))
        covered = {int(((t + 22.5) % 360) // 45) for t, _ in errors}
        assert covered == set(range(8)), f"sweep missed octants: {sorted(covered)} {errors}"
        worst = max(errors, key=lambda e: e[1])
        print(f"\nMEASURED compass sweep: worst {worst[1]:.2f} deg at true {worst[0]:.0f}")
        assert worst[1] < 3.0, f"compass off by {worst[1]:.1f} deg at true {worst[0]:.0f}"
    finally:
        watch.close()


@pytest.mark.parametrize("gps", ["quiet", "neo6m"])
def test_driving_round_a_lawn_records_it_and_a_station_is_measured(web_stack, gps):
    """5b's acceptance, against /ground_truth: the trace's polygon vs the path
    the robot really drove, and an averaged station vs the true pose.

    quiet: the recorder itself — the polygon sits on the driven path.
    neo6m: the default wandering GPS. The SHAPE (odometry) must survive it —
    compared after moving the centroids together — while the placement is off
    by whatever the GPS was off during the drive (reported, not asserted:
    decision 1). Before the shape came from odom, this drive crossed itself.
    """
    from shapely import affinity
    from shapely.geometry import LineString, Point, Polygon

    web_stack([LAWN], *(QUIET_GPS if gps == "quiet" else ()))
    wait_http(lambda s: s["robot"] is not None and s["teach"] is not None, 60.0, "teach")
    watch = Watcher()
    try:
        deadline = time.monotonic() + 15.0
        while not watch.truth and time.monotonic() < deadline:
            time.sleep(0.1)
        driver = Driver(watch)
        corners = [(6.0, 0.0), (6.0, 4.0), (0.0, 4.0), (0.0, 0.0)]
        http("POST", "/api/teach/trace", {"kind": "area"})
        began = time.monotonic()
        for x, y in corners:
            driver.to(x, y)
        ended = time.monotonic()
        result = http("POST", "/api/teach/finish", {})["result"]
        assert result["properties"] == {"kind": "area"}, result
        ring = [metres(*p) for p in result["geometry"]["coordinates"][0]]
        path = LineString([(p[1], p[2]) for p in watch.truth if began <= p[0] <= ended])
        recorded = Polygon(ring)
        driven = Polygon(path.coords)
        offset = math.hypot(recorded.centroid.x - driven.centroid.x,
                            recorded.centroid.y - driven.centroid.y)
        shape = affinity.translate(recorded, driven.centroid.x - recorded.centroid.x,
                                   driven.centroid.y - recorded.centroid.y)

        def gap(polygon):
            # Both ways, by hand. ⚠️ shapely's hausdorff_distance is over
            # vertices (a 4-corner ring vs a dense path read 7.2 m), and with
            # densify= it still read 7.2 m for a ring 0.04 m away.
            boundary = polygon.exterior
            return max(max(boundary.distance(Point(c)) for c in path.coords),
                       max(path.distance(boundary.interpolate(i / 1000, normalized=True))
                           for i in range(1001)))

        print(f"\nMEASURED trace ({gps}): {len(ring) - 1} corners, path gap "
              f"{gap(recorded):.3f} m, shape gap {gap(shape):.3f} m, offset {offset:.2f} m, "
              f"area {recorded.area:.2f} m2 vs driven {driven.area:.2f}, "
              f"GPS wander {result['quality']['anchor_wander_m']} m")
        assert 4 <= len(ring) - 1 <= 8, ring
        assert recorded.area == pytest.approx(driven.area, rel=0.08)
        assert gap(shape) < 0.4, f"the recorded SHAPE is {gap(shape):.2f} m off the driven one"
        if gps == "quiet":
            assert gap(recorded) < 0.4, f"recorded boundary {gap(recorded):.2f} m off the path"
        assert http("POST", "/api/teach/take", {})["result"]["id"] == result["id"]

        # Station: stand still, average 3 s of fixes and heading.
        driver.stop(1.0)
        http("POST", "/api/teach/point", {"kind": "station", "seconds": 3})
        taken = None
        deadline = time.monotonic() + 20.0
        while taken is None and time.monotonic() < deadline:
            driver.send(0.0, 0.0)
            if http("GET", "/api/status")["teach"]["result_ready"]:
                taken = http("POST", "/api/teach/take", {})["result"]
        assert taken is not None and "error" not in taken, taken
        _, x, y, yaw = watch.truth[-1]
        sx, sy = metres(*taken["geometry"]["coordinates"])
        truth_compass = (90.0 - math.degrees(yaw)) % 360.0
        heading_error = compass_error(taken["properties"]["heading_deg"], truth_compass)
        print(f"MEASURED station ({gps}): {math.hypot(sx - x, sy - y):.3f} m, heading "
              f"{heading_error:.2f} deg, spread {taken['quality']['spread_m']} m")
        if gps == "quiet":
            assert math.hypot(sx - x, sy - y) < 0.2
        assert heading_error < 3.0
    finally:
        watch.close()
