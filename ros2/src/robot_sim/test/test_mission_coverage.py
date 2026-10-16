"""A coverage mission, end to end, measured on ground truth. Skips without ROS.

    site.geojson ─► mission_runner ─/fromLLArray─► planner ─► navigate_to_pose × N ─► the wheels
                                                   GPS, madgwick, ekf_global, navsat_transform

What the robot ACTUALLY drove over (/ground_truth), against the lawn drawn in
the site file:
  * coverage — the share of the lawn within cut_width / 2 of the true track
  * overrun  — how far the true track went outside the lawn while cutting

These are mission-quality numbers (docs/missions.md decision 1: GPS error is
lived with, so its cost is measured, not assumed). The quiet-GPS run is the
wiring's pass/fail; the NEO-6M run is the honest number.

Spawns a GPS + Nav2 stack per run, a few minutes each. CI stage `mission`.
"""

import json
import math
import os
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from robot_missions_interfaces.srv import StartMission  # noqa: E402
from robot_sim.sim_node import EARTH_RADIUS_M           # noqa: E402

DATUM_LAT, DATUM_LON = 41.0, 29.0
QUIET_GPS = ("-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")
LAWN = [(2.0, -1.5), (6.0, -1.5), (6.0, 1.5), (2.0, 1.5)]       # sim_world, 4 x 3 m
CUT_WIDTH = 0.5
MISSION_TIMEOUT_S = 900.0


def lonlat(east, north):
    return [DATUM_LON + math.degrees(
                east / (EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)))),
            DATUM_LAT + math.degrees(north / EARTH_RADIUS_M)]


def lawn_feature():
    ring = [lonlat(*c) for c in LAWN] + [lonlat(*LAWN[0])]
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": "area", "name": "cim"}}


class Watcher(Node):
    def __init__(self):
        super().__init__("coverage_watcher")
        self.track = []                 # (x, y, cutting)
        self.status = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mission_runner/status", self._on_status, latched)
        self.create_subscription(Odometry, "/ground_truth", self._on_truth, 10)

    def _on_status(self, message):
        self.status = json.loads(message.data)

    def _on_truth(self, message):
        p = message.pose.pose.position
        cutting = bool(self.status and self.status["state"] == "RUNNING"
                       and self.status.get("work"))
        self.track.append((p.x, p.y, cutting))

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return False


def measure(track, step=0.05):
    """(coverage, overrun) of the lawn by the CUTTING part of the track."""
    cut = [(x, y) for x, y, c in track if c]
    xs = [LAWN[0][0] + step / 2 + i * step
          for i in range(int((LAWN[1][0] - LAWN[0][0]) / step))]
    ys = [LAWN[0][1] + step / 2 + j * step
          for j in range(int((LAWN[2][1] - LAWN[0][1]) / step))]
    reach = CUT_WIDTH / 2
    covered = 0
    for x in xs:
        near = [(px, py) for px, py in cut if abs(px - x) <= reach]
        for y in ys:
            if any((px - x) ** 2 + (py - y) ** 2 <= reach * reach for px, py in near):
                covered += 1
    x0, y0 = LAWN[0]
    x1, y1 = LAWN[2]
    overrun = max((math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
                   for x, y in cut), default=0.0)
    return covered / (len(xs) * len(ys)), overrun


def run_mission(site_nav2_stack, tmp_path, sim_args, mission_params, anchor="chain"):
    stack = site_nav2_stack([lawn_feature()], *sim_args)
    log_path = tmp_path / "mission_runner.log"
    log = open(log_path, "w", encoding="utf-8")
    runner = subprocess.Popen(
        ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
         "-p", f"site_file:={stack.site_file}",
         "-p", f"state_file:={tmp_path / 'mission_state.json'}",
         "-p", f"cut_width:={CUT_WIDTH}", "-p", f"anchor:={anchor}"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stack._procs.append(runner)          # killed with the stack

    watcher = Watcher()
    try:
        assert watcher.spin_for(20.0, lambda: watcher.status is not None), \
            "mission_runner published no status"
        client = watcher.create_client(StartMission, "/mission_runner/start")
        assert client.wait_for_service(timeout_sec=20.0)
        deadline = time.monotonic() + 30.0
        while True:                       # the supervisor's state may lag the runner
            request = StartMission.Request()
            request.mission = json.dumps({"type": "coverage", "area": "cim",
                                          "params": mission_params})
            future = client.call_async(request)
            while not future.done():
                rclpy.spin_once(watcher, timeout_sec=0.05)
            if future.result().accepted:
                break
            assert time.monotonic() < deadline, future.result().message
            watcher.spin_for(1.0)
        started = time.monotonic()
        finished = watcher.spin_for(
            MISSION_TIMEOUT_S,
            lambda: watcher.status and watcher.status["state"] in ("DONE", "FAILED", "PAUSED"))
        elapsed = time.monotonic() - started
        tail = "".join(open(log_path, encoding="utf-8").readlines()[-25:])
        assert finished, (f"mission did not finish in {MISSION_TIMEOUT_S} s: "
                          f"{watcher.status}\nmission_runner's log:\n{tail}")
        assert watcher.status["state"] == "DONE", f"{watcher.status}\nmission_runner's log:\n{tail}"
        coverage, overrun = measure(watcher.track)
        dump = os.environ.get("COVERAGE_TRACK_DUMP")
        if dump:                         # for looking at a run by hand
            with open(dump, "w", encoding="utf-8") as handle:
                json.dump({"track": watcher.track, "lawn": LAWN}, handle)
        print(f"coverage run ({' '.join(sim_args) or 'NEO-6M GPS'}, anchor {anchor}): "
              f"{watcher.status['total']} waypoints, skipped {watcher.status['skipped']}, "
              f"{elapsed:.0f} s, coverage {coverage:.1%}, overrun {overrun:.2f} m")
        return coverage, overrun, watcher.status
    finally:
        watcher.destroy_node()
        log.close()


def test_coverage_with_a_quiet_gps(site_nav2_stack, tmp_path):
    """The wiring's pass/fail: with the GPS quiet, `map` is the true world.

    MEASURED (anchor chain, 4 x 3 m lawn, 0.5 m cut, overlap 0.2): 96.2 /
    95.0 / 96.0 / 95.7% cut, 0.00 m overrun, ~200 s. For the record, the
    designs this replaced, same lawn: navigate_to_pose on the general profile
    63.4% / 0.71 m; one map->odom snapshot for the whole lawn 90.0% / 0.36 m
    (odom's gyro-only yaw slanted the rows ~15 deg).
    """
    coverage, overrun, status = run_mission(site_nav2_stack, tmp_path, QUIET_GPS,
                                            {"overlap": 0.2})
    assert status["skipped"] == []
    assert coverage >= 0.92, f"only {coverage:.1%} of the lawn was cut"
    assert overrun <= 0.15, f"cut {overrun:.2f} m outside the lawn"


def test_coverage_with_the_neo6m_gps(site_nav2_stack, tmp_path):
    """The honest number: sim_node's default GPS, 2 m wander (tau 60 s) + 1 m
    white noise — the NEO-6M class (decision 5).

    MEASURED, anchor chain: 18.6% / 2.17 m and 69.5% / 0.97 m. The rows stay
    true to each other; the whole pattern lands wherever the GPS put `map` when
    the mission started, and on a lawn this small that offset is most of the
    lawn. anchor leg (pattern follows the GPS row by row): 47.8% / 3.58 m and
    44.9% / 4.71 m — steadier, never good, and it cuts metres outside.
    docs/missions.md decision 1 accepted this cost; this test keeps it
    visible, it does not guard a number.
    """
    coverage, overrun, status = run_mission(site_nav2_stack, tmp_path, (), {"overlap": 0.2})
    assert status["state"] == "DONE"
