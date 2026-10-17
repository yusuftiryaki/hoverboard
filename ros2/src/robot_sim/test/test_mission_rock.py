"""A rock nobody surveyed, in the middle of the lawn: the bumper finds it, the
map learns it, the mission mows round it and finishes. Skips without ROS.

    row ─► bumper presses (sim: world.bumper_pressed) ─► ESP32 vetoes forward
        ─► site_map_server: detected obstacle on /obstacle_map + on disk (lat/lon)
        ─► the row's goal fails ─► mission_runner retries it, cut round the
           detection (planner.free_intervals) ─► the rows after it too

docs/missions.md, "Robotun kendi tespit ettiği engeller", step 1. What it does
NOT do: see the rock before touching it (no range sensor, decision 8), or
write it into the site — a person confirms it later.

Quiet GPS. One GPS + Nav2 stack, ~5 min. CI stage `mission`.
"""

import json
import math
import os
import subprocess

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from std_msgs.msg import Bool                           # noqa: E402

from robot_sim.sim_node import EARTH_RADIUS_M           # noqa: E402

from test_mission_coverage import (                     # noqa: E402
    CUT_WIDTH, DATUM_LAT, DATUM_LON, QUIET_GPS, Watcher, lawn_feature, measure,
    start_mission)

# On the middle row (rows at y = -1.2 ... 1.2, 0.4 apart): the rows at
# y = -0.4, 0 and 0.4 all pass within robot radius + rock radius = 0.6 m.
ROCK_X, ROCK_Y, ROCK_R = 4.0, 0.0, 0.2
SIM_ARGS = (*QUIET_GPS,
            "-p", f"unsurveyed_obstacle_centers:=[{ROCK_X}, {ROCK_Y}]",
            "-p", f"unsurveyed_obstacle_radii:=[{ROCK_R}]",
            "-p", "bumper_from_obstacles:=true")
TIMEOUT_S = 1200.0


class RockWatcher(Watcher):
    def __init__(self):
        super().__init__()
        self.collisions = 0             # rising edges of the sim's oracle
        self._colliding = False
        self.create_subscription(Bool, "/collision_truth", self._on_collision, 10)

    def _on_collision(self, message):
        if message.data and not self._colliding:
            self.collisions += 1
        self._colliding = message.data


def to_sim_world(lat, lon):
    """lat/lon -> sim_world metres, as sim_node's GPS defines it."""
    return (math.radians(lon - DATUM_LON) * EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)),
            math.radians(lat - DATUM_LAT) * EARTH_RADIUS_M)


def test_the_bumper_finds_an_unsurveyed_rock_and_the_mission_mows_round_it(
        site_nav2_stack, tmp_path):
    stack = site_nav2_stack([lawn_feature()], *SIM_ARGS)
    log_path = tmp_path / "mission_runner.log"
    with open(log_path, "w", encoding="utf-8") as log:
        runner = subprocess.Popen(
            ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
             "-p", f"site_file:={stack.site_file}",
             "-p", f"state_file:={tmp_path / 'mission_state.json'}",
             "-p", f"cut_width:={CUT_WIDTH}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stack._procs.append(runner)

    watcher = RockWatcher()
    try:
        assert watcher.spin_for(30.0, lambda: watcher.status is not None)
        start_mission(watcher, {"type": "coverage", "area": "cim", "params": {"overlap": 0.2}})
        finished = watcher.spin_for(
            TIMEOUT_S,
            lambda: watcher.status and watcher.status["state"] in ("DONE", "FAILED", "PAUSED"))
        tail = "".join(open(log_path, encoding="utf-8").readlines()[-30:])
        assert finished and watcher.status["state"] == "DONE", (
            f"{watcher.status}\ncollisions {watcher.collisions}\nmission_runner:\n{tail}")

        with open(stack.detected_file, encoding="utf-8") as handle:
            detected = json.load(handle)["detected"]
        where = [to_sim_world(d["lat"], d["lon"]) for d in detected]
        coverage, overrun = measure(watcher.track)
        if os.environ.get("COVERAGE_TRACK_DUMP"):
            with open(os.environ["COVERAGE_TRACK_DUMP"], "w", encoding="utf-8") as handle:
                json.dump({"track": watcher.track, "where": where}, handle)
        print(f"rock: {watcher.collisions} collision(s), detections at "
              f"{[(round(x, 2), round(y, 2)) for x, y in where]}, skipped "
              f"{watcher.status['skipped']}, coverage {coverage:.1%}, overrun {overrun:.2f} m")

        # Found, and found where it is. A contact point lies on the rock's
        # surface (0.2 m from its centre) — as well as the robot knew where IT
        # was when the bumper pressed. MEASURED (quiet GPS): 0.17 and 0.53 m
        # from the centre, i.e. up to ~0.35 m of the robot's own localisation
        # error at that moment (after backing off and turning).
        assert detected, "the bumper pressed but nothing was recorded"
        assert all(math.hypot(x - ROCK_X, y - ROCK_Y) <= 0.6 for x, y in where), where
        # Learned once: after the first press, the rows that would have hit it
        # again are cut round it. Two is allowed (a press while backing off).
        assert watcher.collisions <= 2, f"hit the rock {watcher.collisions} times"
        # The rest of the lawn still gets mown. MEASURED 75.1% (the rock-free
        # lawn: 95-96%). The price is the clearance kept round each
        # detection — 0.25 m circle + robot 0.4 + 0.1 + detection_margin 0.3,
        # about a metre from its centre, so a little rock blanks a lot of
        # lawn. Tighter, and the rows' cut ends sat in the costmap's inscribed
        # zone: three rows skipped (70.7%). A person labelling the rock with
        # its real size (phase 4) gives the lawn back.
        assert coverage >= 0.70, f"only {coverage:.1%} cut"
    finally:
        watcher.destroy_node()
