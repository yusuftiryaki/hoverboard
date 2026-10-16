"""The mission layer's acceptance drive: mow, run low, go home, charge, come
back, finish — everything from the site file, on the real `map`.

    site.geojson ─┬─► mission_runner ─► rows ─────────────────────────────────┐
                  ├─► site_map_server ─► /obstacle_map (lawn's keep-outs, station body)
                  └─► battery_supervisor ─► home = the site's station (missions.md 3b)
    battery LOW ─► supervisor cancels the rows ─► staging ─► DockRobot(dock_pose) ─► CHARGING
    FULL ─► DOCKED_IDLE ─► mission_runner asks the supervisor to undock ─► NORMAL ─► rows again

Nothing here is test-only wiring: the supervisor gets the station from the same
site file the runner plans on, through navsat_transform like every other
lat/lon. docks.yaml keeps its placeholder at the map origin — the supervisor
must NOT be using it (the robot did not start on the dock, so that pose is
wrong by metres).

Quiet GPS: this proves the chain; the NEO-6M's effect on docking starts is a
separate, still unmeasured question (handoff, SP3-SP5).

Spawns one GPS + Nav2 + battery stack, ~10 min. CI stage `mission`.
"""

import math
import subprocess

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import Bool, String                   # noqa: E402


from test_mission_coverage import (                     # noqa: E402
    CUT_WIDTH, QUIET_GPS, Watcher, lawn_feature, lonlat, measure, start_mission)

# The station, docked base_link at (0, -3), the robot driving SOUTH into it
# (test_docking.py's station). Compass heading 180 = facing south.
DOCK_X, DOCK_Y, DOCK_YAW = 0.0, -3.0, -math.pi / 2
CAPACITY_AH = 0.1
# Load while mowing is ~0.5-0.8 A (battery_model: 1.5 A per m/s + turning), so
# from 0.33 the 0.30 line falls within the first rows, and a full pack then
# covers the rest of the lawn with plenty to spare.
SIM_ARGS = (*QUIET_GPS,
            "-p", f"dock_pose:=[{DOCK_X}, {DOCK_Y}, {DOCK_YAW}]",
            "-p", f"battery_capacity_ah:={CAPACITY_AH}",
            "-p", "battery_initial_soc:=0.33",
            "-p", "battery_idle_current_a:=0.0")
MONITOR = {"capacity_ah": CAPACITY_AH}
SUPERVISOR = {"return_soc": 0.30, "critical_soc": 0.05}
TIMEOUT_S = 1500.0


def station_feature():
    return {"type": "Feature",
            "geometry": {"type": "Point", "coordinates": lonlat(DOCK_X, DOCK_Y)},
            "properties": {"kind": "station", "name": "home", "heading_deg": 180.0}}


class DockWatcher(Watcher):
    def __init__(self):
        super().__init__()
        self.phases = []
        self.contact_seen = False
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/battery_supervisor/state", self._on_phase, latched)
        self.create_subscription(Bool, "/dock_contact_truth", self._on_contact, 10)

    def _on_phase(self, message):
        if not self.phases or self.phases[-1] != message.data:
            self.phases.append(message.data)

    def _on_contact(self, message):
        self.contact_seen = self.contact_seen or message.data


def test_mow_run_low_charge_at_the_sites_station_and_finish(site_dock_stack, tmp_path):
    stack = site_dock_stack([lawn_feature(), station_feature()], SIM_ARGS,
                            monitor=MONITOR, supervisor=SUPERVISOR)
    log_path = tmp_path / "mission_runner.log"
    with open(log_path, "w", encoding="utf-8") as log:
        runner = subprocess.Popen(
            ["ros2", "run", "robot_missions", "mission_runner", "--ros-args",
             "-p", f"site_file:={stack.site_file}",
             "-p", f"state_file:={tmp_path / 'mission_state.json'}",
             "-p", f"cut_width:={CUT_WIDTH}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stack._procs.append(runner)

    watcher = DockWatcher()
    try:
        assert watcher.spin_for(30.0, lambda: watcher.status is not None
                                and watcher.phases and watcher.phases[-1] == "NORMAL"), \
            f"no runner status or supervisor not NORMAL: {watcher.phases}"
        start_mission(watcher, {"type": "coverage", "area": "cim",
                                "params": {"overlap": 0.2}})

        finished = watcher.spin_for(
            TIMEOUT_S, lambda: watcher.status and watcher.status["state"] in ("DONE", "FAILED"))
        tail = "".join(open(log_path, encoding="utf-8").readlines()[-25:])
        assert finished, (f"not done in {TIMEOUT_S} s: {watcher.status}; supervisor "
                          f"{watcher.phases}\nmission_runner:\n{tail}")
        assert watcher.status["state"] == "DONE", f"{watcher.status}\n{tail}"

        # The whole trip, in order: home, docked, charged, full, undocked, back.
        trip = ["RETURNING", "DOCKING", "CHARGING", "DOCKED_IDLE", "UNDOCKING", "NORMAL"]
        it = iter(watcher.phases)
        assert all(phase in it for phase in trip), f"supervisor went {watcher.phases}"
        assert "SAFE_STOP" not in watcher.phases
        # Ground truth: the contacts really touched — at the SITE's station.
        # docks.yaml still says (0, 0) facing east; docking there would have
        # failed (no station, no marker) or driven into the lawn.
        assert watcher.contact_seen, "DockRobot finished but the contacts never touched"

        coverage, overrun = measure(watcher.track)
        print(f"mission with a charge: supervisor {watcher.phases}, skipped "
              f"{watcher.status['skipped']}, coverage {coverage:.1%}, overrun {overrun:.2f} m")
        assert watcher.status["skipped"] == []
        assert coverage >= 0.85, f"only {coverage:.1%} cut across the interruption"
        assert overrun <= 0.15
    finally:
        watcher.destroy_node()
