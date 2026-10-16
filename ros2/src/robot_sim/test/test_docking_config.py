"""The docking chain's numbers that live in several files, held together.

Each pair below is one physical fact written down twice, in files that do not
read each other. Nothing errors when they drift: the robot just stages in the
wrong place, ends the dock on the wrong current, or locks out the person
trying to drive it. Same idea as test_gazebo_config.py — YAML only, no ROS,
milliseconds.
"""

import math
import os

import yaml

from robot_sim.dock_model import DockStation

CONFIG = os.path.join(os.path.dirname(__file__), "..", "..", "robot_bringup", "config")


def load(name):
    with open(os.path.join(CONFIG, name), encoding="utf-8") as handle:
        return yaml.safe_load(handle)


NAV2 = load("nav2.yaml")
DOCKING = NAV2["docking_server"]["ros__parameters"]
PLUGIN = DOCKING[DOCKING["dock_plugins"][0]]
BATTERY = load("battery_manager.yaml")
SUPERVISOR = BATTERY["battery_supervisor"]["ros__parameters"]
MUX = load("twist_mux.yaml")["twist_mux"]["ros__parameters"]


def test_the_supervisor_stages_where_the_dock_server_retries_from():
    assert SUPERVISOR["staging_distance"] == -PLUGIN["staging_x_offset"]


def test_charging_means_the_same_current_to_both():
    """The dock server ends DockRobot on it; the supervisor enters CHARGING on
    it. Different numbers = one says charging while the other waits."""
    assert SUPERVISOR["charging_current_a"] == PLUGIN["charging_threshold"]


def test_the_supervisor_docks_the_type_the_server_has():
    assert SUPERVISOR["dock_type"] in DOCKING["dock_plugins"]


def test_home_is_surveyed_in_map_and_is_the_dock_the_supervisor_asks_for():
    docks = load("docks.yaml")["docks"]
    home = docks[SUPERVISOR["dock_id"]]
    assert home.get("frame", "map") == "map"           # handoff decision 12
    assert home["type"] == SUPERVISOR["dock_type"]
    assert len(home["pose"]) == 3 and all(math.isfinite(v) for v in home["pose"])


def test_the_marker_offset_is_the_simulated_stations():
    """nav2.yaml carries the real robot's offset from marker to docked pose;
    robot_sim's station is built to the same design. If SP6 builds it
    differently, change both — the e2e docking test only means something while
    they agree."""
    assert PLUGIN["external_detection_translation_x"] == -DockStation(0, 0, 0).tag_distance
    assert PLUGIN["external_detection_translation_y"] == 0.0


def test_the_marker_convention_rotation_is_a_right_angle_pair():
    """Nav2's defaults for an aruco/AprilTag detector: roll -pi/2, pitch +pi/2."""
    assert math.isclose(PLUGIN["external_detection_rotation_roll"], -math.pi / 2, abs_tol=1e-4)
    assert math.isclose(PLUGIN["external_detection_rotation_pitch"], math.pi / 2, abs_tol=1e-4)
    assert PLUGIN["external_detection_rotation_yaw"] == 0.0


def test_the_battery_lock_stops_nav2_and_never_teleop():
    """twist_mux masks inputs whose priority is BELOW the lock's."""
    lock = MUX["locks"]["battery"]["priority"]
    assert MUX["topics"]["navigation"]["priority"] < lock < MUX["topics"]["teleop"]["priority"]
    assert MUX["locks"]["battery"]["topic"] == "nav_lock"


def test_return_comes_before_critical():
    assert 0.0 <= SUPERVISOR["critical_soc"] < SUPERVISOR["return_soc"] <= 1.0
    assert 0.0 < SUPERVISOR["critical_cell_v"] < SUPERVISOR["return_cell_v"]
