"""Mission specs, the task registry, and planning through a converter. ROS-free.

The converter is the same flat-earth ENU stand-in as test_site_map.py (the
real /fromLLArray is exercised in the stack tests); positions are hand-computed
literals around (41 N, 29 E): 10 m north = 8.993216e-05 deg, 10 m east =
1.191613e-04 deg.
"""

import math

import pytest

from robot_missions.mission import (
    TASK_TYPES, MissionError, RobotSpec, mission_positions, parse_mission,
    plan_from_answer, plan_mission)
from robot_missions.planner import RING_TOLERANCE
from robot_missions.site import parse_site
from robot_missions.site_map import NotReady

LAT0, LON0 = 41.0, 29.0
DLAT_10M = 8.993216e-05
DLON_10M = 1.191613e-04
R = 6371000.0


def enu(points, rotate_deg=0.0):
    c, s = math.cos(math.radians(rotate_deg)), math.sin(math.radians(rotate_deg))
    out = []
    for p in points:
        east = math.radians(p.lon - LON0) * R * math.cos(math.radians(LAT0))
        north = math.radians(p.lat - LAT0) * R
        out.append((c * east - s * north, s * east + c * north))
    return out


def ll(east_m, north_m):
    return [LON0 + DLON_10M * east_m / 10, LAT0 + DLAT_10M * north_m / 10]


def polygon(*corners):
    ring = [ll(*c) for c in corners]
    return {"type": "Polygon", "coordinates": [ring + [ring[0]]]}


def feature(kind, name, geometry, **properties):
    return {"type": "Feature", "geometry": geometry,
            "properties": {"kind": kind, "name": name, **properties}}


def site(*extra):
    return parse_site({"type": "FeatureCollection", "features": [
        # A 20 x 6 m lawn, its long side east-west.
        feature("area", "bahce", polygon((0, 0), (20, 0), (20, 6), (0, 6))),
        feature("route", "tur", {"type": "LineString",
                                 "coordinates": [ll(0, 0), ll(10, 0), ll(10, 10)]}),
        *extra,
    ]})


ROBOT = RobotSpec(cut_width=0.5, robot_radius=0.4, obstacle_margin=0.1)


# --- parsing ---------------------------------------------------------------------

def test_defaults_fill_in_and_given_params_override():
    mission = parse_mission({"type": "coverage", "area": "bahce",
                             "params": {"overlap": 0.3}}, site())
    assert mission.type is TASK_TYPES["coverage"]
    assert mission.params == {"overlap": 0.3, "angle_deg": None,
                              "boundary_margin": None, "perimeter_pass": True}


@pytest.mark.parametrize("spec, words", [
    ({"type": "mow", "area": "bahce"}, "type must be one of"),
    ({"type": "coverage", "area": "cimen"}, "no area named 'cimen'"),
    ({"type": "coverage", "route": "tur"}, "runs on 'area'"),
    ({"type": "coverage", "area": "bahce", "params": {"overlab": 0.3}}, "no parameter"),
    ({"type": "coverage", "area": "bahce", "params": {"overlap": 1.5}}, "<= 0.9"),
    ({"type": "coverage", "area": "bahce", "params": {"overlap": "0.3"}}, "must be a float"),
    ({"type": "coverage", "area": "bahce", "params": {"perimeter_pass": 1}}, "must be a bool"),
    ({"type": "patrol", "area": "bahce", "params": {"mode": "zigzag"}}, "one of"),
    ({"type": "patrol", "area": "bahce", "params": {"laps": 1.5}}, "must be a int"),
    ({"type": "route", "route": "yok"}, "no route named"),
    ({"type": "goto", "target": [29.0]}, r"\[lon, lat\]"),
    ({"type": "goto", "target": [29.0, 41.0], "colour": "red"}, "unknown mission field"),
    ("coverage", "an object"),
])
def test_a_bad_spec_is_refused_with_the_reason(spec, words):
    with pytest.raises(MissionError, match=words):
        parse_mission(spec, site())


def test_an_int_is_accepted_where_a_float_is_asked():
    mission = parse_mission({"type": "coverage", "area": "bahce",
                             "params": {"angle_deg": 90}}, site())
    assert mission.params["angle_deg"] == 90.0


def test_every_registered_type_describes_itself():
    # The web form (phase 4) is built from this, not from a hard-coded list.
    for task in TASK_TYPES.values():
        assert task.describe and task.on in ("area", "route", "target")
        for param in task.params.values():
            if param.default is not None:
                assert param.check("x", param.default) == param.default


# --- planning through the converter -----------------------------------------------

def test_coverage_of_the_lawn_by_hand():
    # Auto angle -> rows east-west. Margin defaults to cut_width / 2 = 0.25:
    # centre lines inside x 0.25..19.75, y 0.25..5.75 (5.5 m tall). Spacing
    # 0.5 * (1 - 0.0) = 0.5 -> ceil(11) = 11 rows at y 0.5, 1.0, ..., 5.5.
    plan = plan_mission(site(), parse_mission(
        {"type": "coverage", "area": "bahce",
         "params": {"overlap": 0.0, "perimeter_pass": False}}, site()), enu, ROBOT)
    ys = sorted({round(w.y, 3) for w in plan.waypoints})
    assert ys == pytest.approx([0.5 + 0.5 * k for k in range(11)], abs=2e-3)
    xs = {round(w.x, 2) for w in plan.waypoints}
    assert xs == {0.25, 19.75}
    assert plan.info["rows"] == 11


@pytest.mark.parametrize("bearing, map_deg", [(90.0, 0.0), (0.0, 90.0), (45.0, 45.0),
                                              (135.0, -45.0)])
def test_a_compass_row_angle_becomes_the_map_angle(bearing, map_deg):
    # Rows "along bearing 0" run north-south: map angle 90 deg. Swept.
    plan = plan_mission(site(), parse_mission(
        {"type": "coverage", "area": "bahce", "params": {"angle_deg": bearing}}, site()),
        enu, ROBOT)
    got = math.degrees(plan.info["angle_rad"])
    assert (got - map_deg + 90) % 180 - 90 == pytest.approx(0.0, abs=0.01)


def test_the_row_angle_goes_through_the_converter():
    # In a map rotated 10 deg CCW, east-west rows sit at 10 deg.
    plan = plan_mission(site(), parse_mission(
        {"type": "coverage", "area": "bahce", "params": {"angle_deg": 90.0}}, site()),
        lambda pts: enu(pts, 10.0), ROBOT)
    assert math.degrees(plan.info["angle_rad"]) == pytest.approx(10.0, abs=0.01)


def test_survey_obstacles_are_cut_out_of_the_coverage():
    # A tree at (10, 3), r 0.5, grown by robot_radius + margin = 0.5: no work
    # leg may pass within 1.0 m of it, less the edge pass's ring
    # simplification (RING_TOLERANCE, which cuts chords inward). A keep-out
    # covering the east 5 m: no pose east of x = 15 - 0.5.
    tree = feature("obstacle", "agac", {"type": "Point", "coordinates": ll(10, 3)},
                   radius_m=0.5)
    bed = feature("keepout", "cicek", polygon((15, -1), (21, -1), (21, 7), (15, 7)))
    s = site(tree, bed)
    plan = plan_mission(s, parse_mission({"type": "coverage", "area": "bahce"}, s),
                        enu, ROBOT)
    assert max(w.x for w in plan.waypoints) <= 14.5 + 1e-3
    for a, b in zip(plan.waypoints, plan.waypoints[1:]):
        if b.work:
            # distance from (10, 3) to the leg
            ax, ay, bx, by = a.x - 10, a.y - 3, b.x - 10, b.y - 3
            t = max(0.0, min(1.0, -(ax * (bx - ax) + ay * (by - ay))
                             / max(1e-12, (bx - ax) ** 2 + (by - ay) ** 2)))
            assert math.hypot(ax + t * (bx - ax), ay + t * (by - ay)) >= \
                1.0 - RING_TOLERANCE - 2e-3


def test_route_and_goto_come_back_in_map():
    s = site()
    plan = plan_mission(s, parse_mission({"type": "route", "route": "tur"}, s), enu, ROBOT)
    assert [(round(w.x, 2), round(w.y, 2)) for w in plan.waypoints] == [
        (0, 0), (10, 0), (10, 10)]
    plan = plan_mission(s, parse_mission(
        {"type": "goto", "target": ll(3, 4), "params": {"heading_deg": 0.0}}, s), enu, ROBOT)
    (pose,) = plan.waypoints
    assert (round(pose.x, 2), round(pose.y, 2)) == (3, 4)
    assert math.degrees(pose.yaw) == pytest.approx(90.0, abs=0.01)    # north


def test_patrol_runs_round_the_inset_lawn():
    s = site()
    plan = plan_mission(s, parse_mission(
        {"type": "patrol", "area": "bahce", "params": {"step": 50.0, "laps": 0}}, s),
        enu, ROBOT, start=(0, 0))
    corners = [(round(w.x, 2), round(w.y, 2)) for w in plan.waypoints]
    assert corners == [(0.25, 0.25), (19.75, 0.25), (19.75, 5.75), (0.25, 5.75), (0.25, 0.25)]
    assert plan.repeat == 0


def test_one_conversion_call_for_everything():
    calls = []

    def counting(points):
        calls.append(len(points))
        return enu(points)

    tree = feature("obstacle", "agac", {"type": "Point", "coordinates": ll(10, 3)},
                   radius_m=0.5)
    s = site(tree)
    mission = parse_mission({"type": "coverage", "area": "bahce",
                             "params": {"angle_deg": 90.0}}, s)
    plan_mission(s, mission, counting, ROBOT)
    # tree centre (1) + lawn ring (4) + heading pair (2)
    assert calls == [7] == [len(mission_positions(s, mission))]


@pytest.mark.parametrize("answer", [[], [(0.0, 0.0)]])
def test_a_short_answer_is_not_ready(answer):
    s = site()
    mission = parse_mission({"type": "coverage", "area": "bahce"}, s)
    with pytest.raises(NotReady):
        plan_from_answer(s, mission, answer, ROBOT)
