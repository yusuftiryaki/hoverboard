"""teach.py: traces and averaged points from the robot's fixes. ROS-free.

Positions are hand-computed around (41.0 N, 29.0 E), as in test_site.py:
    1 m north = 8.993216e-06 deg lat, 1 m east = 1.191613e-05 deg lon
"""

import math

import pytest

from robot_missions.site import LatLon, parse_site
from robot_missions.teach import (
    Teacher, TeachError, apply, bearing_difference, circular_mean_deg, compass_from_yaw,
    mean_pose, simplify)

LAT0, LON0 = 41.0, 29.0
DLAT_1M = 8.993216e-06
DLON_1M = 1.191613e-05


def at(east, north):
    return LatLon(LAT0 + north * DLAT_1M, LON0 + east * DLON_1M)


def metres(lonlat):
    return ((lonlat[0] - LON0) / DLON_1M, (lonlat[1] - LAT0) / DLAT_1M)


# --- compass: the A6 lesson — sweep, never one heading ---------------------------

@pytest.mark.parametrize("yaw_deg, compass", [(0, 90), (90, 0), (180, 270), (-90, 180),
                                              (45, 45), (-45, 135), (135, 315)])
def test_compass_from_yaw_hand_computed(yaw_deg, compass):
    # map is ENU: yaw 0 faces east, 90 faces north.
    assert compass_from_yaw(math.radians(yaw_deg)) == pytest.approx(compass, abs=1e-9)


def test_compass_from_yaw_all_the_way_round():
    for compass in range(0, 360, 7):
        yaw = math.radians(90 - compass)
        for wrap in (-2 * math.pi, 0.0, 2 * math.pi):
            got = compass_from_yaw(yaw + wrap)
            assert 0.0 <= got < 360.0
            assert abs((got - compass + 180) % 360 - 180) < 1e-9


def test_circular_mean_wraps_through_north():
    assert bearing_difference(circular_mean_deg([350, 10]), 0.0) < 1e-9
    assert circular_mean_deg([80, 100]) == pytest.approx(90.0)
    assert circular_mean_deg([0, 180]) is None
    assert circular_mean_deg([]) is None


# --- simplification --------------------------------------------------------------

def test_simplify_drops_points_on_a_line_but_keeps_corners_and_pins():
    line = [(x, 0.0) for x in range(11)] + [(10.0, y) for y in range(1, 11)]
    pinned = [False] * len(line)
    assert simplify(line, pinned, 0.3) == [(0, 0), (10, 0), (10, 10)]
    pinned[5] = True
    assert simplify(line, pinned, 0.3) == [(0, 0), (5, 0), (10, 0), (10, 10)]


def test_simplify_keeps_a_bump_bigger_than_the_tolerance():
    path = [(0, 0), (1, 0), (2, 0.5), (3, 0), (4, 0)]
    assert (2, 0.5) in simplify(path, [False] * 5, 0.3)
    assert (2, 0.5) not in simplify(path, [False] * 5, 0.6)


@pytest.mark.parametrize("pose, point, expected", [
    ((0, 0, 0), (1, 2), (1, 2)),
    ((10, -5, 0), (1, 2), (11, -3)),
    ((0, 0, math.pi / 2), (1, 0), (0, 1)),          # x turned onto y
    ((3, 4, math.pi), (1, 2), (2, 2)),
])
def test_apply_composes_a_pose_and_a_point(pose, point, expected):
    assert apply(pose, point) == pytest.approx(expected, abs=1e-12)


def test_mean_pose_wraps_yaw_through_pi():
    x, y, yaw = mean_pose([(0, 0, math.pi - 0.1), (2, 4, -math.pi + 0.1)])
    assert (x, y) == (1, 2) and abs(abs(yaw) - math.pi) < 1e-9


# --- traces ----------------------------------------------------------------------

def to_ll(points):
    """Stand-in for navsat_transform's /toLL: map metres → lat/lon around the
    datum, with the hand-computed degrees above."""
    return [at(x, y) for x, y in points]


def drive(teacher, waypoints, step=0.1, t0=0.0, hz=10.0, anchor=(0.0, 0.0, 0.0),
          jitter=None):
    """Feed poses along straight legs between odom waypoints (metres),
    yaw along each leg, `step` metres apart. map->odom is `anchor`, or
    anchor + jitter(i) to play a wandering GPS."""
    t, i = t0, 0
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        for k in range(int(round(length / step)) + 1):
            f = k * step / length
            odom = (x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw)
            map_odom = anchor if jitter is None else tuple(
                a + d for a, d in zip(anchor, jitter(i)))
            teacher.feed(at(*apply(map_odom, odom[:2])), None, t, odom, map_odom)
            t += 1.0 / hz
            i += 1
    return t


def corners_m(result):
    g = result["geometry"]
    coords = g["coordinates"][0][:-1] if g["type"] == "Polygon" else g["coordinates"]
    return [metres(p) for p in coords]


def close(got, want, tol):
    return len(got) == len(want) and all(
        math.hypot(g[0] - w[0], g[1] - w[1]) < tol for g, w in zip(got, want))


def test_a_driven_square_becomes_its_four_corners_and_a_valid_area():
    teacher = Teacher()
    teacher.start_trace("area")
    drive(teacher, [(0, 0), (8, 0), (8, 6), (0, 6), (0, 0.2)])
    result = teacher.finish(to_ll)
    ring = result["geometry"]["coordinates"][0]
    assert ring[0] == ring[-1]
    assert close(corners_m(result), [(0, 0), (8, 0), (8, 6), (0, 6)], 0.3), corners_m(result)
    # The robot's own reader accepts it once it has a name.
    feature = {"type": "Feature", "geometry": result["geometry"],
               "properties": dict(result["properties"], name="cim")}
    assert parse_site({"type": "FeatureCollection", "features": [feature]}).areas


def test_the_shape_is_placed_by_the_mean_map_odom_and_keeps_its_form():
    """The GPS wanders ±1.5 m east-west during the drive (map->odom jumps);
    the shape must stay a rectangle, shifted by the MEAN anchor (10, 20) and
    turned by its yaw (90°)."""
    teacher = Teacher()
    teacher.start_trace("area")
    drive(teacher, [(0, 0), (8, 0), (8, 6), (0, 6), (0, 0.2)],
          anchor=(10.0, 20.0, math.pi / 2),
          jitter=lambda i: (1.5 if i % 2 else -1.5, 0.0, 0.0))
    result = teacher.finish(to_ll)
    # odom (x, y) -> map (10 - y, 20 + x); the ±1.5 cancels in the mean
    # (an even number of fixes: 281).
    want = [(10, 20), (10, 28), (4, 28), (4, 20)]
    got = corners_m(result)
    assert close(got, want, 0.3) or close(got, want, 0.3 + 3.0 / 281), got
    assert result["quality"]["anchor_wander_m"] == pytest.approx(1.5, abs=0.01)


def test_a_route_keeps_its_ends_and_bends():
    teacher = Teacher()
    teacher.start_trace("route")
    drive(teacher, [(0, 0), (5, 0), (5, 4)])
    result = teacher.finish(to_ll)
    assert result["geometry"]["type"] == "LineString"
    assert close(corners_m(result), [(0, 0), (5, 0), (5, 4)], 0.3)


def test_turning_on_the_spot_adds_no_vertices():
    teacher = Teacher()
    teacher.start_trace("route")
    for i in range(72):                       # a full turn, standing still
        yaw = math.radians(i * 5.0)
        teacher.feed(at(0, 0), None, i * 0.1, (0.0, 0.0, yaw), (0.0, 0.0, 0.0))
    assert len(teacher.state()["points"]) == 1


def test_a_pinned_corner_survives_simplification():
    teacher = Teacher()
    teacher.start_trace("route")
    t = drive(teacher, [(0, 0), (4.05, 0)])
    teacher.pin_corner()
    drive(teacher, [(4.05, 0), (8, 0)], t0=t)
    got = corners_m(teacher.finish(to_ll))
    # A straight line, but the corner stays: the first pose after the pin.
    assert len(got) == 3 and got[1] == pytest.approx((4.05, 0.0), abs=1e-6)


def test_a_trace_without_tf_skips_the_fixes_and_says_so():
    teacher = Teacher()
    teacher.start_trace("route")
    teacher.feed(at(0, 0), None, 0.0, None, (0.0, 0.0, 0.0))
    teacher.feed(at(0, 0), None, 0.1, (0.0, 0.0, 0.0), None)
    assert teacher.state()["no_tf"] == 2 and teacher.state()["points"] == []


def test_navsat_not_ready_refuses_and_keeps_recording():
    teacher = Teacher()
    teacher.start_trace("route")
    drive(teacher, [(0, 0), (5, 0)])
    with pytest.raises(TeachError, match="navsat_transform"):
        teacher.finish(lambda points: None)
    assert teacher.state()["mode"] == "trace"
    assert teacher.finish(to_ll)["geometry"]["type"] == "LineString"


@pytest.mark.parametrize("waypoints, message", [
    ([(0, 0), (6, 6), (6, 0), (0, 6), (0, 0.5)], "crosses itself"),
    ([(0, 0), (0.8, 0), (0.8, 0.8), (0, 0.8), (0, 0.1)], "only"),
    ([(0, 0), (5, 0)], "three corners"),
])
def test_a_bad_area_trace_is_refused(waypoints, message):
    teacher = Teacher()
    teacher.start_trace("area")
    drive(teacher, waypoints)
    with pytest.raises(TeachError, match=message):
        teacher.finish(to_ll)
    # Refused, but still recording: drive on or cancel.
    assert teacher.state()["mode"] == "trace"


def test_a_route_needs_two_points():
    teacher = Teacher()
    teacher.start_trace("route")
    teacher.feed(at(0, 0), None, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(TeachError, match="two points"):
        teacher.finish(to_ll)


# --- points ----------------------------------------------------------------------

def test_a_station_is_the_average_position_and_heading():
    teacher = Teacher()
    teacher.start_point("station", 1.95)
    # ±0.5 m east-west jitter about (3, 4); heading either side of north.
    for i in range(21):
        east = 3 + (0.5 if i % 2 else -0.5)
        teacher.feed(at(east, 4), 358.0 if i % 2 else 4.0, i * 0.1)
    assert teacher.state()["mode"] is None and teacher.state()["result_ready"]
    result = teacher.take()
    assert teacher.take() is None, "a result is taken once"
    x, y = metres(result["geometry"]["coordinates"])
    assert x == pytest.approx(3 - 0.5 / 21, abs=1e-4) and y == pytest.approx(4, abs=1e-4)
    mean = (10 * 358.0 + 11 * 4.0 - 10 * 360.0) / 21     # small-angle, hand
    assert result["properties"] == {"kind": "station",
                                    "heading_deg": pytest.approx(mean % 360, abs=0.05)}
    assert result["quality"]["spread_m"] == pytest.approx(0.5, abs=0.01)


def test_a_point_waits_for_both_time_and_enough_fixes():
    teacher = Teacher(min_fixes=5)
    teacher.start_point("poi", 1.0)
    teacher.feed(at(0, 0), None, 0.0)
    teacher.feed(at(0, 0), None, 5.0)          # time is up, only 2 fixes
    assert teacher.state()["averaging"]["progress"] == 1.0
    for i in range(3):
        teacher.feed(at(0, 0), None, 5.1 + i)
    result = teacher.take()
    assert result["properties"] == {"kind": "poi"}
    assert result["quality"]["fixes"] == 5


def test_a_station_without_a_heading_is_an_error_result():
    teacher = Teacher(min_fixes=2)
    teacher.start_point("station", 1.0)
    for i in range(12):
        teacher.feed(at(0, 0), None, i * 0.1)
    assert "heading" in teacher.take()["error"]


def test_one_recording_at_a_time_and_cancel_frees_it():
    teacher = Teacher()
    teacher.start_trace("area")
    with pytest.raises(TeachError, match="already recording"):
        teacher.start_point("poi", 5)
    teacher.cancel()
    teacher.start_point("poi", 5)
    assert teacher.state()["kind"] == "poi"


@pytest.mark.parametrize("call", [
    lambda t: t.start_trace("station"),
    lambda t: t.start_point("area", 5),
    lambda t: t.start_point("poi", 0.5),
    lambda t: t.start_point("poi", float("nan")),
    lambda t: t.start_point("poi", True),
    lambda t: t.pin_corner(),
    lambda t: t.finish(to_ll),
])
def test_nonsense_is_refused(call):
    with pytest.raises(TeachError):
        call(Teacher())


def test_fixes_while_idle_are_ignored():
    teacher = Teacher()
    teacher.feed(at(0, 0), 0.0, 0.0)
    assert teacher.state() == {"mode": None, "kind": None, "result_ready": False}


def test_a_heading_just_short_of_north_rounds_to_zero_not_360():
    teacher = Teacher(min_fixes=2)
    teacher.start_point("station", 1.0)
    for i in range(12):
        teacher.feed(at(0, 0), 359.97, i * 0.1)
    heading = teacher.take()["properties"]["heading_deg"]
    assert heading == 0.0
    station = {"type": "Feature", "geometry": {"type": "Point", "coordinates": [LON0, LAT0]},
               "properties": {"kind": "station", "name": "home", "heading_deg": heading}}
    assert parse_site({"type": "FeatureCollection", "features": [station]}).stations


def test_finish_does_not_hold_the_lock_while_converting():
    """web_ui's to_ll is a service call whose answer arrives on the thread
    that also feeds fixes. Holding the lock across it deadlocked."""
    import threading

    teacher = Teacher()
    teacher.start_trace("route")
    drive(teacher, [(0, 0), (5, 0)])

    def to_ll_while_fixes_arrive(points):
        fed = threading.Thread(target=teacher.state)       # takes the lock
        fed.start()
        fed.join(timeout=2.0)
        assert not fed.is_alive(), "the lock was held across to_ll"
        return to_ll(points)

    assert teacher.finish(to_ll_while_fixes_arrive)["geometry"]["type"] == "LineString"


def test_a_cancel_during_conversion_wins():
    teacher = Teacher()
    teacher.start_trace("route")
    drive(teacher, [(0, 0), (5, 0)])

    def cancelling(points):
        teacher.cancel()
        return to_ll(points)

    with pytest.raises(TeachError, match="cancelled"):
        teacher.finish(cancelling)
    assert teacher.take() is None
