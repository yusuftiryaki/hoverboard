"""Site -> map geometry -> survey grid, with a stand-in converter. ROS-free.

The converter here is a flat-earth ENU around (41 N, 29 E) — a STAND-IN for
navsat_transform's /fromLLArray, which the stack test (test_site_map_stack.py)
exercises for real. What these tests pin is everything around the converter:
which positions get asked, in what order, how shapes come back together, how a
heading becomes a yaw, and that a converter that is not ready is never read as
geometry. Expected map positions are hand-computed (10 m north = 8.993216e-05
deg, 10 m east = 1.191613e-04 deg at 41 N), not derived from the converter.
"""

import math

import pytest

from robot_missions.occupancy import OCCUPIED
from robot_missions.site import parse_site
from robot_missions.site_map import (
    NotReady, StationPose, build_grid, convert_site, dock_disagreements,
    needs_conversion)

LAT0, LON0 = 41.0, 29.0
DLAT_10M = 8.993216e-05
DLON_10M = 1.191613e-04
R = 6371000.0

# robot_sim's DockStation.stop_obstacle with robot_radius 0.4:
# 0.4 + 0.15 + 0.02 ahead of the docked base_link, 0.15 across. Literal on
# purpose; test_docking_config compares the module's constants with the sim.
BODY_OFFSET, BODY_RADIUS = 0.57, 0.15


def enu(points, rotate_deg=0.0):
    """Flat-earth ENU about (LAT0, LON0), optionally rotated CCW."""
    c, s = math.cos(math.radians(rotate_deg)), math.sin(math.radians(rotate_deg))
    out = []
    for p in points:
        east = math.radians(p.lon - LON0) * R * math.cos(math.radians(LAT0))
        north = math.radians(p.lat - LAT0) * R
        out.append((c * east - s * north, s * east + c * north))
    return out


def feature(kind, name, geometry, **properties):
    return {"type": "Feature", "geometry": geometry,
            "properties": {"kind": kind, "name": name, **properties}}


def ring_at(east_m, north_m, size_m):
    lon = LON0 + DLON_10M * east_m / 10
    lat = LAT0 + DLAT_10M * north_m / 10
    dlon, dlat = DLON_10M * size_m / 10, DLAT_10M * size_m / 10
    return [[lon, lat], [lon + dlon, lat], [lon + dlon, lat + dlat], [lon, lat + dlat], [lon, lat]]


def site_of(*features):
    return parse_site({"type": "FeatureCollection", "features": list(features)})


def station(heading_deg, east_m=0.0, north_m=0.0, name="home"):
    return feature("station", name, {"type": "Point", "coordinates": [
        LON0 + DLON_10M * east_m / 10, LAT0 + DLAT_10M * north_m / 10]},
        heading_deg=heading_deg)


def test_shapes_come_back_where_the_hand_computation_puts_them():
    site = site_of(
        feature("keepout", "cicek", {"type": "Polygon", "coordinates": [ring_at(10, 20, 2)]}),
        feature("obstacle", "agac", {"type": "Point", "coordinates": [
            LON0 - DLON_10M, LAT0 + DLAT_10M]}, radius_m=0.4),
    )
    geometry = convert_site(site, enu, BODY_OFFSET, BODY_RADIUS)
    (exterior,) = geometry.keepouts[0]
    expected = [(10, 20), (12, 20), (12, 22), (10, 22)]
    assert len(exterior) == 4
    for (x, y), (ex, ey) in zip(exterior, expected):
        assert (x, y) == (pytest.approx(ex, abs=1e-3), pytest.approx(ey, abs=1e-3))
    (tree,) = geometry.circles
    assert (tree.x, tree.y, tree.radius) == (
        pytest.approx(-10, abs=1e-3), pytest.approx(10, abs=1e-3), 0.4)


@pytest.mark.parametrize("heading_deg, yaw_deg", [
    (0.0, 90.0), (45.0, 45.0), (90.0, 0.0), (135.0, -45.0),
    (180.0, -90.0), (225.0, -135.0), (270.0, 180.0), (315.0, 135.0),
])
def test_a_compass_heading_becomes_a_rep103_yaw_all_the_way_round(heading_deg, yaw_deg):
    # Compass: 0 = north, clockwise. REP-103: 0 = +x (east), counter-clockwise.
    # yaw = 90 - bearing. Swept, not spot-checked (handoff decision 11): a
    # mirrored or 90-degree-off mapping is exact at one heading.
    geometry = convert_site(site_of(station(heading_deg)), enu, BODY_OFFSET, BODY_RADIUS)
    (pose,) = geometry.stations
    turn = math.atan2(math.sin(pose.yaw - math.radians(yaw_deg)),
                      math.cos(pose.yaw - math.radians(yaw_deg)))
    assert abs(math.degrees(turn)) < 0.01
    (body,) = geometry.station_bodies
    assert body.x == pytest.approx(BODY_OFFSET * math.cos(math.radians(yaw_deg)), abs=1e-3)
    assert body.y == pytest.approx(BODY_OFFSET * math.sin(math.radians(yaw_deg)), abs=1e-3)
    assert body.radius == BODY_RADIUS


def test_the_heading_goes_through_the_converter_not_around_it():
    # A map frame rotated 10 deg CCW against ENU must rotate the station's yaw
    # by the same 10 deg. If the yaw were computed from the bearing alone, the
    # docked pose would point 10 deg off in a rotated map.
    geometry = convert_site(site_of(station(90.0)),
                            lambda pts: enu(pts, rotate_deg=10.0), BODY_OFFSET, BODY_RADIUS)
    assert math.degrees(geometry.stations[0].yaw) == pytest.approx(10.0, abs=0.01)


@pytest.mark.parametrize("answer", [[], [(0.0, 0.0)]])
def test_a_short_answer_is_not_ready_never_geometry(answer):
    # navsat_transform's /fromLLArray answers an EMPTY list until its transform
    # exists (docs/missions.md decision 5). Read as geometry that would be a
    # map with nothing on it.
    site = site_of(station(0.0), station(90.0, east_m=5, name="yedek"))
    with pytest.raises(NotReady):
        convert_site(site, lambda pts: answer, BODY_OFFSET, BODY_RADIUS)


def test_a_non_finite_answer_is_not_ready():
    with pytest.raises(NotReady):
        convert_site(site_of(station(0.0)), lambda pts: [(math.nan, 0.0)] * len(pts),
                     BODY_OFFSET, BODY_RADIUS)


def test_areas_and_routes_alone_need_no_conversion_and_draw_nothing():
    site = site_of(
        feature("area", "bahce", {"type": "Polygon", "coordinates": [ring_at(0, 0, 10)]}),
        feature("route", "tur", {"type": "LineString", "coordinates": [
            [LON0, LAT0], [LON0 + DLON_10M, LAT0]]}))
    assert not needs_conversion(site)

    def never(points):
        raise AssertionError("the converter must not be called")

    geometry = convert_site(site, never, BODY_OFFSET, BODY_RADIUS)
    spec, grid = build_grid(geometry, 0.1, 30.0, 10**7)
    assert set(grid) == {0}
    assert (spec.width, spec.height) == (600, 600)


def test_the_survey_grid_carries_keepout_obstacle_and_station():
    site = site_of(
        feature("keepout", "cicek", {"type": "Polygon", "coordinates": [ring_at(10, 0, 2)]}),
        feature("obstacle", "agac", {"type": "Point", "coordinates": [
            LON0, LAT0 + DLAT_10M]}, radius_m=0.5),
        station(90.0),                    # facing east, body centred on (0.57, 0)
    )
    assert needs_conversion(site)
    spec, grid = build_grid(convert_site(site, enu, BODY_OFFSET, BODY_RADIUS), 0.1, 5.0, 10**7)

    def at(x, y):
        return grid[spec.index(*spec.cell_of(x, y))]

    assert at(11.0, 1.0) == OCCUPIED        # inside the keep-out
    assert at(9.5, 1.0) != OCCUPIED         # half a metre west of it
    assert at(0.0, 10.0) == OCCUPIED        # the tree's centre
    assert at(0.0, 10.7) != OCCUPIED        # 0.7 m north of a 0.5 m tree
    assert at(0.57, 0.0) == OCCUPIED        # the station body
    assert at(0.0, 0.0) != OCCUPIED         # the docked robot's own spot is free
    # Bounds by hand, margin 5 m. x: tree's west edge -0.5 -> -5.5; keep-out's
    # east edge 12 -> 17; width (17 + 5.5) / 0.1 = 225. y: station body's
    # south edge -0.15 -> -5.15, snapped down to -5.2; tree's north edge
    # 10.5 -> 15.5; height (15.5 + 5.2) / 0.1 = 207. (The keep-out's corner
    # comes back a hair past 12 m — the literal degrees are rounded — so its
    # side may round up one cell: 225 or 226.)
    assert spec.origin_x == pytest.approx(-5.5, abs=1e-6)
    assert spec.origin_y == pytest.approx(-5.2, abs=1e-6)
    assert spec.width in (225, 226) and spec.height == 207


def test_matching_dock_poses_raise_nothing():
    stations = [StationPose("home", 0.1, -0.2, math.radians(3))]
    assert dock_disagreements(stations, {"home": [0.0, 0.0, 0.0]}, 1.0, math.radians(15)) == []


@pytest.mark.parametrize("station_yaw_deg, pose, words", [
    (0.0, [1.5, 0.0, 0.0], "1.50 m"),
    (0.0, [0.0, 0.0, math.radians(30)], "30°"),
    (20.0, [0.0, 0.0, math.radians(-150)], "170°"),     # the short way round
])
def test_dock_poses_that_part_are_reported(station_yaw_deg, pose, words):
    stations = [StationPose("home", 0.0, 0.0, math.radians(station_yaw_deg))]
    (problem,) = dock_disagreements(stations, {"home": pose}, 1.0, math.radians(15))
    assert words in problem


def test_yaw_wraps_when_comparing_dock_poses():
    # 179 vs -179 deg are 2 deg apart, not 358.
    stations = [StationPose("home", 0.0, 0.0, math.radians(179))]
    assert dock_disagreements(stations, {"home": [0.0, 0.0, math.radians(-179)]},
                              1.0, math.radians(15)) == []


def test_a_station_docks_yaml_does_not_know_is_reported():
    (problem,) = dock_disagreements([StationPose("garaj", 0, 0, 0)],
                                    {"home": [0.0, 0.0, 0.0]}, 1.0, math.radians(15))
    assert "garaj" in problem
