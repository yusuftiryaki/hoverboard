"""The planners, against hand-worked geometry. ROS-free.

Waypoint lists for the simple shapes are written out by hand below; the
coverage-fraction checks use shapely only to MEASURE a plan (area of the swept
strips), never to produce the expected answer.
"""

import math

import numpy as np
import pytest
import shapely
from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Point, Polygon

from robot_missions.planner import (
    PlanError, auto_angle, coverage, free_region, goto, patrol_perimeter,
    patrol_random, route, row_offsets)

RECT = Polygon([(0, 0), (10, 0), (10, 4), (0, 4)])


def xyw(plan):
    return [(round(w.x, 6), round(w.y, 6), w.work) for w in plan.waypoints]


def fraction(plan, width, region, cell=0.1):
    """The share of `region` within width/2 of some WORK leg — sampled on a
    `cell` grid, so it measures the plan without asking GEOS to union dozens
    of strips that share edges (that union dropped whole strips to float
    noise when tried: 82.7 of 112.3 m^2 for a plan that covers all of it)."""
    legs = MultiLineString([[(a.x, a.y), (b.x, b.y)]
                            for a, b in zip(plan.waypoints, plan.waypoints[1:])
                            if b.work and math.hypot(b.x - a.x, b.y - a.y) > 1e-6])
    min_x, min_y, max_x, max_y = region.bounds
    xs, ys = np.meshgrid(np.arange(min_x + cell / 2, max_x, cell),
                         np.arange(min_y + cell / 2, max_y, cell))
    samples = shapely.points(xs.ravel(), ys.ravel())
    inside = samples[shapely.contains(region, samples)]
    return float(np.mean(shapely.distance(inside, legs) <= width / 2 + 1e-9))


# --- free region ----------------------------------------------------------------

def test_the_margin_keeps_rectangle_corners_square():
    # 10 x 4 shrunk by 0.5 with mitred joins: exactly 9 x 3 = 27 m^2, corners
    # at (0.5, 0.5) and (9.5, 3.5). Round joins would give 27 - (1 - pi/4)*... no:
    # an INWARD buffer keeps corners square either way; this pins the size.
    region = free_region(RECT, [], 0.5, 0.0)
    assert region.area == pytest.approx(27.0)
    assert region.bounds == pytest.approx((0.5, 0.5, 9.5, 3.5))


def test_obstacles_grow_by_the_clearance_and_are_cut_out():
    # A 0.5 m tree at (5, 2), clearance 0.5: a 1 m circle removed (pi m^2,
    # less a hair for the buffer's polygonal circle).
    region = free_region(RECT, [Point(5, 2).buffer(0.5)], 0.0, 0.5)
    assert region.area == pytest.approx(40.0 - math.pi, abs=0.05)
    assert not region.contains(Point(5, 2.9))       # 0.9 m from the centre
    assert region.contains(Point(5, 3.1))           # 1.1 m from the centre


def test_an_area_the_margin_eats_entirely_is_an_error():
    with pytest.raises(PlanError, match="nothing of the area"):
        free_region(RECT, [], 2.5, 0.0)


def test_a_self_intersecting_area_is_an_error():
    bowtie = Polygon([(0, 0), (4, 4), (4, 0), (0, 4)])
    with pytest.raises(PlanError, match="intersects itself"):
        free_region(bowtie, [], 0.0, 0.0)


# --- rows -------------------------------------------------------------------------

@pytest.mark.parametrize("low, high, spacing, expected", [
    (0.0, 5.0, 1.0, [0.5, 1.5, 2.5, 3.5, 4.5]),
    (0.0, 4.6, 1.0, [0.3, 1.3, 2.3, 3.3, 4.3]),
    (0.0, 0.4, 1.0, [0.2]),                     # narrower than one row: the middle
    (2.0, 3.0, 0.25, [2.125, 2.375, 2.625, 2.875]),
])
def test_rows_are_centred_with_equal_edge_gaps(low, high, spacing, expected):
    assert row_offsets(low, high, spacing) == pytest.approx(expected)


def test_a_rectangle_is_the_classic_serpentine():
    # 10 x 4, rows 1 m apart along x: y = 0.5, 1.5, 2.5, 3.5, alternating.
    plan = coverage(free_region(RECT, [], 0, 0), 1.0, angle=0.0, perimeter_pass=False)
    assert xyw(plan) == [
        (0, 0.5, False), (10, 0.5, True),
        (10, 1.5, False), (0, 1.5, True),
        (0, 2.5, False), (10, 2.5, True),
        (10, 3.5, False), (0, 3.5, True),
    ]
    # Each pose faces along its row; the transit pose already faces the way
    # the row will be driven, so the robot turns once, at the row's start.
    assert [round(w.yaw, 6) for w in plan.waypoints] == pytest.approx(
        [0, 0, math.pi, math.pi, 0, 0, math.pi, math.pi])
    assert plan.info["rows"] == 4 and plan.info["row_length_m"] == pytest.approx(40.0)
    assert plan.precise                 # rows need the precise Nav2 profile


def test_start_decides_the_corner_the_serpentine_begins_in():
    plan = coverage(free_region(RECT, [], 0, 0), 1.0, angle=0.0, perimeter_pass=False,
                    start=(10.5, 4.5))
    assert xyw(plan)[:2] == [(10, 3.5, False), (0, 3.5, True)]
    assert xyw(plan)[-1] == (10, 0.5, True)


def test_rows_follow_the_angle_given():
    # The same rectangle, rows along y (90 deg): 10 rows at x = 0.5 .. 9.5.
    plan = coverage(free_region(RECT, [], 0, 0), 1.0, angle=math.pi / 2, perimeter_pass=False)
    xs = sorted({round(w.x, 6) for w in plan.waypoints})
    assert xs == pytest.approx([0.5 + k for k in range(10)])
    for a, b in zip(plan.waypoints[0::2], plan.waypoints[1::2]):
        assert a.x == pytest.approx(b.x) and {round(a.y, 6), round(b.y, 6)} == {0, 4}


def test_auto_angle_runs_rows_along_the_long_side():
    assert auto_angle(RECT) == pytest.approx(0.0)
    tall = Polygon([(0, 0), (4, 0), (4, 10), (0, 10)])
    assert abs(auto_angle(tall)) == pytest.approx(math.pi / 2)
    for degrees in (-60, -30, 15, 30, 75):
        tilted = affinity.rotate(Polygon([(0, 0), (20, 0), (20, 5), (0, 5)]), degrees,
                                 origin=(0, 0))
        assert math.degrees(auto_angle(tilted)) == pytest.approx(degrees, abs=1e-6)


@pytest.mark.parametrize("degrees", range(0, 180, 15))
def test_a_tilted_field_is_covered_whatever_its_tilt(degrees):
    # Swept round (decision 11): a 20 x 6 field at every tilt, auto rows,
    # spacing = width: the work legs must sweep the whole centre-line region
    # (measured: 100.0% at all twelve tilts) and stay inside it.
    field = affinity.rotate(Polygon([(0, 0), (20, 0), (20, 6), (0, 6)]), degrees,
                            origin=(3, 1))
    region = free_region(field, [], 0.15, 0.0)
    plan = coverage(region, 0.3, perimeter_pass=False)
    assert math.degrees(plan.info["angle_rad"]) % 180 == pytest.approx(degrees % 180, abs=1e-6)
    assert fraction(plan, 0.3, region) >= 0.995
    assert all(region.buffer(1e-6).contains(Point(w.x, w.y)) for w in plan.waypoints)


def test_a_tree_splits_rows_and_every_leg_avoids_it():
    # 10 x 10 lawn, a 0.5 m tree at (5, 5) grown by 0.5: rows crossing it split
    # in two. No WORK leg may pass within 1 m of the tree's centre.
    region = free_region(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]),
                         [Point(5, 5).buffer(0.5)], 0.0, 0.5)
    plan = coverage(region, 0.5, angle=0.0, perimeter_pass=False)
    assert plan.info["segments"] > plan.info["rows"]
    tree = Point(5, 5)
    for a, b in zip(plan.waypoints, plan.waypoints[1:]):
        if b.work:
            # 1 mm slack: the grown circle is a 64-gon, its flats sit inside
            # the true circle by r(1 - cos(pi/64)) = 1.2 mm.
            assert LineString([(a.x, a.y), (b.x, b.y)]).distance(tree) >= 1.0 - 2e-3
    assert fraction(plan, 0.5, region) >= 0.95


def test_the_perimeter_pass_trims_what_slanted_rows_leave():
    # A triangular lawn, centre lines 0.15 m inside it, a 0.3 m cut. Measured
    # on the LAWN (what gets cut), not on the centre-line region: rows end on
    # the slanted edge in a saw-tooth that leaves uncut triangles. Measured:
    # 95.7% with rows only, 99.9% with the pass.
    triangle = Polygon([(0, 0), (12, 0), (0, 8)])
    region = free_region(triangle, [], 0.15, 0.0)
    rows_only = coverage(region, 0.3, angle=0.0, perimeter_pass=False)
    with_edge = coverage(region, 0.3, angle=0.0, perimeter_pass=True)
    assert fraction(rows_only, 0.3, triangle) < 0.97
    assert fraction(with_edge, 0.3, triangle) >= 0.99
    # The edge pass begins with a transit, then is all work.
    tail = with_edge.waypoints[len(rows_only.waypoints):]
    assert tail[0].work is False and all(w.work for w in tail[1:])


def test_the_perimeter_pass_goes_round_holes_too():
    region = free_region(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]),
                         [Point(5, 5).buffer(0.5)], 0.0, 0.5)
    plan = coverage(region, 0.5, angle=0.0)
    # Some pose of the pass lies on the ring 1 m round the tree.
    assert any(abs(math.hypot(w.x - 5, w.y - 5) - 1.0) < 0.12 for w in plan.waypoints)
    # Circles are simplified before sampling: a handful of poses, not hundreds.
    assert len(plan.waypoints) < 120


def test_no_row_long_enough_is_an_error():
    sliver = free_region(Polygon([(0, 0), (0.2, 0), (0.2, 5), (0, 5)]), [], 0, 0)
    with pytest.raises(PlanError, match="no row"):
        coverage(sliver, 0.3, angle=0.0)


# --- patrol -----------------------------------------------------------------------

def test_perimeter_patrol_by_hand():
    # 10 x 4, step 5: corners plus the midpoints of the 10 m sides, counter-
    # clockwise from the corner nearest (9, 3) = (10, 4), closed.
    plan = patrol_perimeter(free_region(RECT, [], 0, 0), step=5.0, laps=3, start=(9, 3))
    assert [(round(w.x, 6), round(w.y, 6)) for w in plan.waypoints] == [
        (10, 4), (5, 4), (0, 4), (0, 0), (5, 0), (10, 0), (10, 4)]
    assert plan.waypoints[0].work is False and all(w.work for w in plan.waypoints[1:])
    assert plan.repeat == 3
    assert plan.info["lap_m"] == pytest.approx(28.0)
    assert not plan.precise


def test_random_patrol_is_inside_spread_and_repeatable():
    region = free_region(RECT, [Point(5, 2).buffer(0.5)], 0.2, 0.5)
    a = patrol_random(region, points=12, seed=7, min_separation=3.0, start=(0, 0))
    b = patrol_random(region, points=12, seed=7, min_separation=3.0, start=(0, 0))
    c = patrol_random(region, points=12, seed=8, min_separation=3.0, start=(0, 0))
    assert a.waypoints == b.waypoints and a.waypoints != c.waypoints
    assert len(a.waypoints) == 12
    previous = (0, 0)
    for w in a.waypoints:
        assert region.contains(Point(w.x, w.y))
        assert math.hypot(w.x - previous[0], w.y - previous[1]) >= 3.0
        previous = (w.x, w.y)


def test_an_impossible_separation_is_an_error():
    with pytest.raises(PlanError, match="min_separation"):
        patrol_random(free_region(RECT, [], 0, 0), points=3, min_separation=50.0)


# --- route / goto -----------------------------------------------------------------

def test_route_and_back_and_forth():
    plan = route([(0, 0), (4, 0), (4, 3)], repeat=2, back_and_forth=True)
    assert [(w.x, w.y) for w in plan.waypoints] == [(0, 0), (4, 0), (4, 3), (4, 0), (0, 0)]
    assert plan.waypoints[0].work is False
    assert plan.info["length_m"] == pytest.approx(14.0)
    assert plan.repeat == 2
    # Each pose faces the next: east, north, then back south, then west.
    assert [round(math.degrees(w.yaw)) for w in plan.waypoints] == [0, 90, -90, 180, 180]


def test_goto_faces_the_way_it_came_unless_told():
    assert goto((3, 4), start=(0, 0)).waypoints[0].yaw == pytest.approx(math.atan2(4, 3))
    assert goto((3, 4), yaw=1.0, start=(0, 0)).waypoints[0].yaw == 1.0


def test_bad_counts_are_errors():
    with pytest.raises(PlanError):
        route([(0, 0)])
    with pytest.raises(PlanError):
        route([(0, 0), (1, 1)], repeat=-1)
    with pytest.raises(PlanError):
        patrol_perimeter(free_region(RECT, [], 0, 0), laps=-1)
