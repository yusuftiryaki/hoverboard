"""A keep-out zone drawn in lat/lon, avoided by Nav2 on the real `map`.

    site.geojson ─► site_map_server ─/fromLLArray─► /obstacle_map ─► costmaps ─► NavFn/RPP
                                     navsat_transform ◄── GPS, madgwick, ekf_global

The whole chain the missions will stand on, end to end, measured on ground
truth: the robot must go AROUND a zone that exists only as lat/lon in a file —
nothing in the world stops it, a keep-out is a flower bed, not a wall. So the
only thing that can make it detour is the map, and the contrast test (same
goal, no zone) is what tells a detour from RPP wandering (A6: a measurement
that ran in one configuration proves nothing).

The GPS wander is off and its noise tiny, as in test_site_map_stack.py: with
the NEO-6M's 2-5 m wander on, `map` and the robot's estimate in it move
together (common-mode for navigation) but the TRUE path would then sit metres
from the zone it avoided in map — that is a mission-quality number for
docs/missions.md phase 3, not a pass/fail for the wiring.

Spawns two GPS + Nav2 stacks, ~3 min. Part of the `nav2` CI stage.
"""

import math

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from robot_sim.sim_node import EARTH_RADIUS_M           # noqa: E402

from test_nav2 import ARRIVAL_TOLERANCE, Navigator      # noqa: E402

DATUM_LAT, DATUM_LON = 41.0, 29.0
QUIET_GPS = ("-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")
# A 1 m deep, 6 m wide band across the straight line to the goal.
ZONE = (3.5, -3.0, 4.5, 3.0)                    # x0, y0, x1, y1 in sim_world
GOAL = (8.0, 0.0)
ROBOT_RADIUS = 0.4                               # nav2.yaml
# Going round a 6 m band means passing |y| >= 3 + the robot's radius.
DETOUR_Y = 3.0
STRAIGHT_LINE_Y = 0.5


def lonlat(east, north):
    """sim_world metres -> [lon, lat], exactly as sim_node's GPS does it."""
    return [DATUM_LON + math.degrees(
                east / (EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)))),
            DATUM_LAT + math.degrees(north / EARTH_RADIUS_M)]


def zone_feature():
    x0, y0, x1, y1 = ZONE
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    return {"type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [[lonlat(*c) for c in corners]]},
            "properties": {"kind": "keepout", "name": "cicek"}}


def distance_to_zone(x, y):
    x0, y0, x1, y1 = ZONE
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


@pytest.fixture
def navigator(ros):
    node = Navigator()
    yield node
    node.destroy_node()


def drive_to_goal(navigator):
    navigator.spin(3.0)
    assert navigator.truth is not None, "no /ground_truth — is sim_node up?"
    status = navigator.go(*GOAL, timeout=240.0)
    navigator.spin(1.0)
    x, y = navigator.position
    assert status == 4, f"Nav2 reported status {status} (4 = SUCCEEDED)"
    error = math.hypot(x - GOAL[0], y - GOAL[1])
    assert error < ARRIVAL_TOLERANCE, (
        f"Nav2 says it arrived, but ground truth is {error:.2f} m from the goal")
    return navigator.path


def test_nav2_goes_around_a_keepout_drawn_in_lat_lon(site_nav2_stack, navigator):
    site_nav2_stack([zone_feature()], *QUIET_GPS)
    path = drive_to_goal(navigator)

    detour = max(abs(py) for _, py in path)
    clearance = min(distance_to_zone(px, py) for px, py in path)
    print(f"keep-out: detour max|y| = {detour:.2f} m, closest body-centre "
          f"approach {clearance:.2f} m (robot radius {ROBOT_RADIUS})")
    assert detour >= DETOUR_Y, (
        f"the robot never went round the zone (max |y| = {detour:.2f} m) — did "
        "/obstacle_map arrive from site_map_server?")
    # The zone has no physics: entering it is not stopped by anything, so this
    # is the real assertion. The robot's body (radius 0.4) must stay out.
    assert clearance >= ROBOT_RADIUS - 0.05, (
        f"the robot's centre came {clearance:.2f} m from the keep-out — its "
        f"{ROBOT_RADIUS} m body went inside")


def test_without_the_keepout_the_same_goal_is_a_straight_line(site_nav2_stack, navigator):
    site_nav2_stack([], *QUIET_GPS)
    path = drive_to_goal(navigator)
    deviation = max(abs(py) for _, py in path)
    assert deviation < STRAIGHT_LINE_Y, (
        f"with nothing in the way the robot still swung {deviation:.2f} m off "
        "the line — the keep-out test cannot tell avoidance from wandering")
