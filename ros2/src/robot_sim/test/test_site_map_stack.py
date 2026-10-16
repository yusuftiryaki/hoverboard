"""The survey map through the REAL chain: site.geojson -> site_map_server ->
navsat_transform's /fromLLArray -> /obstacle_map. Skips without ROS, ~40 s.

robot_missions' own tests prove the geometry with a stand-in converter. This
proves the stand-in was honest: that the lat/lon written into a site file lands
in `map` where the robot's own GPS says that spot is. A sign, an axis or a
lat/lon swap would put the obstacles somewhere else and every costmap would
still look perfectly healthy (A6's lesson, in a new place).

Ground truth: the simulator DEFINES where a lat/lon is — its fake GPS turns
sim_world metres into lat/lon with a flat earth about its datum (sim_node
_publish_gps). The site's positions are written with that same definition, so
"right" means "the obstacle's cells sit on its sim_world position".

The GPS's wander is switched OFF here and its white noise made tiny. With them
on, `map` is anchored on the first fix and carries its error (5.45 m measured
once, ekf.yaml), which is common-mode and cancels for navigation but would
swamp the millimetres this test is about. What remains is navsat_transform's
real UTM projection against the sim's flat earth, plus the fused heading at the
moment the transform was made. Swept round the compass (decision 11): a mirror
or a quarter turn is exact in some direction.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray         # noqa: E402
from nav_msgs.msg import OccupancyGrid                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402

from robot_missions.occupancy import OCCUPIED           # noqa: E402
from robot_missions.site_map_server import DIAG_NAME    # noqa: E402
from robot_sim.sim_node import EARTH_RADIUS_M           # noqa: E402

from conftest import wait_for_message                   # noqa: E402

DATUM_LAT, DATUM_LON = 41.0, 29.0          # sim_node's defaults
RING_M = 12.0
TREE_RADIUS = 0.5
QUIET_GPS = ("-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02")
# navsat_transform's `delay` (3 s) + EKF + madgwick settling, generously.
MAP_TIMEOUT_S = 60.0


def lonlat(east, north):
    """sim_world metres -> [lon, lat], exactly as sim_node's GPS does it."""
    return [DATUM_LON + math.degrees(
                east / (EARTH_RADIUS_M * math.cos(math.radians(DATUM_LAT)))),
            DATUM_LAT + math.degrees(north / EARTH_RADIUS_M)]


def tree(name, east, north):
    return {"type": "Feature",
            "geometry": {"type": "Point", "coordinates": lonlat(east, north)},
            "properties": {"kind": "obstacle", "name": name, "radius_m": TREE_RADIUS}}


def keepout(name, corners):
    ring = [lonlat(*c) for c in corners] + [lonlat(*corners[0])]
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": "keepout", "name": name}}


def station_facing_east():
    # docks.yaml's placeholder is the map origin facing east; this matches it,
    # so the cross-check must stay quiet.
    return {"type": "Feature", "geometry": {"type": "Point", "coordinates": lonlat(0, 0)},
            "properties": {"kind": "station", "name": "home", "heading_deg": 90.0}}


BEARINGS = range(0, 360, 45)


def ring_of_trees():
    return [tree(f"agac{b}", RING_M * math.sin(math.radians(b)),
                 RING_M * math.cos(math.radians(b))) for b in BEARINGS]


def centroid_near(message, x, y, window):
    info = message.info
    sx = sy = n = 0
    for j in range(int((y - window - info.origin.position.y) / info.resolution),
                   int((y + window - info.origin.position.y) / info.resolution) + 1):
        for i in range(int((x - window - info.origin.position.x) / info.resolution),
                       int((x + window - info.origin.position.x) / info.resolution) + 1):
            if message.data[j * info.width + i] == OCCUPIED:
                sx += info.origin.position.x + (i + 0.5) * info.resolution
                sy += info.origin.position.y + (j + 0.5) * info.resolution
                n += 1
    return (sx / n, sy / n, n) if n else None


def cell(message, x, y):
    info = message.info
    i = int(math.floor((x - info.origin.position.x) / info.resolution))
    j = int(math.floor((y - info.origin.position.y) / info.resolution))
    return message.data[j * info.width + i]


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def test_site_positions_land_where_the_gps_puts_them_all_the_way_round(site_stack):
    # Inside the ring, clear of every tree's 2 m search window.
    square = [(3.0, -6.0), (6.0, -6.0), (6.0, -3.0), (3.0, -3.0)]
    site_stack([*ring_of_trees(), keepout("cicek", square), station_facing_east()],
               *QUIET_GPS)
    message = wait_for_message(rclpy, "/obstacle_map", OccupancyGrid, MAP_TIMEOUT_S,
                               qos=LATCHED)
    assert message is not None, (
        "no /obstacle_map: site_map_server never got a full answer from "
        "/fromLLArray (is navsat_transform running and fed? see its log)")
    assert message.header.frame_id == "map"

    errors = []
    for b in BEARINGS:
        x, y = RING_M * math.sin(math.radians(b)), RING_M * math.cos(math.radians(b))
        found = centroid_near(message, x, y, 2.0)
        assert found is not None, f"the tree at bearing {b} is not on the map near ({x}, {y})"
        cx, cy, n = found
        # pi r^2 / cell^2 = 78.5 cells; the centre-test rim may differ by a ring.
        assert 60 <= n <= 100, f"bearing {b}: {n} cells for a 0.5 m tree"
        errors.append((b, math.hypot(cx - x, cy - y)))
    print("site -> map error per bearing (m):",
          ", ".join(f"{b}: {e:.3f}" for b, e in errors))
    worst = max(e for _, e in errors)
    # MEASURED, 4 runs: worst 0.065 / 0.035 / 0.074 / 0.061 m. The bound has to
    # sit BELOW what the one real bug here produced: with sim_node reporting
    # base_link instead of the antenna (0.15 m behind it), the same sweep read
    # 0.105-0.182 m, worst 0.182 — a 0.25 m bound passed it.
    assert worst < 0.12, f"a site position landed {worst:.3f} m from its sim_world spot"

    # The keep-out: well inside is occupied, 0.6 m outside each side is not.
    assert cell(message, 4.5, -4.5) == OCCUPIED
    for x, y in ((2.4, -4.5), (6.6, -4.5), (4.5, -6.6), (4.5, -2.4)):
        assert cell(message, x, y) != OCCUPIED, f"keep-out spills to ({x}, {y})"

    # The station body is 0.57 m ahead (east) of the docked pose at the origin.
    body = centroid_near(message, 0.57, 0.0, 0.5)
    assert body is not None and math.hypot(body[0] - 0.57, body[1]) < 0.1


def test_a_matching_station_raises_no_dock_error(site_stack):
    site_stack([station_facing_east()], *QUIET_GPS)
    assert wait_for_message(rclpy, "/obstacle_map", OccupancyGrid, MAP_TIMEOUT_S,
                            qos=LATCHED) is not None
    # /diagnostics is shared by every node; collect for a while, keep ours.
    listener = rclpy.create_node("site_diag_listener")
    seen = []
    listener.create_subscription(
        DiagnosticArray, "/diagnostics",
        lambda m: seen.extend(s for s in m.status if s.name == DIAG_NAME), 50)
    try:
        deadline = time.monotonic() + 10.0
        while not seen and time.monotonic() < deadline:
            rclpy.spin_once(listener, timeout_sec=0.2)
    finally:
        listener.destroy_node()
    assert seen, "site_map_server published no diagnostics"
    assert all(s.level == b"\x00" for s in seen), [s.message for s in seen]
