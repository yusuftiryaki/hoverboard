"""Site (lat/lon) -> geometry in `map` metres -> the survey OccupancyGrid. ROS-free.

The conversion itself is NOT here: it is a callable handed in, and on the robot
that callable is navsat_transform's /fromLLArray — the same transform that puts
the robot's own GPS fixes into `map` (docs/missions.md decision 4). This module
only decides WHICH positions to convert, in one batch, and rebuilds the shapes
from the answers. That keeps it testable against hand-computed numbers with no
ROS anywhere, and it keeps every lat/lon on the robot going through one model.

Why the shared transform matters rather than "any good projection": the site's
positions and the robot's GPS positions then carry the same datum error, which
cancels (handoff decision 12). And Istanbul's UTM grid convergence is ~1.3°,
1.1 m across 50 m — a second, independent conversion that merely got that one
term differently would be off by more than a cutting swath.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence, Tuple

from robot_missions.occupancy import (
    Circle, Point, covering_grid, empty_grid, mark_circle, mark_polygon)
from robot_missions.site import LatLon, Site, offset

# lat/lon positions in, `map` (x, y) out, same length and order. Anything else
# is a converter that is not ready — see convert_site.
Converter = Callable[[Sequence[LatLon]], Sequence[Point]]

# How far along a station's heading the second, direction-carrying point is
# placed. Long enough that float noise in UTM's ~1e6 m coordinates is
# irrelevant, short enough that the local spherical offset() is exact enough.
HEADING_BASELINE_M = 5.0

# The station body on the map: a circle this far ahead of the docked base_link,
# this wide. robot_sim's DockStation.stop_obstacle with nav2.yaml's
# robot_radius 0.4: 0.4 + 0.15 + 0.02 = 0.57 ahead, 0.15 across.
# test_docking_config holds the two together; SP6 changes both or neither.
STATION_BODY_OFFSET = 0.57
STATION_BODY_RADIUS = 0.15


class NotReady(RuntimeError):
    """The converter answered, but not with one point per position asked —
    navsat_transform's /fromLLArray answers an EMPTY list until its transform
    exists (decision 5), and that must never be read as geometry."""


@dataclass(frozen=True)
class StationPose:
    name: str
    x: float
    y: float
    yaw: float                  # rad, `map` frame, REP-103 (CCW from +x)


@dataclass(frozen=True)
class MapGeometry:
    """The site's obstacle-relevant shapes, in `map` metres."""

    keepouts: Tuple[Tuple[Tuple[Point, ...], ...], ...] = ()     # rings per polygon
    obstacle_polygons: Tuple[Tuple[Tuple[Point, ...], ...], ...] = ()
    circles: Tuple[Circle, ...] = ()
    stations: Tuple[StationPose, ...] = ()
    station_bodies: Tuple[Circle, ...] = ()

    def points(self) -> List[Point]:
        found: List[Point] = []
        for polygon in (*self.keepouts, *self.obstacle_polygons):
            for ring in polygon:
                found.extend(ring)
        for circle in (*self.circles, *self.station_bodies):
            found.extend(((circle.x - circle.radius, circle.y - circle.radius),
                          (circle.x + circle.radius, circle.y + circle.radius)))
        found.extend((s.x, s.y) for s in self.stations)
        return found


def needs_conversion(site: Site) -> bool:
    """False when the map can be built without navsat_transform at all."""
    return bool(site.keepouts or site.circle_obstacles or site.polygon_obstacles
                or site.stations)


def positions_to_convert(site: Site) -> List[LatLon]:
    """Every lat/lon the survey map needs, in the order assemble() reads them:
    keep-out rings, obstacle-polygon rings, circle centres, then per station
    its position and a second point HEADING_BASELINE_M along its heading.

    Areas and routes are not obstacles and are not drawn; missions convert
    those themselves when they run.
    """
    asked: List[LatLon] = []
    for polygon in (*(k.polygon for k in site.keepouts),
                    *(o.polygon for o in site.polygon_obstacles)):
        for ring in polygon.rings():
            asked.extend(ring)
    asked.extend(o.center for o in site.circle_obstacles)
    for station in site.stations:
        asked.extend((station.position,
                      offset(station.position, station.heading_deg, HEADING_BASELINE_M)))
    return asked


def assemble(site: Site, answer: Sequence[Point], body_offset: float,
             body_radius: float) -> MapGeometry:
    """Rebuild the shapes from the converted positions_to_convert(site).

    The station's body is placed `body_offset` ahead of the docked base_link
    along its heading, `body_radius` across — the same geometry robot_sim's
    DockStation.stop_obstacle uses (test_docking_config holds them together).
    """
    expected = len(positions_to_convert(site))
    if len(answer) != expected:
        raise NotReady(f"asked to convert {expected} positions, got {len(answer)} back")
    for x, y in answer:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise NotReady(f"converter returned a non-finite point ({x}, {y})")
    points = iter(answer)

    def ring(length: int) -> Tuple[Point, ...]:
        return tuple(next(points) for _ in range(length))

    def polygon(shape) -> Tuple[Tuple[Point, ...], ...]:
        return tuple(ring(len(r)) for r in shape.rings())

    keepouts = tuple(polygon(k.polygon) for k in site.keepouts)
    obstacle_polygons = tuple(polygon(o.polygon) for o in site.polygon_obstacles)
    circles = tuple(Circle(*next(points), o.radius_m) for o in site.circle_obstacles)
    stations = []
    bodies = []
    for station in site.stations:
        (x, y), (ahead_x, ahead_y) = next(points), next(points)
        yaw = math.atan2(ahead_y - y, ahead_x - x)
        stations.append(StationPose(station.name, x, y, yaw))
        bodies.append(Circle(x + body_offset * math.cos(yaw),
                             y + body_offset * math.sin(yaw), body_radius))
    return MapGeometry(keepouts=keepouts, obstacle_polygons=obstacle_polygons,
                       circles=circles, stations=tuple(stations),
                       station_bodies=tuple(bodies))


def convert_site(site: Site, convert: Converter, body_offset: float,
                 body_radius: float) -> MapGeometry:
    """positions_to_convert -> ONE converter call -> assemble."""
    asked = positions_to_convert(site)
    if not asked:
        return MapGeometry()
    return assemble(site, list(convert(asked)), body_offset, body_radius)


def build_grid(geometry: MapGeometry, resolution: float, margin: float,
               max_cells: int):
    """(GridSpec, row-major int8 cells) for the survey map."""
    spec = covering_grid(geometry.points(), resolution, margin, max_cells)
    grid = empty_grid(spec)
    for polygon in (*geometry.keepouts, *geometry.obstacle_polygons):
        mark_polygon(grid, spec, polygon)
    for circle in (*geometry.circles, *geometry.station_bodies):
        mark_circle(grid, spec, circle)
    return spec, grid


def dock_disagreements(stations: Sequence[StationPose],
                       docks: Dict[str, Sequence[float]],
                       max_distance: float, max_yaw: float) -> List[str]:
    """Where the site's station and docks.yaml's pose for the same name part.

    opennav_docking and battery_supervisor still read docks.yaml; this map draws
    the site's station. Apart by more than the marker can absorb (about a metre
    sideways from the staging pose, docks.yaml's header), docking fails — and
    the station on the map is not where Nav2 drives to dock, so a robot heading
    past it can drive into it. docs/missions.md "Bilinen açıklar".
    """
    problems = []
    for station in stations:
        pose = docks.get(station.name)
        if pose is None:
            problems.append(f"site station {station.name!r} has no dock of that "
                            "name in docks.yaml")
            continue
        apart = math.hypot(station.x - pose[0], station.y - pose[1])
        turn = abs(math.atan2(math.sin(station.yaw - pose[2]),
                              math.cos(station.yaw - pose[2])))
        if apart > max_distance or turn > max_yaw:
            problems.append(
                f"station {station.name!r}: the site puts it at ({station.x:.2f}, "
                f"{station.y:.2f}, yaw {math.degrees(station.yaw):.0f}°) in map, "
                f"docks.yaml at ({pose[0]:.2f}, {pose[1]:.2f}, yaw "
                f"{math.degrees(pose[2]):.0f}°) — {apart:.2f} m / "
                f"{math.degrees(turn):.0f}° apart")
    return problems
