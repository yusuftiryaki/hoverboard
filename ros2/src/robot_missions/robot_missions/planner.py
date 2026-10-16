"""Mission geometry -> waypoints, in `map` metres. ROS-free.

Every planner here takes shapes already in `map` (mission.py converts the
site's lat/lon through navsat_transform first, docs/missions.md decision 4) and
returns a Plan: the poses to drive through, in order. Nav2 does the driving
BETWEEN poses — it routes around the survey map's obstacles on its own — so a
plan only says where to be, not how to get there.

    coverage   boustrophedon rows across the free region, then (optionally) a
               pass round its edges
    patrol     the free region's perimeter, or random points inside it
    route      a recorded / drawn line of points
    goto       one point

⚠️ The free region is computed ONCE, here, for every type: the area shrunk by
boundary_margin, minus every keep-out, obstacle and station body grown by
`clearance`. A planner that drew its own idea of "free" would send Nav2 goals
inside an obstacle's inflation, and Nav2 rejects or abandons those — the robot
would skip rows with nothing in the log but a failed goal.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

XY = Tuple[float, float]

# Rings are simplified this much before edge passes are sampled: a buffered
# circle has a vertex every few centimetres, and every vertex would become a
# Nav2 goal. ⚠️ A simplified ring's chords cut INSIDE a hole's ring, i.e.
# toward the obstacle, by up to this much — it eats into the clearance. 0.1
# was tried first and put the robot's body 2 cm from a tree
# (test_survey_obstacles_are_cut_out_of_the_coverage); 0.02 stays well inside
# RobotSpec.obstacle_margin (0.1) and still turns a 1 m circle into ~16 poses.
RING_TOLERANCE = 0.02


class PlanError(ValueError):
    """The mission cannot be planned as given. The message says why."""


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float                  # rad, `map`, REP-103
    # True = the leg ARRIVING here is work (cutting, patrolling); False = a
    # transit to the start of the next piece of work. Phase 3 switches the
    # trimmer on these, and progress is counted in work legs.
    work: bool = True


@dataclass(frozen=True)
class Plan:
    kind: str
    waypoints: Tuple[Waypoint, ...]
    # How many times to run the waypoints: 1 = once, 0 = until stopped.
    repeat: int = 1
    info: Dict[str, float] = field(default_factory=dict)
    # Drive work legs as rows: straight FollowPath goals in `odom` on the
    # precise profile (mission_runner.py), not loose navigate_to_pose goals.
    # Coverage needs it — rows 0.2-0.4 m apart; patrol and routes do not.
    precise: bool = False


# --- the free region ------------------------------------------------------------

def free_region(area: Polygon, obstacles: Iterable[BaseGeometry],
                boundary_margin: float, clearance: float) -> BaseGeometry:
    """The part of `area` the robot's CENTRE may be planned through.

    The area shrinks by boundary_margin with MITRED corners — a rectangular
    lawn stays rectangular, where round joins would leave its corners uncut.
    Obstacles grow by `clearance` with round joins: the robot is round.
    """
    if boundary_margin < 0 or clearance < 0:
        raise PlanError("boundary_margin and clearance must be >= 0")
    if not area.is_valid:
        raise PlanError("the area polygon intersects itself — redraw it")
    region = area.buffer(-boundary_margin, join_style="mitre") if boundary_margin else area
    blocked = [g.buffer(clearance) if clearance else g for g in obstacles]
    if blocked:
        region = region.difference(unary_union(blocked))
    region = region.buffer(0)
    if region.is_empty or region.area < 1e-6:
        raise PlanError(
            f"nothing of the area is left to drive in after a {boundary_margin} m "
            f"margin and {clearance} m clearance round obstacles")
    return region


def polygons_of(region: BaseGeometry) -> List[Polygon]:
    if isinstance(region, Polygon):
        return [region]
    if isinstance(region, MultiPolygon):
        return list(region.geoms)
    return [g for g in getattr(region, "geoms", []) if isinstance(g, Polygon)]


# --- coverage -------------------------------------------------------------------

def auto_angle(region: BaseGeometry) -> float:
    """Rows along the long side of the region's minimum rotated rectangle:
    fewest rows, so fewest turns — turning in place is where a diff-drive robot
    loses the most time and scuffs the most grass. In (-pi/2, pi/2]."""
    corners = list(region.minimum_rotated_rectangle.exterior.coords)[:4]
    (x0, y0), (x1, y1), (x2, y2) = corners[0], corners[1], corners[2]
    if math.hypot(x1 - x0, y1 - y0) >= math.hypot(x2 - x1, y2 - y1):
        angle = math.atan2(y1 - y0, x1 - x0)
    else:
        angle = math.atan2(y2 - y1, x2 - x1)
    while angle <= -math.pi / 2:
        angle += math.pi
    while angle > math.pi / 2:
        angle -= math.pi
    return angle


def row_offsets(low: float, high: float, spacing: float) -> List[float]:
    """Row centre lines across [low, high], `spacing` apart and centred, so the
    two outer rows sit the same distance (<= spacing / 2) from the edges:
    height 5, spacing 1 -> 0.5, 1.5, ..., 4.5; height 4.6 -> 0.3, ..., 4.3."""
    height = high - low
    count = max(1, int(math.ceil(height / spacing - 1e-9)))
    first = low + (height - (count - 1) * spacing) / 2.0
    return [first + k * spacing for k in range(count)]


def coverage(region: BaseGeometry, spacing: float, angle: Optional[float] = None,
             start: Optional[XY] = None, min_segment: float = 0.3,
             perimeter_pass: bool = True, perimeter_step: float = 2.0) -> Plan:
    """Boustrophedon rows across `region`, `spacing` apart, along `angle`
    (rad in `map`; None = auto_angle).

    Rows are cut out of the region, so a row crossing a hole becomes two
    segments. The segments are then chained greedily: from wherever the robot
    is, the nearest unvisited segment end, entered there, driven to its other
    end. On a plain rectangle that is exactly the serpentine; round a hole it
    finishes one side before crossing — Nav2 routes the crossing.

    perimeter_pass: afterwards, once round every edge of the region (outer
    boundary and every hole), at the same centre-line offset as the rows.
    Rows end where they meet a slanted edge at a slant; this trims the strip
    they leave.
    """
    if spacing <= 0:
        raise PlanError(f"row spacing must be > 0, got {spacing}")
    angle = auto_angle(region) if angle is None else angle
    # Rotate the region so rows run along +x; rotate the answer back.
    flat = affinity.rotate(region, -angle, origin=(0.0, 0.0), use_radians=True)
    min_x, min_y, max_x, max_y = flat.bounds

    segments: List[Tuple[int, XY, XY]] = []
    for row, y in enumerate(row_offsets(min_y, max_y, spacing)):
        line = LineString([(min_x - 1.0, y), (max_x + 1.0, y)])
        for piece in _lines_of(flat.intersection(line)):
            (ax, ay), (bx, by) = piece.coords[0], piece.coords[-1]
            if piece.length >= min_segment:
                segments.append((row, (min(ax, bx), ay), (max(ax, bx), by)))
    if not segments:
        raise PlanError(f"no row of the area is longer than {min_segment} m")

    here = _rotate(start, -angle) if start is not None else segments[0][1]
    flat_waypoints: List[Tuple[float, float, float, bool]] = []
    remaining = list(range(len(segments)))
    rows_used = set()
    while remaining:
        best = min(remaining, key=lambda i: (
            min(_dist(here, segments[i][1]), _dist(here, segments[i][2])),
            segments[i][0], segments[i][1][0]))
        remaining.remove(best)
        row, left, right = segments[best]
        rows_used.add(row)
        enter, leave = (left, right) if _dist(here, left) <= _dist(here, right) else (right, left)
        heading = math.atan2(leave[1] - enter[1], leave[0] - enter[0])
        flat_waypoints.append((enter[0], enter[1], heading, False))
        flat_waypoints.append((leave[0], leave[1], heading, True))
        here = leave

    waypoints = [Waypoint(*_rotate((x, y), angle), yaw + angle, work)
                 for x, y, yaw, work in flat_waypoints]
    work_length = sum(math.hypot(r[0] - l[0], r[1] - l[1]) for _, l, r in segments)
    info = {"rows": float(len(rows_used)), "segments": float(len(segments)),
            "row_length_m": work_length, "angle_rad": angle, "spacing_m": spacing,
            "region_area_m2": region.area}
    if perimeter_pass:
        edge = _perimeter_waypoints(region, perimeter_step, (waypoints[-1].x, waypoints[-1].y))
        if edge:
            edge[0] = Waypoint(edge[0].x, edge[0].y, edge[0].yaw, work=False)
            waypoints.extend(edge)
            info["perimeter_m"] = sum(p.exterior.length + sum(i.length for i in p.interiors)
                                      for p in polygons_of(region))
    return Plan("coverage", tuple(_wrap_yaws(waypoints)), repeat=1, info=info, precise=True)


# --- patrol ---------------------------------------------------------------------

def patrol_perimeter(region: BaseGeometry, step: float = 2.0, laps: int = 1,
                     start: Optional[XY] = None) -> Plan:
    """Round the region's outer boundary (each piece, if the region is split),
    counter-clockwise, starting at the corner nearest `start`; closed, so a
    lap ends where it began. Waypoints at every corner and at most `step` apart."""
    if laps < 0:
        raise PlanError(f"laps must be >= 0 (0 = until stopped), got {laps}")
    waypoints: List[Waypoint] = []
    here = start
    for polygon in _nearest_first(polygons_of(region), start):
        ring = _sampled_ring(polygon.exterior.coords, step, counter_clockwise=True)
        ring = _starting_nearest(ring, here)
        ring.append(ring[0])
        waypoints.extend(_poses_along(ring, first_is_work=False))
        here = ring[-1]
    return Plan("patrol", tuple(waypoints), repeat=laps,
                info={"lap_m": sum(p.exterior.length for p in polygons_of(region))})


def patrol_random(region: BaseGeometry, points: int = 10, seed: int = 0,
                  min_separation: float = 3.0, laps: int = 1,
                  start: Optional[XY] = None) -> Plan:
    """`points` random points inside the region, each at least min_separation
    from the one before (so every leg actually moves the robot). Seeded and
    deterministic: phase 3 re-plans each lap with seed + lap for variety."""
    if points < 1 or laps < 0:
        raise PlanError("points must be >= 1 and laps >= 0")
    rng = random.Random(seed)
    min_x, min_y, max_x, max_y = region.bounds
    chosen: List[XY] = []
    previous = start
    attempts = 0
    while len(chosen) < points:
        attempts += 1
        if attempts > 10000 * points:
            raise PlanError(f"could not place {points} points {min_separation} m apart "
                            "inside the area — lower min_separation")
        candidate = (rng.uniform(min_x, max_x), rng.uniform(min_y, max_y))
        if not region.contains(Point(candidate)):
            continue
        if previous is not None and _dist(candidate, previous) < min_separation:
            continue
        chosen.append(candidate)
        previous = candidate
    # With a start, the first pose faces away from it; the start itself is not
    # a waypoint.
    path = [start, *chosen] if start is not None else chosen
    poses = _poses_along(path, first_is_work=True)
    if start is not None:
        poses = poses[1:]
    return Plan("patrol", tuple(poses), repeat=laps, info={"points": float(points)})


# --- route / goto -----------------------------------------------------------------

def route(points: Sequence[XY], repeat: int = 1, back_and_forth: bool = False) -> Plan:
    """Through `points` in order. back_and_forth: then back the same way, so a
    repeated route does not need a jump from its end to its start."""
    if len(points) < 2:
        raise PlanError("a route needs at least two points")
    if repeat < 0:
        raise PlanError(f"repeat must be >= 0 (0 = until stopped), got {repeat}")
    path = list(points)
    if back_and_forth:
        path += list(reversed(path[:-1]))
    waypoints = _poses_along(path, first_is_work=False)
    return Plan("route", tuple(waypoints), repeat=repeat,
                info={"length_m": sum(_dist(a, b) for a, b in zip(path, path[1:]))})


def goto(target: XY, yaw: Optional[float] = None, start: Optional[XY] = None) -> Plan:
    """One pose. Without a yaw, face the way the robot drove to get there."""
    if yaw is None:
        yaw = math.atan2(target[1] - start[1], target[0] - start[0]) if start else 0.0
    return Plan("goto", (Waypoint(target[0], target[1], yaw, work=True),))


# --- helpers ------------------------------------------------------------------------

def _lines_of(geometry: BaseGeometry) -> List[LineString]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    return [g for g in getattr(geometry, "geoms", []) if isinstance(g, LineString)]


def _rotate(point: XY, angle: float) -> XY:
    c, s = math.cos(angle), math.sin(angle)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def _dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _wrap_yaws(waypoints: Iterable[Waypoint]) -> List[Waypoint]:
    return [Waypoint(w.x, w.y, _wrap(w.yaw), w.work) for w in waypoints]


def _sampled_ring(coords, step: float, counter_clockwise: bool) -> List[XY]:
    ring = list(LineString(coords).simplify(RING_TOLERANCE).coords)
    if ring[0] == ring[-1]:
        ring = ring[:-1]
    signed = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]))
    if (signed > 0) != counter_clockwise:
        ring.reverse()
    out: List[XY] = []
    for a, b in zip(ring, ring[1:] + ring[:1]):
        pieces = max(1, int(math.ceil(_dist(a, b) / step - 1e-9)))
        for k in range(pieces):
            out.append((a[0] + (b[0] - a[0]) * k / pieces, a[1] + (b[1] - a[1]) * k / pieces))
    return out


def _starting_nearest(ring: List[XY], point: Optional[XY]) -> List[XY]:
    if point is None:
        return ring
    index = min(range(len(ring)), key=lambda i: (_dist(ring[i], point), i))
    return ring[index:] + ring[:index]


def _nearest_first(polygons: List[Polygon], start: Optional[XY]) -> List[Polygon]:
    if start is None:
        return polygons
    return sorted(polygons, key=lambda p: p.exterior.distance(Point(start)))


def _poses_along(path: Sequence[XY], first_is_work: bool) -> List[Waypoint]:
    """A pose per point, each facing the next (the last keeps the previous
    heading). The first leg is a transit unless first_is_work."""
    poses = []
    for i, (x, y) in enumerate(path):
        if i + 1 < len(path):
            yaw = math.atan2(path[i + 1][1] - y, path[i + 1][0] - x)
        elif i > 0:
            yaw = math.atan2(y - path[i - 1][1], x - path[i - 1][0])
        else:
            yaw = 0.0
        poses.append(Waypoint(x, y, yaw, work=first_is_work or i > 0))
    return poses


def _perimeter_waypoints(region: BaseGeometry, step: float, here: XY) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    for polygon in _nearest_first(polygons_of(region), here):
        rings = [polygon.exterior, *polygon.interiors]
        for ring_coords in rings:
            ring = _starting_nearest(_sampled_ring(ring_coords.coords, step, True), here)
            ring.append(ring[0])
            poses = _poses_along(ring, first_is_work=False)
            waypoints.extend(poses)
            here = ring[-1]
    return waypoints
