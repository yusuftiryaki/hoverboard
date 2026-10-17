"""The site file: everything the robot knows about its field, in lat/lon.

ROS-free. One GeoJSON FeatureCollection holds the areas missions run in, the
keep-out zones and surveyed obstacles the obstacle map draws, the charging
station and the recorded routes — whether they were drawn on a map at a desk,
recorded by driving the robot, or typed by hand. docs/missions.md has the
format; this module is its only reader.

⚠️ GEOJSON IS [LONGITUDE, LATITUDE]. Every position below is read in that order
and stored as LatLon(lat, lon) with named fields, so the swap happens in exactly
one place. A swapped file is still made of valid numbers, which is why the
spread check exists: Istanbul read backwards lands ~1300 km away from itself,
and a site several kilometres across is not a field this robot drives.

⚠️ NOTHING IS HALF-READ. A file with one bad feature is rejected whole. A map
missing one keep-out zone looks exactly like a correct map, and the robot would
drive into the flower bed nobody can see is gone.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

EARTH_RADIUS_M = 6371000.0
# Further apart than this and the file is almost certainly broken (lat/lon
# swapped, a point typed with a missing digit), not a real field.
MAX_SPREAD_M = 5000.0

KINDS = ("area", "keepout", "obstacle", "station", "route")


class SiteError(ValueError):
    """The site file is malformed. The message names the feature."""


@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float


Ring = Tuple[LatLon, ...]


@dataclass(frozen=True)
class Polygon:
    """An exterior ring and its holes, each WITHOUT GeoJSON's repeated closing
    position — the ring closes implicitly from its last point to its first."""

    exterior: Ring
    holes: Tuple[Ring, ...] = ()

    def rings(self) -> Tuple[Ring, ...]:
        return (self.exterior, *self.holes)


@dataclass(frozen=True)
class Area:
    name: str
    polygon: Polygon


@dataclass(frozen=True)
class Keepout:
    name: str
    polygon: Polygon


@dataclass(frozen=True)
class CircleObstacle:
    name: str
    center: LatLon
    radius_m: float


@dataclass(frozen=True)
class PolygonObstacle:
    name: str
    polygon: Polygon


@dataclass(frozen=True)
class Station:
    """The DOCKED robot: base_link's position when it sits on the contacts, and
    the compass bearing it faces then (0 = north, clockwise) — the same meaning
    as docks.yaml's pose, in the site's coordinates."""

    name: str
    position: LatLon
    heading_deg: float


@dataclass(frozen=True)
class Route:
    name: str
    points: Tuple[LatLon, ...]


@dataclass(frozen=True)
class Site:
    areas: Tuple[Area, ...] = ()
    keepouts: Tuple[Keepout, ...] = ()
    circle_obstacles: Tuple[CircleObstacle, ...] = ()
    polygon_obstacles: Tuple[PolygonObstacle, ...] = ()
    stations: Tuple[Station, ...] = ()
    routes: Tuple[Route, ...] = ()

    def is_empty(self) -> bool:
        return not any((self.areas, self.keepouts, self.circle_obstacles,
                        self.polygon_obstacles, self.stations, self.routes))


def load_site(path: str, missing_ok: bool = False) -> Site:
    """Read and validate a site file. Raises SiteError (or OSError).

    missing_ok: a file that does not exist YET is an empty site — the web page
    creates it on its first save. ⚠️ Without this, a robot configured with a
    site_file nobody had saved yet lost site_map_server and battery_supervisor
    at boot (FileNotFoundError): no survey map, so Nav2 aborted every goal.
    Found by running the simulator demo by hand."""
    if missing_ok and not os.path.exists(path):
        return Site()
    with open(path, encoding="utf-8") as handle:
        try:
            document = json.load(handle)
        except json.JSONDecodeError as exc:
            raise SiteError(f"{path}: not JSON: {exc}") from exc
    return parse_site(document)


def parse_site(document: Any) -> Site:
    if not isinstance(document, dict) or document.get("type") != "FeatureCollection":
        raise SiteError("a site file is a GeoJSON FeatureCollection")
    features = document.get("features")
    if not isinstance(features, list):
        raise SiteError("FeatureCollection has no 'features' list")

    found: Dict[str, List[Any]] = {kind: [] for kind in KINDS}
    for index, feature in enumerate(features):
        kind, item = _parse_feature(index, feature)
        if any(other.name == item.name for other in found[kind]):
            raise SiteError(f"two {kind} features are both named {item.name!r}; "
                            "missions refer to them by name")
        found[kind].append(item)

    obstacles = found["obstacle"]
    site = Site(
        areas=tuple(found["area"]),
        keepouts=tuple(found["keepout"]),
        circle_obstacles=tuple(o for o in obstacles if isinstance(o, CircleObstacle)),
        polygon_obstacles=tuple(o for o in obstacles if isinstance(o, PolygonObstacle)),
        stations=tuple(found["station"]),
        routes=tuple(found["route"]),
    )
    _check_spread(site)
    return site


def all_positions(site: Site) -> List[LatLon]:
    points: List[LatLon] = []
    for item in (*site.areas, *site.keepouts, *site.polygon_obstacles):
        for ring in item.polygon.rings():
            points.extend(ring)
    points.extend(o.center for o in site.circle_obstacles)
    points.extend(s.position for s in site.stations)
    for route in site.routes:
        points.extend(route.points)
    return points


def distance_m(a: LatLon, b: LatLon) -> float:
    """Haversine. Only for sanity checks — never for navigation geometry, which
    goes through navsat_transform (docs/missions.md decision 4)."""
    phi1, phi2 = math.radians(a.lat), math.radians(b.lat)
    dphi = phi2 - phi1
    dlam = math.radians(b.lon - a.lon)
    h = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def offset(origin: LatLon, bearing_deg: float, distance: float) -> LatLon:
    """The point `distance` metres from `origin` along a compass bearing.

    Local spherical approximation; used only to turn a heading into a second
    point so navsat_transform can carry the DIRECTION into `map` too. Over a few
    metres its error is far below anything the robot can steer to.
    """
    bearing = math.radians(bearing_deg)
    north = distance * math.cos(bearing)
    east = distance * math.sin(bearing)
    return LatLon(
        lat=origin.lat + math.degrees(north / EARTH_RADIUS_M),
        lon=origin.lon + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(origin.lat)))),
    )


# --- parsing ----------------------------------------------------------------

def _parse_feature(index: int, feature: Any):
    where = f"feature #{index}"
    if not isinstance(feature, dict) or feature.get("type") != "Feature":
        raise SiteError(f"{where}: not a GeoJSON Feature")
    properties = feature.get("properties")
    if not isinstance(properties, dict):
        raise SiteError(f"{where}: no properties")
    name = properties.get("name")
    if not isinstance(name, str) or not name.strip():
        raise SiteError(f"{where}: properties.name is required")
    where = f"feature #{index} ({name!r})"
    kind = properties.get("kind")
    if kind not in KINDS:
        raise SiteError(f"{where}: properties.kind must be one of {KINDS}, got {kind!r}")
    geometry = feature.get("geometry")
    if not isinstance(geometry, dict):
        raise SiteError(f"{where}: no geometry")
    gtype = geometry.get("type")
    coords = geometry.get("coordinates")

    def need(*types):
        if gtype not in types:
            raise SiteError(f"{where}: a {kind} must be a {' or '.join(types)}, got {gtype!r}")

    if kind in ("area", "keepout"):
        need("Polygon")
        polygon = _polygon(where, coords)
        return kind, (Area if kind == "area" else Keepout)(name, polygon)
    if kind == "obstacle":
        need("Point", "Polygon")
        if gtype == "Polygon":
            return kind, PolygonObstacle(name, _polygon(where, coords))
        radius = _positive(where, properties, "radius_m")
        return kind, CircleObstacle(name, _position(where, coords), radius)
    if kind == "station":
        need("Point")
        heading = properties.get("heading_deg")
        if not _is_number(heading) or not 0.0 <= heading < 360.0:
            raise SiteError(f"{where}: heading_deg must be a compass bearing in "
                            f"[0, 360), got {heading!r}")
        return kind, Station(name, _position(where, coords), float(heading))
    need("LineString")
    if not isinstance(coords, list) or len(coords) < 2:
        raise SiteError(f"{where}: a route needs at least two points")
    return kind, Route(name, tuple(_position(where, c) for c in coords))


def _polygon(where: str, coords: Any) -> Polygon:
    if not isinstance(coords, list) or not coords:
        raise SiteError(f"{where}: a Polygon needs at least an exterior ring")
    rings = tuple(_ring(f"{where} ring {i}", ring) for i, ring in enumerate(coords))
    return Polygon(exterior=rings[0], holes=rings[1:])


def _ring(where: str, coords: Any) -> Ring:
    if not isinstance(coords, list) or len(coords) < 4:
        raise SiteError(f"{where}: a linear ring has at least 4 positions "
                        "(3 corners + the closing repeat of the first)")
    points = tuple(_position(where, c) for c in coords)
    if points[0] != points[-1]:
        raise SiteError(f"{where}: ring is not closed — GeoJSON repeats the first "
                        "position at the end")
    points = points[:-1]
    if len(set(points)) < 3:
        raise SiteError(f"{where}: ring has fewer than 3 distinct corners")
    return points


def _position(where: str, coords: Any) -> LatLon:
    if (not isinstance(coords, list) or len(coords) not in (2, 3)
            or not all(_is_number(c) for c in coords)):
        raise SiteError(f"{where}: a position is [lon, lat] or [lon, lat, alt], got {coords!r}")
    lon, lat = float(coords[0]), float(coords[1])
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise SiteError(f"{where}: [lon, lat] = [{lon}, {lat}] is not on Earth")
    return LatLon(lat=lat, lon=lon)


def _positive(where: str, properties: Dict[str, Any], key: str) -> float:
    value = properties.get(key)
    if not _is_number(value) or value <= 0.0:
        raise SiteError(f"{where}: {key} must be a positive number, got {value!r}")
    return float(value)


def _is_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _check_spread(site: Site) -> None:
    points = all_positions(site)
    if not points:
        return
    first = points[0]
    for point in points[1:]:
        if distance_m(first, point) > MAX_SPREAD_M:
            raise SiteError(
                f"site spans more than {MAX_SPREAD_M:.0f} m ({first} to {point}). "
                "That is not one field — most likely a position written as "
                "[lat, lon] instead of GeoJSON's [lon, lat].")


def describe(site: Site) -> str:
    parts: Sequence[Tuple[str, int]] = (
        ("area", len(site.areas)), ("keepout", len(site.keepouts)),
        ("obstacle", len(site.circle_obstacles) + len(site.polygon_obstacles)),
        ("station", len(site.stations)), ("route", len(site.routes)))
    return ", ".join(f"{count} {name}" for name, count in parts if count) or "empty"
