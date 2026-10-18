"""The site's lat/lon in sim_world metres, and back — the SIMULATOR's truth.
ROS-free.

The exact inverse of sim_node's GPS formula (flat earth about the datum, with
sim_node's EARTH_RADIUS_M): where the simulator puts a lat/lon is where it
really is in its world. Only for sim-only, privileged code (the edge expert,
the camera's lawn, the lawn generator). The robot's own conversion is
navsat_transform's (docs/missions.md decision 4) — never this.
"""

from __future__ import annotations

import json
import math
from typing import List, Tuple

from shapely.geometry import Polygon

EARTH_RADIUS_M = 6378137.0       # sim_node's (sim_node imports it from here)


def to_world(lon: float, lat: float, datum_lat: float, datum_lon: float) -> Tuple[float, float]:
    return (math.radians(lon - datum_lon) * EARTH_RADIUS_M * math.cos(math.radians(datum_lat)),
            math.radians(lat - datum_lat) * EARTH_RADIUS_M)


def to_lonlat(x: float, y: float, datum_lat: float, datum_lon: float) -> Tuple[float, float]:
    return (datum_lon + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(datum_lat)))),
            datum_lat + math.degrees(y / EARTH_RADIUS_M))


def areas_in_world(site_file: str, datum_lat: float, datum_lon: float,
                   name: str = "") -> List[Polygon]:
    """The site's `area` exteriors (all, or the one called `name`)."""
    with open(site_file, encoding="utf-8") as handle:
        site = json.load(handle)
    out = []
    for feature in site.get("features", []):
        props = feature.get("properties", {})
        if props.get("kind") != "area" or (name and props.get("name") != name):
            continue
        ring = feature["geometry"]["coordinates"][0]
        out.append(Polygon([to_world(lon, lat, datum_lat, datum_lon) for lon, lat in ring]))
    return out


def area_feature(name: str, polygon: Polygon, datum_lat: float, datum_lon: float) -> dict:
    ring = [list(to_lonlat(x, y, datum_lat, datum_lon)) for x, y in polygon.exterior.coords]
    return {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
            "properties": {"kind": "area", "name": name}}
