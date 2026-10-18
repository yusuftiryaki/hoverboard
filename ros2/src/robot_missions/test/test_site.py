"""The site file reader. ROS-free.

Positions are hand-computed literals around (41.0 N, 29.0 E), never produced by
the module under test (handoff decision 10):
    10 m north = +8.993216e-05 deg lat     (10 / 6371000 rad)
    10 m east  = +1.191613e-04 deg lon     (10 / (6371000 cos 41 deg) rad)
"""

import copy
import json
import math

import pytest

from robot_missions.site import (
    LatLon, SiteError, describe, distance_m, load_site, offset, parse_site)

LAT0, LON0 = 41.0, 29.0
DLAT_10M = 8.993216e-05
DLON_10M = 1.191613e-04


def feature(kind, name, geometry, **properties):
    return {"type": "Feature", "geometry": geometry,
            "properties": {"kind": kind, "name": name, **properties}}


def square(lon, lat, side_lon, side_lat):
    ring = [[lon, lat], [lon + side_lon, lat], [lon + side_lon, lat + side_lat],
            [lon, lat + side_lat], [lon, lat]]
    return {"type": "Polygon", "coordinates": [ring]}


def point(lon, lat):
    return {"type": "Point", "coordinates": [lon, lat]}


def collection(*features):
    return {"type": "FeatureCollection", "features": list(features)}


def full_site():
    return collection(
        feature("area", "bahce", square(LON0, LAT0, DLON_10M, DLAT_10M)),
        feature("keepout", "cicek", square(LON0, LAT0, DLON_10M / 2, DLAT_10M / 2)),
        feature("obstacle", "agac", point(LON0 + DLON_10M, LAT0), radius_m=0.4),
        feature("obstacle", "kaya", square(LON0, LAT0 + DLAT_10M, DLON_10M / 10, DLAT_10M / 10)),
        feature("station", "home", point(LON0, LAT0), heading_deg=90.0),
        feature("route", "tur", {"type": "LineString",
                                 "coordinates": [[LON0, LAT0], [LON0 + DLON_10M, LAT0]]}),
    )


def test_every_kind_parses_into_its_own_group():
    site = parse_site(full_site())
    assert [a.name for a in site.areas] == ["bahce"]
    assert [k.name for k in site.keepouts] == ["cicek"]
    assert [o.name for o in site.circle_obstacles] == ["agac"]
    assert [o.name for o in site.polygon_obstacles] == ["kaya"]
    assert [s.name for s in site.stations] == ["home"]
    assert [r.name for r in site.routes] == ["tur"]
    assert not site.is_empty()


def test_geojson_positions_are_lon_then_lat():
    # [29.0, 41.0] is longitude 29, latitude 41 — Istanbul, not the Sahara.
    site = parse_site(full_site())
    assert site.stations[0].position == LatLon(lat=41.0, lon=29.0)
    assert site.circle_obstacles[0].center == LatLon(lat=41.0, lon=LON0 + DLON_10M)


def test_the_closing_position_is_dropped_and_holes_kept():
    hole = [[LON0 + 0.25 * DLON_10M, LAT0 + 0.25 * DLAT_10M],
            [LON0 + 0.75 * DLON_10M, LAT0 + 0.25 * DLAT_10M],
            [LON0 + 0.75 * DLON_10M, LAT0 + 0.75 * DLAT_10M],
            [LON0 + 0.25 * DLON_10M, LAT0 + 0.25 * DLAT_10M]]
    geometry = square(LON0, LAT0, DLON_10M, DLAT_10M)
    geometry["coordinates"].append(hole)
    site = parse_site(collection(feature("keepout", "halka", geometry)))
    polygon = site.keepouts[0].polygon
    assert len(polygon.exterior) == 4
    assert polygon.exterior[0] == LatLon(LAT0, LON0)
    assert len(polygon.holes) == 1 and len(polygon.holes[0]) == 3


def test_an_empty_collection_is_a_valid_empty_site():
    site = parse_site(collection())
    assert site.is_empty()


def test_load_site_reads_a_file(tmp_path):
    path = tmp_path / "site.geojson"
    path.write_text(json.dumps(full_site()), encoding="utf-8")
    assert load_site(str(path)).stations[0].heading_deg == 90.0


def test_not_json_is_a_site_error(tmp_path):
    path = tmp_path / "site.geojson"
    path.write_text("{ nope", encoding="utf-8")
    with pytest.raises(SiteError, match="not JSON"):
        load_site(str(path))


def broken(mutate):
    document = full_site()
    mutate(document)
    return document


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d.update(type="Feature"), "FeatureCollection"),
    (lambda d: d.pop("features"), "features"),
    (lambda d: d["features"][0]["properties"].update(kind="lawn"), "kind must be one of"),
    (lambda d: d["features"][0]["properties"].pop("name"), "name is required"),
    (lambda d: d["features"][0]["properties"].update(name="  "), "name is required"),
    (lambda d: d["features"][1]["properties"].update(name="cicek")
        or d["features"].append(copy.deepcopy(d["features"][1])), "both named"),
    (lambda d: d["features"][0]["geometry"]["coordinates"][0].pop(), "not closed"),
    (lambda d: d["features"][0]["geometry"].update(
        coordinates=[[[LON0, LAT0], [LON0 + DLON_10M, LAT0], [LON0, LAT0]]]), "at least 4"),
    (lambda d: d["features"][0]["geometry"].update(
        coordinates=[[[LON0, LAT0], [LON0 + DLON_10M, LAT0], [LON0 + DLON_10M, LAT0],
                      [LON0, LAT0]]]), "fewer than 3 distinct"),
    (lambda d: d["features"][2]["properties"].pop("radius_m"), "radius_m"),
    (lambda d: d["features"][2]["properties"].update(radius_m=-0.4), "radius_m"),
    (lambda d: d["features"][2]["properties"].update(radius_m=True), "radius_m"),
    (lambda d: d["features"][2]["properties"].update(radius_m=float("nan")), "radius_m"),
    (lambda d: d["features"][4]["properties"].update(heading_deg=360.0), "heading_deg"),
    (lambda d: d["features"][4]["properties"].update(heading_deg=-1.0), "heading_deg"),
    (lambda d: d["features"][4]["properties"].pop("heading_deg"), "heading_deg"),
    (lambda d: d["features"][4]["geometry"].update(coordinates=[29.0, 91.0]), "not on Earth"),
    (lambda d: d["features"][4]["geometry"].update(coordinates=[29.0]), "a position is"),
    (lambda d: d["features"][4]["geometry"].update(type="Polygon"), "must be a Point"),
    (lambda d: d["features"][0]["geometry"].update(type="Point"), "must be a Polygon"),
    (lambda d: d["features"][5]["geometry"]["coordinates"].pop(), "at least two points"),
])
def test_a_bad_feature_rejects_the_whole_file(mutate, message):
    with pytest.raises(SiteError, match=message):
        parse_site(broken(mutate))


def test_the_same_name_may_repeat_across_kinds():
    # An area and a keep-out both called "on": missions look names up per kind.
    site = parse_site(collection(
        feature("area", "on", square(LON0, LAT0, DLON_10M, DLAT_10M)),
        feature("keepout", "on", square(LON0, LAT0, DLON_10M, DLAT_10M))))
    assert site.areas[0].name == site.keepouts[0].name == "on"


def test_a_position_written_lat_lon_is_caught_by_the_spread():
    # Istanbul written backwards: [41, 29] is a valid point in Egypt.
    swapped = collection(
        feature("station", "home", point(LON0, LAT0), heading_deg=0.0),
        feature("obstacle", "agac", point(LAT0, LON0), radius_m=0.4))
    with pytest.raises(SiteError, match=r"\[lat, lon\] instead of"):
        parse_site(swapped)


def test_offset_matches_hand_computed_degrees():
    north = offset(LatLon(LAT0, LON0), 0.0, 10.0)
    assert north.lat == pytest.approx(LAT0 + DLAT_10M, abs=1e-11)
    assert north.lon == pytest.approx(LON0, abs=1e-12)
    east = offset(LatLon(LAT0, LON0), 90.0, 10.0)
    assert east.lat == pytest.approx(LAT0, abs=1e-11)
    assert east.lon == pytest.approx(LON0 + DLON_10M, abs=1e-10)
    # Compass bearings run clockwise: 270 is west.
    west = offset(LatLon(LAT0, LON0), 270.0, 10.0)
    assert west.lon == pytest.approx(LON0 - DLON_10M, abs=1e-10)


@pytest.mark.parametrize("bearing", range(0, 360, 45))
def test_distance_and_offset_agree_all_the_way_round(bearing):
    there = offset(LatLon(LAT0, LON0), float(bearing), 25.0)
    assert distance_m(LatLon(LAT0, LON0), there) == pytest.approx(25.0, abs=1e-3)


def test_distance_matches_a_hand_computed_value():
    # 10 m north, by construction of DLAT_10M.
    assert distance_m(LatLon(LAT0, LON0), LatLon(LAT0 + DLAT_10M, LON0)) == \
        pytest.approx(10.0, abs=1e-4)
    assert math.isclose(distance_m(LatLon(LAT0, LON0), LatLon(LAT0, LON0)), 0.0)


def test_a_site_file_not_saved_yet_can_be_an_empty_site(tmp_path):
    path = str(tmp_path / "later.geojson")
    assert load_site(path, missing_ok=True).is_empty()
    with pytest.raises(OSError):
        load_site(path)                  # by default a missing file is still an error


# --- poi: labelled points (docs/missions.md 5b) -------------------------------

def test_a_poi_is_a_marker_unless_it_has_a_radius():
    site = parse_site(collection(
        feature("poi", "kuyu-1", point(LON0, LAT0), label="kuyu", radius_m=0.6,
                note="kapağı çürük"),
        feature("poi", "taş-1", point(LON0 + DLON_10M, LAT0), label="sınır taşı")))
    well, stone = site.pois
    assert (well.label, well.note, well.radius_m) == ("kuyu", "kapağı çürük", 0.6)
    assert (stone.label, stone.note, stone.radius_m) == ("sınır taşı", "", None)
    # Only the one with a radius is an obstacle — for the map and the planner.
    assert [(o.name, o.center, o.radius_m) for o in site.circle_obstacles] == [
        ("kuyu-1", LatLon(LAT0, LON0), 0.6)]
    assert describe(site) == "2 poi"           # not counted twice


def test_a_poi_alone_is_not_an_empty_site():
    site = parse_site(collection(feature("poi", "p", point(LON0, LAT0), label="direk")))
    assert not site.is_empty()


@pytest.mark.parametrize("properties, message", [
    ({}, "needs a label"),
    ({"label": "  "}, "needs a label"),
    ({"label": 3}, "needs a label"),
    ({"label": "kuyu", "note": 5}, "note must be text"),
    ({"label": "kuyu", "radius_m": 0}, "radius_m"),
    ({"label": "kuyu", "radius_m": "1"}, "radius_m"),
])
def test_a_bad_poi_rejects_the_file(properties, message):
    with pytest.raises(SiteError, match=message):
        parse_site(collection(feature("poi", "p", point(LON0, LAT0), **properties)))


def test_a_poi_must_be_a_point():
    with pytest.raises(SiteError, match="must be a Point"):
        parse_site(collection(feature("poi", "p", square(LON0, LAT0, DLON_10M, DLAT_10M),
                                      label="kuyu")))


def test_a_poi_far_away_trips_the_spread_check():
    with pytest.raises(SiteError, match="spans more than"):
        parse_site(collection(feature("area", "a", square(LON0, LAT0, DLON_10M, DLAT_10M)),
                              feature("poi", "p", point(LAT0, LON0), label="kuyu")))
