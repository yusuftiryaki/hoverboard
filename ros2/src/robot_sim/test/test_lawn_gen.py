"""lawn_gen: random lawns the expert can drive. ROS-free."""

import json

import pytest
from shapely.geometry import Point, Polygon, box

from robot_sim.lawn_gen import LawnSpec, check, interior_angles, random_lawn
from robot_sim.site_world import area_feature, areas_in_world, to_lonlat, to_world


def test_interior_angles_hand_computed():
    assert sorted(round(a) for a in interior_angles(box(0, 0, 2, 1))) == [90] * 4
    l_shape = Polygon([(0, 0), (4, 0), (4, 1), (1, 1), (1, 3), (0, 3)])
    assert sorted(round(a) for a in interior_angles(l_shape)) == [90] * 5 + [270]


@pytest.mark.parametrize("seed", range(12))
def test_every_lawn_is_valid_contains_the_start_and_the_expert_drives_it(seed):
    spec = LawnSpec()
    lawn = random_lawn(seed, spec)
    assert lawn.is_valid and spec.area_m2[0] <= lawn.area <= spec.area_m2[1]
    assert lawn.exterior.distance(Point(0, 0)) >= spec.margin_m
    metrics = check(lawn, spec)          # the acceptance itself, re-run
    assert metrics["error_mean"] < 0.06 and metrics["outside"] == 0.0


def test_lawns_differ_by_seed_and_repeat_by_seed():
    assert random_lawn(3).equals(random_lawn(3))
    assert not random_lawn(3).equals(random_lawn(4))
    corners = {len(random_lawn(s).exterior.coords) - 1 for s in range(20)}
    assert len(corners) >= 3, "corner counts should vary"


@pytest.mark.parametrize("polygon, message", [
    (box(0.5, -3, 6, 3), "too near the edge"),                       # origin outside
    (box(-1.2, -1.2, 1.2, 1.2), "area"),                              # too small
    # A dumbbell: the 1 m corridor is narrower than 2 x (offset + width).
    (box(-3, -3, 1, 3).union(box(1, -0.5, 3, 0.5)).union(box(3, -3, 7, 3)), "narrow"),
    (Polygon([(-4, -4), (9, -4), (-4, 4)]), "corner"),                 # a 32 deg spike
])
def test_unusable_lawns_are_refused(polygon, message):
    with pytest.raises(ValueError, match=message):
        check(polygon, LawnSpec())


def test_site_world_round_trips_and_writes_a_site_area(tmp_path):
    lon, lat = to_lonlat(3.0, -2.0, 41.0, 29.0)
    assert to_world(lon, lat, 41.0, 29.0) == pytest.approx((3.0, -2.0), abs=1e-9)
    lawn = random_lawn(1)
    site = tmp_path / "site.geojson"
    site.write_text(json.dumps({"type": "FeatureCollection",
                                "features": [area_feature("cim", lawn, 41.0, 29.0)]}))
    (back,) = areas_in_world(str(site), 41.0, 29.0)
    assert back.symmetric_difference(lawn).area < 1e-6
