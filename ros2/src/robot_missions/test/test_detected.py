"""The detected-obstacle store. ROS-free."""

import json
import math

import pytest

from robot_missions.detected import DetectedStore, contact_point


@pytest.mark.parametrize("yaw_deg, expected", [
    (0, (1.45, 2.0)), (90, (1.0, 2.45)), (180, (0.55, 2.0)), (-90, (1.0, 1.55))])
def test_the_contact_is_ahead_of_the_robot(yaw_deg, expected):
    x, y = contact_point(1.0, 2.0, math.radians(yaw_deg), 0.45)
    assert (x, y) == (pytest.approx(expected[0]), pytest.approx(expected[1]))


def test_a_press_on_a_known_detection_is_the_same_contact(tmp_path):
    store = DetectedStore(str(tmp_path / "d.json"))
    first, new = store.add_contact(3.0, 1.0, 0.25)
    assert new and first.count == 1
    again, new = store.add_contact(3.1, 1.15, 0.25)       # 0.18 m: on the circle
    assert not new and again is first and first.count == 2


def test_a_press_beside_a_detection_is_more_of_the_rock(tmp_path):
    # 0.3 m away: outside the 0.25 m circle — the same rock from another side,
    # the part not drawn yet. Merging it would leave that part off the map.
    store = DetectedStore(str(tmp_path / "d.json"))
    store.add_contact(3.0, 1.0, 0.25)
    other, new = store.add_contact(3.3, 1.0, 0.25)
    assert new and len(store.items) == 2


def test_an_explicit_merge_distance_still_works(tmp_path):
    store = DetectedStore(str(tmp_path / "d.json"), merge_distance=0.5)
    store.add_contact(3.0, 1.0, 0.25)
    assert not store.add_contact(3.3, 1.2, 0.25)[1]


def test_only_lat_lon_survives_a_restart(tmp_path):
    path = str(tmp_path / "state" / "d.json")
    store = DetectedStore(path)
    item, _ = store.add_contact(3.0, 1.0, 0.25)
    item.lat, item.lon = 41.00001, 29.00002
    store.add_contact(9.0, 9.0, 0.25)                     # never got a lat/lon
    store.save()
    raw = json.load(open(path))["detected"]
    assert len(raw) == 2 and "x" not in raw[0]
    again = DetectedStore(path)
    loaded = again.load()
    assert len(loaded) == 1                               # the one without lat/lon is dropped
    assert (loaded[0].lat, loaded[0].lon) == (41.00001, 29.00002)
    assert math.isnan(loaded[0].x) and again.placed() == []


def test_a_missing_file_is_an_empty_store(tmp_path):
    assert DetectedStore(str(tmp_path / "none.json")).load() == []
