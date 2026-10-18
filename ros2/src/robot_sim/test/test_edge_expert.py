"""edge_expert: the privileged expert on an ideal unicycle, and the metric on
hand-made drives. ROS-free."""

import math

import pytest
from shapely.geometry import Polygon

from robot_sim.edge_expert import (
    EdgeExpert, ExpertConfig, drive_unicycle, edge_metrics, edge_path, signed_distance)

LAWN = Polygon([(-1, -1), (5, -1), (5, 3), (-1, 3)])            # 6 x 4 m
L_SHAPE = Polygon([(0, 0), (8, 0), (8, 3), (3, 3), (3, 7), (0, 7)])


def test_signed_distance_is_positive_inside():
    assert signed_distance(LAWN, 0.0, 0.0) == pytest.approx(1.0)
    assert signed_distance(LAWN, -1.5, 0.0) == pytest.approx(-0.5)


def test_the_path_is_the_boundary_pulled_in_and_ordered():
    path = edge_path(LAWN, 0.4, "ccw")
    assert path.length == pytest.approx(2 * (5.2 + 3.2))          # 6-0.8 by 4-0.8
    xs, ys = zip(*path.coords)
    assert min(xs) == pytest.approx(-0.6) and max(ys) == pytest.approx(2.6)
    signed_area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                      in zip(path.coords, list(path.coords)[1:]))
    assert signed_area > 0, "ccw: counter-clockwise"
    assert sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(
        edge_path(LAWN, 0.4, "cw").coords, list(edge_path(LAWN, 0.4, "cw").coords)[1:])) < 0


def test_a_lawn_too_narrow_for_the_offset_is_refused():
    with pytest.raises(ValueError, match="too narrow"):
        edge_path(Polygon([(0, 0), (0.6, 0), (0.6, 5), (0, 5)]), 0.4)


@pytest.mark.parametrize("lawn, start, direction", [
    (LAWN, (0.0, 0.0, 0.0), "ccw"),
    (LAWN, (0.0, 0.0, math.pi), "cw"),
    (L_SHAPE, (1.5, 1.5, 0.0), "ccw"),            # a concave corner
    (LAWN, (-3.0, 1.0, 0.0), "ccw"),              # starts OUTSIDE: approaches first
])
def test_the_expert_drives_one_lap_on_the_edge(lawn, start, direction):
    expert = EdgeExpert(lawn, ExpertConfig(offset_m=0.4), direction)
    poses = drive_unicycle(expert, start)
    assert expert.done, f"lap not finished: {expert.progress_m:.1f}/{expert.length:.1f} m"
    # Score the lap itself (from the first on-path pose), not the approach.
    on = next(i for i, (x, y, _) in enumerate(poses)
              if expert.path.distance(__import__("shapely").geometry.Point(x, y)) < 0.2)
    m = edge_metrics(lawn, 0.4, [(x, y) for x, y, _ in poses[on:]])
    assert m["error_mean"] < 0.05 and m["error_p95"] < 0.15, m
    assert m["outside"] == 0.0 and m["reached"] > 0.9, m


def test_the_lap_is_counted_in_the_driving_direction_only():
    expert = EdgeExpert(LAWN, ExpertConfig(offset_m=0.4), "ccw")
    # Standing on the path and jittering back and forth adds nothing.
    for k in range(50):
        expert.step(1.0 + (0.05 if k % 2 else -0.05), -0.6, 0.0)
    assert expert.progress_m < 0.1


def test_the_label_is_pure_and_turns_toward_the_path():
    expert = EdgeExpert(LAWN, ExpertConfig(offset_m=0.4), "ccw")
    # On the bottom edge (y = -0.6) facing east, ccw goes east: straight on.
    v, w = expert.label(1.0, -0.6, 0.0)
    assert v == pytest.approx(0.3) and abs(w) < 1e-6
    # Same place facing west: turn round on the spot.
    assert expert.label(1.0, -0.6, math.pi)[0] == 0.0
    # Inside of the path (north of it), facing east: steer right (w < 0).
    assert expert.label(1.0, -0.3, 0.0)[1] < 0
    assert expert.progress_m == 0.0, "label() never moves the lap"


def test_metrics_on_hand_made_drives():
    on_edge = [(x, -0.6) for x in [i * 0.1 for i in range(40)]]
    m = edge_metrics(LAWN, 0.4, on_edge)
    assert m["error_mean"] == pytest.approx(0.0, abs=1e-9) and m["within"] == 1.0
    assert m["outside"] == 0.0
    # The bottom edge from x = -0.5..4.4 is reached: ~4.9 of 20 m of boundary.
    assert m["reached"] == pytest.approx(4.9 / 20.0, abs=0.03)
    drifted = [(x, -0.3) for x, _ in on_edge]                     # 0.3 m too far in
    assert edge_metrics(LAWN, 0.4, drifted)["error_mean"] == pytest.approx(0.3)
    outside = [(x, -1.2) for x, _ in on_edge]
    assert edge_metrics(LAWN, 0.4, outside)["outside"] == 1.0
