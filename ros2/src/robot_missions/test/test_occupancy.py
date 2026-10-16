"""Shapes -> cells, against hand-counted cells. ROS-free.

Every expected count below is worked out in the test's comment from the
cell-centre rule, not from running the code.
"""

import pytest

from robot_missions.occupancy import (
    FREE, OCCUPIED, Circle, GridSpec, covering_grid, empty_grid, mark_circle,
    mark_polygon, occupied_cells)

# 4 x 4 m at 0.1 m, lower-left corner at (-2, -2): cell (i, j) has its centre at
# (-2 + (i + 0.5) * 0.1, -2 + (j + 0.5) * 0.1), so cell (20, 20) is centred on
# (0.05, 0.05).
SPEC = GridSpec(width=40, height=40, resolution=0.1, origin_x=-2.0, origin_y=-2.0)


def cells(grid, spec=SPEC):
    return {(i, j) for j in range(spec.height) for i in range(spec.width)
            if grid[spec.index(i, j)] == OCCUPIED}


def test_a_unit_square_occupies_exactly_its_hundred_cells():
    # Centres 0.05 .. 0.95 on both axes: cells 20..29 x 20..29.
    grid = empty_grid(SPEC)
    mark_polygon(grid, SPEC, [[(0, 0), (1, 0), (1, 1), (0, 1)]])
    assert cells(grid) == {(i, j) for i in range(20, 30) for j in range(20, 30)}


def test_winding_direction_does_not_matter():
    clockwise, counter = empty_grid(SPEC), empty_grid(SPEC)
    mark_polygon(clockwise, SPEC, [[(0, 0), (0, 1), (1, 1), (1, 0)]])
    mark_polygon(counter, SPEC, [[(0, 0), (1, 0), (1, 1), (0, 1)]])
    assert cells(clockwise) == cells(counter) and len(cells(counter)) == 100


def test_a_hole_is_left_free():
    # 2 x 2 m square (400 cells) with a 1 x 1 m hole in the middle (100 cells).
    grid = empty_grid(SPEC)
    mark_polygon(grid, SPEC, [[(0, 0), (2, 0), (2, 2), (0, 2)],
                              [(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)]])
    assert occupied_cells(grid) == 300
    # The hole's centre (1.0, 1.0) is the corner of cells 29/30; (1.05, 1.05) is
    # the centre of cell (30, 30) — inside the hole.
    assert grid[SPEC.index(30, 30)] == FREE
    # (0.25, 0.25): cell (22, 22), in the ring.
    assert grid[SPEC.index(22, 22)] == OCCUPIED


def test_a_concave_l_keeps_its_notch_free():
    # L = 2 x 2 square minus its upper-right 1 x 1 quadrant: 400 - 100 cells.
    grid = empty_grid(SPEC)
    mark_polygon(grid, SPEC, [[(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]])
    assert occupied_cells(grid) == 300
    assert grid[SPEC.index(35, 35)] == FREE          # (1.55, 1.55), the notch
    assert grid[SPEC.index(35, 25)] == OCCUPIED      # (1.55, 0.55), the foot


def test_a_triangle_by_hand():
    # (0,0), (1.02,0), (0,1.02): centre ((i+.5)/10, (j+.5)/10) is inside when
    # x + y <= 1.02, i.e. i + j <= 9.2, i.e. i + j <= 9 for i, j >= 0 —
    # 10 + 9 + ... + 1 = 55 cells. The 0.02 keeps every centre off the edge.
    grid = empty_grid(SPEC)
    mark_polygon(grid, SPEC, [[(0, 0), (1.02, 0), (0, 1.02)]])
    assert cells(grid) == {(20 + i, 20 + j) for i in range(10) for j in range(10)
                           if i + j <= 9}


def test_a_vertex_on_a_scanline_is_crossed_once():
    # A diamond centred on (0.05, 0.05) with half-diagonal 0.52: its left and
    # right corners sit EXACTLY on row 20's centre line (y = 0.05). Counting
    # both edges that meet there would toggle the fill twice and leave the
    # whole row empty. Inside: |di| + |dj| <= 5.2 -> <= 5: 2*5*5 + 2*5 + 1 = 61.
    grid = empty_grid(SPEC)
    c = 0.05
    mark_polygon(grid, SPEC, [[(c - 0.52, c), (c, c - 0.52), (c + 0.52, c), (c, c + 0.52)]])
    assert occupied_cells(grid) == 61
    assert {(i, j) for i, j in cells(grid) if j == 20} == {(i, 20) for i in range(15, 26)}


def test_a_circle_by_hand():
    # r = 0.25 about the centre of cell (20, 20). Offsets (di, dj) in cells are
    # inside when di^2 + dj^2 <= 6.25: (0,0); (±1,0),(0,±1); (±1,±1);
    # (±2,0),(0,±2); (±2,±1),(±1,±2) — 1 + 4 + 4 + 4 + 8 = 21. (±2,±2) is 8.
    grid = empty_grid(SPEC)
    mark_circle(grid, SPEC, Circle(0.05, 0.05, 0.25))
    assert cells(grid) == {(20 + di, 20 + dj) for di in range(-2, 3) for dj in range(-2, 3)
                           if di * di + dj * dj <= 6}


def test_shapes_hanging_off_the_grid_are_clipped_not_wrapped():
    grid = empty_grid(SPEC)
    mark_circle(grid, SPEC, Circle(-2.0, 0.05, 0.22))
    mark_polygon(grid, SPEC, [[(1.5, -3), (3, -3), (3, -1.5), (1.5, -1.5)]])
    occupied = cells(grid)
    # Nothing may appear on the far edge (wrapped into the previous row).
    assert all(i < 3 or i > 30 for i, _ in occupied)
    assert all(0 <= i < SPEC.width and 0 <= j < SPEC.height for i, j in occupied)
    # Circle at x = -2, r = 0.22: centres -1.95 and -1.85 are inside, -1.75
    # (0.25 away) is not -> columns 0 and 1 only.
    assert {i for i, j in occupied if j == 20 and i < 10} == {0, 1}


def test_covering_grid_by_hand():
    # Points span x -3.05 .. 12, y -4 .. 2, plus the origin; margin 1 m.
    # min x: floor(-40.5) * 0.1 = -4.1   width:  ceil((13 + 4.1) / 0.1) = 171
    # min y: floor(-50)   * 0.1 = -5.0   height: ceil((3 + 5) / 0.1)   = 80
    spec = covering_grid([(-3.05, 2.0), (12.0, -4.0)], 0.1, 1.0, 10**6)
    assert spec.origin_x == pytest.approx(-4.1)
    assert spec.origin_y == pytest.approx(-5.0)
    assert (spec.width, spec.height) == (171, 80)


def test_covering_grid_always_includes_the_map_origin():
    spec = covering_grid([(50.0, 50.0)], 1.0, 0.0, 10**6)
    assert (spec.origin_x, spec.origin_y) == (0.0, 0.0)
    assert (spec.width, spec.height) == (50, 50)


def test_an_empty_site_still_gets_a_grid_around_the_origin():
    spec = covering_grid([], 0.1, 30.0, 10**7)
    assert (spec.origin_x, spec.origin_y) == (pytest.approx(-30.0), pytest.approx(-30.0))
    assert (spec.width, spec.height) == (600, 600)


def test_a_grid_over_the_cell_budget_raises_instead_of_clipping():
    with pytest.raises(ValueError, match="max_cells"):
        covering_grid([(1000.0, 1000.0)], 0.1, 0.0, 10**6)


def test_bad_resolution_or_margin_raises():
    with pytest.raises(ValueError):
        covering_grid([], 0.0, 1.0, 10**6)
    with pytest.raises(ValueError):
        covering_grid([], 0.1, -1.0, 10**6)
