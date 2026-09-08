"""Obstacle grid tests — no ROS, no hardware.

The grid is a PICTURE of the world the robot physically collides against. If
the picture is mirrored, transposed or offset, Nav2 plans around a phantom and
drives into the real thing — and every downstream test still passes, because
they all grade against the same wrong picture. That is A6 exactly.

So, per the two rules A6 bought us:
  * the occupied cells are pinned to indices worked out BY HAND, not to a
    second implementation of the same formula, and
  * placement is SWEPT around the circle rather than checked at one spot,
    because a mirror and a transpose both look perfect on the x axis.

The grids here are 6x4, not square, and the obstacles are off-centre. A square
grid with a centred obstacle is invariant under exactly the bugs being hunted.
"""

import math

import pytest

from robot_sim.obstacle_map import (
    DEFAULT_GRID,
    FREE,
    OCCUPIED,
    GridSpec,
    build_obstacle_grid,
    parse_obstacle_params,
)
from robot_sim.world import CircularObstacle, KinematicWorld

# 6 cells east-west, 4 north-south, 1 m each. Cell centres are therefore
# x = -2.5 -1.5 -0.5 0.5 1.5 2.5   and   y = -1.5 -0.5 0.5 1.5
SMALL = GridSpec(width=6, height=4, resolution=1.0, origin_x=-3.0, origin_y=-2.0)

RPM_FOR_1MS = 60.0 / (2.0 * math.pi * 0.0825)


def occupied_indices(grid):
    return {i for i, value in enumerate(grid) if value == OCCUPIED}


def test_cell_centers_are_hand_computed():
    # Corner cell: origin + half a cell.
    assert SMALL.cell_center(0, 0) == pytest.approx((-2.5, -1.5))
    # Opposite corner: 5 and 3 are the last valid indices of 6 and 4.
    assert SMALL.cell_center(5, 3) == pytest.approx((2.5, 1.5))


def test_grid_matches_hand_computed_cells():
    """A r=1.0 circle at (0.5, 0.5) on a 1 m grid marks a plus, not a square.

    Worked out by hand: cell centres sit at whole ±0.5 values, so the four
    diagonal neighbours are hypot(1, 1) = 1.414 m away and fall outside r=1.0,
    while the four edge neighbours are exactly 1.0 m away and fall inside
    (the test is <=). Occupied cells, as (grid_x, grid_y):
        (3,2) centre       -> index 2*6+3 = 15
        (2,2) west         -> index 2*6+2 = 14
        (4,2) east         -> index 2*6+4 = 16
        (3,1) south        -> index 1*6+3 =  9
        (3,3) north        -> index 3*6+3 = 21
    """
    grid = build_obstacle_grid([CircularObstacle(x=0.5, y=0.5, radius=1.0)], SMALL)

    assert len(grid) == 24
    assert occupied_indices(grid) == {9, 14, 15, 16, 21}
    assert grid.count(FREE) == 19


def test_data_is_row_major_with_x_varying_fastest():
    """Pin the OccupancyGrid index convention with a single off-axis cell.

    The obstacle sits on the centre of the south-east cell (2.5, -1.5) =
    (grid_x 5, grid_y 0), whose row-major index is 0*6 + 5 = 5. Transposing
    the index would give 5*6 + 0 = 30, which is off the end of a 24-cell grid —
    the whole point of using a 6x4 grid rather than a square one.
    """
    grid = build_obstacle_grid([CircularObstacle(x=2.5, y=-1.5, radius=0.4)], SMALL)

    assert occupied_indices(grid) == {5}


def test_placement_sweeps_the_full_circle():
    """Eight placements around the origin, each pinned to a hand-read cell.

    A mirrored y, a swapped x/y and a sign error on the origin each survive a
    single placement on the x axis. None survive a full turn: this walks an
    obstacle around all eight neighbours of the cell containing (0.5, 0.5) and
    demands the marked cell be the one in that compass direction.
    """
    spec = SMALL
    center_cell = (3, 2)                      # holds world (0.5, 0.5)
    # (bearing from east, CCW) -> (grid_x, grid_y) step. y grows north, so
    # north is +grid_y: getting this table wrong is the bug being hunted.
    sweep = {
        0: (1, 0),      # east
        45: (1, 1),
        90: (0, 1),     # north
        135: (-1, 1),
        180: (-1, 0),   # west
        225: (-1, -1),
        270: (0, -1),   # south
        315: (1, -1),
    }
    for degrees, (step_x, step_y) in sweep.items():
        radians = math.radians(degrees)
        # One cell away from (0.5, 0.5) in that direction, landing exactly on
        # the neighbouring cell's centre. Radius 0.4 < half a cell, so it can
        # only ever mark the one cell it sits in.
        world_x = 0.5 + math.cos(radians) * (1.0 if degrees % 90 == 0 else math.sqrt(2.0))
        world_y = 0.5 + math.sin(radians) * (1.0 if degrees % 90 == 0 else math.sqrt(2.0))
        grid = build_obstacle_grid(
            [CircularObstacle(x=world_x, y=world_y, radius=0.4)], spec
        )
        expected = spec.index(center_cell[0] + step_x, center_cell[1] + step_y)
        assert occupied_indices(grid) == {expected}, f"{degrees}° yanlış hücreyi işaretledi"


def test_obstacles_off_the_edge_are_clipped_not_wrapped():
    """Half an obstacle hanging off the west edge must not appear in the east.

    grid_x would be -1 for the cells past the edge; unclamped, index() turns
    that into the LAST cell of the row below — an obstacle teleported across
    the map. Only (0,2) is genuinely in range: its centre (-2.5, 0.5) is 0.5 m
    from the obstacle, while (0,1) and (0,3) are hypot(0.5, 1.0) = 1.118 away.
    """
    grid = build_obstacle_grid([CircularObstacle(x=-3.0, y=0.5, radius=1.0)], SMALL)

    assert occupied_indices(grid) == {SMALL.index(0, 2)}
    assert grid[SMALL.index(5, 1)] == FREE       # the cell a wrap would hit

    # Entirely past the east edge: nothing marked, and no IndexError.
    assert occupied_indices(build_obstacle_grid(
        [CircularObstacle(x=10.0, y=0.5, radius=1.0)], SMALL)) == set()


def test_empty_obstacle_list_is_all_free():
    grid = build_obstacle_grid([], SMALL)
    assert set(grid) == {FREE}
    assert len(grid) == 24


def test_default_grid_covers_the_nav2_global_costmap_window():
    """nav2.yaml's global costmap is a 60 x 60 m rolling window at 0.1 m. A map
    smaller than that leaves costmap edges unexplained."""
    assert DEFAULT_GRID.width * DEFAULT_GRID.resolution == pytest.approx(60.0)
    assert DEFAULT_GRID.height * DEFAULT_GRID.resolution == pytest.approx(60.0)
    assert DEFAULT_GRID.cell_center(0, 0) == pytest.approx((-29.95, -29.95))


# ---- parameters ----------------------------------------------------------
def test_parse_builds_the_same_objects_the_world_collides_against():
    obstacles = parse_obstacle_params([1.0, 2.0, -3.0, 4.5], [0.2, 0.7])

    assert obstacles == (
        CircularObstacle(x=1.0, y=2.0, radius=0.2),
        CircularObstacle(x=-3.0, y=4.5, radius=0.7),
    )


def test_parse_rejects_lists_that_do_not_line_up():
    with pytest.raises(ValueError, match="x/y pairs"):
        parse_obstacle_params([1.0, 2.0, 3.0], [0.2])
    with pytest.raises(ValueError, match="obstacle_radii"):
        parse_obstacle_params([1.0, 2.0, 3.0, 4.0], [0.2])
    with pytest.raises(ValueError, match="positive"):
        parse_obstacle_params([1.0, 2.0], [0.0])


def test_parse_of_empty_params_is_an_empty_world():
    assert parse_obstacle_params([], []) == ()


# ---- the map and the physics must describe ONE world ---------------------
def test_map_marks_the_bare_obstacle_and_the_world_stops_a_robot_radius_short():
    """The two halves of A3c, checked against each other.

    The world stops the robot at robot_radius + obstacle radius; the map marks
    only the obstacle itself. So the robot comes to rest OUTSIDE every occupied
    cell, with the gap it leaves being the robot radius — which is the costmap
    inflation layer's job to add, in one place, rather than being baked into
    the map twice.
    """
    obstacle = CircularObstacle(x=1.0, y=0.0, radius=0.2)
    spec = GridSpec(width=40, height=40, resolution=0.05, origin_x=-1.0, origin_y=-1.0)
    grid = build_obstacle_grid([obstacle], spec)

    world = KinematicWorld(tau=1e-9, obstacles=(obstacle,), robot_radius=0.3)
    for _ in range(200):
        world.step(RPM_FOR_1MS, RPM_FOR_1MS, 0.02)

    assert world.collision
    # Hand-computed: contact at x = 1.0 - (0.3 + 0.2) = 0.5.
    assert world.pose.x == pytest.approx(0.5, abs=0.03)

    stopped_x = int((world.pose.x - spec.origin_x) / spec.resolution)
    stopped_y = int((world.pose.y - spec.origin_y) / spec.resolution)
    assert grid[spec.index(stopped_x, stopped_y)] == FREE

    # Every occupied cell centre really is inside the bare obstacle, never the
    # inflated one — the map must not pre-inflate.
    for index in occupied_indices(grid):
        cell_x, cell_y = spec.cell_center(index % spec.width, index // spec.width)
        distance = math.hypot(cell_x - obstacle.x, cell_y - obstacle.y)
        assert distance <= obstacle.radius
        assert distance < world.robot_radius + obstacle.radius
