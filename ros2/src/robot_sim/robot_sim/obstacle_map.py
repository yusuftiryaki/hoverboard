"""Obstacle geometry as an OccupancyGrid. ROS-free, deterministic.

This turns the same obstacles KinematicWorld collides against into the cell
grid a costmap can read. The emphasis is on *same*: sim_node parses the
obstacle parameters once, here, and hands the resulting tuple to both the world
and the map. A6 was caused by one model of the world living in two files that
quietly drifted apart, and an obstacle the map shows but the physics does not
have (or the reverse) is that same bug wearing a different hat — the robot
would swerve around a phantom, or drive into something invisible, and every
test would stay green.

⚠️ WHAT THE MAP HOLDS IS THE BARE OBSTACLE, NOT THE NO-GO ZONE. The world stops
the robot at `robot_radius + obstacle.radius`; the grid marks only cells inside
`obstacle.radius`. The missing robot radius is the costmap inflation layer's
job (nav2.yaml, inflation_radius 0.55). Marking the inflated circle here would
inflate it twice and the robot would refuse gaps it fits through.

⚠️ Cells are tested at their CENTRE, not their corners. A cell is occupied when
its centre lies within the radius, so the marked area under-covers the true
circle by up to half a cell at the rim. Deliberate, same reason: the safety
margin belongs to inflation, in one place, where it is tuned.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from robot_sim.world import CircularObstacle

FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class GridSpec:
    """Geometry of the published grid — the single source for both the cell
    maths here and the OccupancyGrid.info sim_node fills in."""

    width: int = 600            # cells
    height: int = 600           # cells
    resolution: float = 0.1     # m per cell
    origin_x: float = -30.0     # world x of the grid's lower-left CORNER, m
    origin_y: float = -30.0     # world y of the grid's lower-left CORNER, m

    def cell_center(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """World coordinates of a cell's centre — hence the +0.5."""
        return (
            self.origin_x + (grid_x + 0.5) * self.resolution,
            self.origin_y + (grid_y + 0.5) * self.resolution,
        )

    def index(self, grid_x: int, grid_y: int) -> int:
        """Row-major, the order OccupancyGrid.data is defined in: x varies
        fastest. Transposing this is invisible on a square grid with a
        symmetric obstacle, which is why the tests use neither."""
        return grid_y * self.width + grid_x


# 60 x 60 m at 0.1 m/cell, deliberately matching global_costmap's rolling
# window in nav2.yaml — a map smaller than the window would leave the edges
# unexplained, and a larger one is cells nobody reads.
DEFAULT_GRID = GridSpec()


def parse_obstacle_params(
    centers: Sequence[float], radii: Sequence[float]
) -> Tuple[CircularObstacle, ...]:
    """Flat ROS params -> obstacles. `centers` is [x0, y0, x1, y1, ...].

    ROS parameters cannot hold a list of tuples, hence the flattening. Every
    mismatch raises instead of truncating: a half-read obstacle list would put
    the map and the physics in different places, which is exactly the failure
    A6 taught us to make loud rather than plausible.
    """
    if len(centers) % 2 != 0:
        raise ValueError(
            f"obstacle_centers must be x/y pairs, got {len(centers)} values"
        )
    if len(radii) != len(centers) // 2:
        raise ValueError(
            f"obstacle_centers describes {len(centers) // 2} obstacle(s) but "
            f"obstacle_radii has {len(radii)}"
        )
    if any(radius <= 0.0 for radius in radii):
        raise ValueError(f"obstacle radii must be positive, got {list(radii)}")
    return tuple(
        CircularObstacle(
            x=float(centers[i]), y=float(centers[i + 1]), radius=float(radii[i // 2])
        )
        for i in range(0, len(centers), 2)
    )


def build_obstacle_grid(
    obstacles: Sequence[CircularObstacle], spec: GridSpec = DEFAULT_GRID
) -> list[int]:
    """Return row-major occupancy values (0 free, 100 occupied)."""
    grid = [FREE] * (spec.width * spec.height)
    for obstacle in obstacles:
        # Only sweep the obstacle's bounding box; 360 000 cells per obstacle
        # would be silly. Clamped to the grid so an obstacle that hangs over
        # the edge is clipped rather than wrapping into the opposite row.
        min_x = max(0, int(math.floor(
            (obstacle.x - obstacle.radius - spec.origin_x) / spec.resolution)))
        max_x = min(spec.width - 1, int(math.ceil(
            (obstacle.x + obstacle.radius - spec.origin_x) / spec.resolution)))
        min_y = max(0, int(math.floor(
            (obstacle.y - obstacle.radius - spec.origin_y) / spec.resolution)))
        max_y = min(spec.height - 1, int(math.ceil(
            (obstacle.y + obstacle.radius - spec.origin_y) / spec.resolution)))
        for grid_y in range(min_y, max_y + 1):
            for grid_x in range(min_x, max_x + 1):
                world_x, world_y = spec.cell_center(grid_x, grid_y)
                if math.hypot(world_x - obstacle.x, world_y - obstacle.y) <= obstacle.radius:
                    grid[spec.index(grid_x, grid_y)] = OCCUPIED
    return grid
