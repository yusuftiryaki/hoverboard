"""Obstacle geometry in `map` metres -> OccupancyGrid cells. ROS-free.

Same conventions as robot_sim/obstacle_map.py, on purpose — the simulator's map
stands in for this one in every Nav2 test, and test_site_map_matches_sim (stack)
holds the two to the same cells:

⚠️ THE GRID HOLDS THE BARE OBSTACLE, NOT THE NO-GO ZONE. The robot's radius is
the costmap inflation layer's job (nav2.yaml); marking it here too would inflate
twice and the robot would refuse gaps it fits through.

⚠️ CELLS ARE TESTED AT THEIR CENTRE. A cell is occupied when its centre is inside
the shape, so the marked area under-covers the true outline by up to half a
cell at the rim. The safety margin belongs to inflation, in one place.
"""

from __future__ import annotations

import math
from array import array
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

FREE = 0
OCCUPIED = 100

Point = Tuple[float, float]
Ring = Sequence[Point]


@dataclass(frozen=True)
class GridSpec:
    width: int                  # cells
    height: int                 # cells
    resolution: float           # m per cell
    origin_x: float             # map x of the grid's lower-left CORNER, m
    origin_y: float             # map y of the grid's lower-left CORNER, m

    def cell_center(self, grid_x: int, grid_y: int) -> Point:
        return (self.origin_x + (grid_x + 0.5) * self.resolution,
                self.origin_y + (grid_y + 0.5) * self.resolution)

    def index(self, grid_x: int, grid_y: int) -> int:
        """Row-major, x fastest — OccupancyGrid.data's order."""
        return grid_y * self.width + grid_x

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


def covering_grid(points: Iterable[Point], resolution: float, margin: float,
                  max_cells: int) -> GridSpec:
    """The smallest grid holding every point plus `margin` on each side.

    The margin is there for the global costmap's ROLLING window (60 x 60 m in
    nav2.yaml): it follows the robot, and wherever it hangs past the map the
    static layer has nothing to say. With margin = half the window, a robot
    anywhere inside the site sees the map under its whole window.

    The origin (0, 0) is always included — it is where the robot was when the
    map frame was made.

    ⚠️ Raises instead of shrinking or coarsening on its own: a map silently
    clipped to max_cells would drop the keep-out zones beyond the clip.
    """
    if resolution <= 0.0 or margin < 0.0:
        raise ValueError(f"resolution must be > 0 and margin >= 0, got {resolution}, {margin}")
    xs, ys = [0.0], [0.0]
    for x, y in points:
        xs.append(x)
        ys.append(y)
    # Snapped outward to whole cells so a point exactly on a cell boundary
    # cannot land in a cell the grid does not have. The 1e-9 keeps float noise
    # from adding a whole extra row: (3.0 + 5.0) / 0.1 is 80.00000000000001.
    min_x = math.floor((min(xs) - margin) / resolution + 1e-9) * resolution
    min_y = math.floor((min(ys) - margin) / resolution + 1e-9) * resolution
    width = int(math.ceil((max(xs) + margin - min_x) / resolution - 1e-9))
    height = int(math.ceil((max(ys) + margin - min_y) / resolution - 1e-9))
    if width * height > max_cells:
        raise ValueError(
            f"the site needs a {width} x {height} grid ({width * height} cells) at "
            f"{resolution} m, over max_cells={max_cells}. Raise the resolution "
            "(coarser cells) or max_cells — or check for a stray far-away point.")
    return GridSpec(width=width, height=height, resolution=resolution,
                    origin_x=min_x, origin_y=min_y)


def empty_grid(spec: GridSpec) -> array:
    return array("b", bytes(spec.width * spec.height))


def mark_circle(grid: array, spec: GridSpec, circle: Circle) -> None:
    """Occupy every cell whose centre is within the circle. Same test as
    robot_sim.obstacle_map.build_obstacle_grid."""
    min_x, min_y = spec.cell_of(circle.x - circle.radius, circle.y - circle.radius)
    max_x, max_y = spec.cell_of(circle.x + circle.radius, circle.y + circle.radius)
    for grid_y in range(max(0, min_y), min(spec.height - 1, max_y) + 1):
        for grid_x in range(max(0, min_x), min(spec.width - 1, max_x) + 1):
            cx, cy = spec.cell_center(grid_x, grid_y)
            if math.hypot(cx - circle.x, cy - circle.y) <= circle.radius:
                grid[spec.index(grid_x, grid_y)] = OCCUPIED


def mark_polygon(grid: array, spec: GridSpec, rings: Sequence[Ring]) -> None:
    """Occupy every cell whose centre is inside the polygon (even-odd rule).

    `rings` is the exterior followed by its holes, each implicitly closed. With
    even-odd, a hole needs no special casing and neither does orientation —
    GeoJSON's right-hand rule is a recommendation, and a hand-written or
    UI-drawn ring can wind either way.

    Scanline: per cell row, the x where each edge crosses the row's centre line,
    sorted, filled pairwise. An edge counts on [lower y, upper y) so a vertex
    lying exactly on the scanline is crossed once, not twice.
    """
    edges = []
    for ring in rings:
        for i, (x0, y0) in enumerate(ring):
            x1, y1 = ring[(i + 1) % len(ring)]
            if y0 != y1:
                edges.append((x0, y0, x1, y1))
    if not edges:
        return
    all_y = [y for e in edges for y in (e[1], e[3])]
    _, row_lo = spec.cell_of(0.0, min(all_y))
    _, row_hi = spec.cell_of(0.0, max(all_y))
    for grid_y in range(max(0, row_lo), min(spec.height - 1, row_hi) + 1):
        _, cy = spec.cell_center(0, grid_y)
        crossings = []
        for x0, y0, x1, y1 in edges:
            if (y0 <= cy < y1) or (y1 <= cy < y0):
                crossings.append(x0 + (cy - y0) * (x1 - x0) / (y1 - y0))
        crossings.sort()
        for left, right in zip(crossings[0::2], crossings[1::2]):
            # Cell centres strictly within [left, right].
            first = int(math.ceil((left - spec.origin_x) / spec.resolution - 0.5))
            last = int(math.floor((right - spec.origin_x) / spec.resolution - 0.5))
            for grid_x in range(max(0, first), min(spec.width - 1, last) + 1):
                grid[spec.index(grid_x, grid_y)] = OCCUPIED


def occupied_cells(grid: Sequence[int]) -> int:
    return sum(1 for value in grid if value == OCCUPIED)
