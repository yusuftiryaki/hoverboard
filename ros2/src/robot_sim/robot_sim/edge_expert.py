"""The privileged edge-following expert and the edge metric (docs/learning.md,
6a). ROS-free; sim only.

PRIVILEGED: it drives from /ground_truth and the lawn's TRUE boundary — things
no policy (and no real robot) has. That is the point. The policy learns from
camera + state; the expert supplies unlimited demonstrations, and because it
is a function of the state it can label ANY state the policy reaches
(DAgger, 6d).

The task: drive once round the inside of the lawn, base_link `offset_m` from
the boundary (the trimmer head reaches the edge from there), direction
"ccw" (boundary on the right) or "cw".

The metric is the same ground truth, so expert and policy are scored alike:
signed distance from the boundary (+ inside), its error from offset_m, how
much of the boundary was reached, how long the robot was outside.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient

XY = Tuple[float, float]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def edge_path(boundary: Polygon, offset_m: float, direction: str = "ccw") -> LineString:
    """The line base_link should follow: the boundary pulled `offset_m` in,
    ordered in the driving direction. The biggest piece if the pull splits it."""
    if direction not in ("ccw", "cw"):
        raise ValueError(f"direction is ccw or cw, got {direction!r}")
    inner = boundary.buffer(-offset_m, join_style="mitre", mitre_limit=2.0)
    if inner.is_empty:
        raise ValueError(f"the lawn is too narrow for a {offset_m} m offset")
    if inner.geom_type == "MultiPolygon":
        inner = max(inner.geoms, key=lambda g: g.area)
    inner = orient(inner, sign=1.0 if direction == "ccw" else -1.0)
    return LineString(inner.exterior.coords)


@dataclass
class ExpertConfig:
    offset_m: float = 0.4
    speed: float = 0.3            # m/s on straight edges
    lookahead: float = 0.6        # m along the path
    max_angular: float = 1.0      # rad/s
    turn_in_place_deg: float = 30.0
    corner_deg: float = 30.0      # a vertex turning more than this is a corner
    on_path_m: float = 0.2        # lap counting starts once this close


class EdgeExpert:
    """Pure pursuit on edge_path(); counts the lap by arc length."""

    def __init__(self, boundary: Polygon, config: ExpertConfig = ExpertConfig(),
                 direction: str = "ccw", laps: float = 1.0):
        self.config = config
        self.path = edge_path(boundary, config.offset_m, direction)
        self.length = self.path.length
        self.laps = laps
        # Corners: the carrot never passes one before the robot does, so the
        # robot drives INTO the corner and turns on the spot (a differential
        # drive can) instead of cutting it — pure pursuit alone left 16 % of
        # a rectangle's edge unreached (test_edge_expert).
        self.corners = self._corners(config.corner_deg)
        self._last_s: Optional[float] = None
        self.progress_m = 0.0          # along the path, in the driving direction

    def _corners(self, corner_deg: float) -> List[float]:
        coords = list(self.path.coords)[:-1]
        out = []
        for i, here in enumerate(coords):
            before, after = coords[i - 1], coords[(i + 1) % len(coords)]
            a = math.atan2(here[1] - before[1], here[0] - before[0])
            b = math.atan2(after[1] - here[1], after[0] - here[0])
            if abs(_wrap(b - a)) > math.radians(corner_deg):
                out.append(self.path.project(Point(here)))
        return sorted(out)

    def _carrot(self, s: float) -> float:
        """Arc length of the pursuit target: lookahead ahead of s, but not
        past the next corner (unless the robot is already at it)."""
        ahead = self.config.lookahead
        for corner in self.corners:
            gap = (corner - s) % self.length
            if 0.05 < gap < ahead:
                ahead = gap
        return (s + ahead) % self.length

    @property
    def done(self) -> bool:
        return self.progress_m >= self.laps * self.length

    def label(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """What the expert would command at this pose. Pure: no lap state —
        this is the DAgger label for any state the policy reaches."""
        c = self.config
        here = Point(x, y)
        s = self.path.project(here)
        target = self.path.interpolate(self._carrot(s))
        heading = _wrap(math.atan2(target.y - y, target.x - x) - yaw)
        if abs(heading) > math.radians(c.turn_in_place_deg):
            return 0.0, math.copysign(c.max_angular, heading)
        distance = max(1e-6, math.hypot(target.x - x, target.y - y))
        v = c.speed * max(0.3, math.cos(heading))
        w = 2.0 * v * math.sin(heading) / distance          # pure pursuit curvature
        return v, max(-c.max_angular, min(c.max_angular, w))

    def step(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """label() plus the lap count; (0, 0) once the laps are done."""
        here = Point(x, y)
        s = self.path.project(here)
        if self.path.distance(here) < self.config.on_path_m:
            if self._last_s is not None:
                # Signed: jitter back and forth nets zero.
                ds = (s - self._last_s + self.length / 2) % self.length - self.length / 2
                self.progress_m += ds
            self._last_s = s
        if self.done:
            return 0.0, 0.0
        return self.label(x, y, yaw)


def signed_distance(boundary: Polygon, x: float, y: float) -> float:
    """Distance to the boundary line, + inside the lawn, - outside."""
    p = Point(x, y)
    d = boundary.exterior.distance(p)
    return d if boundary.contains(p) else -d


def edge_metrics(boundary: Polygon, offset_m: float, poses: Sequence[XY],
                 tolerance_m: float = 0.1, reach_step_m: float = 0.1) -> Dict[str, float]:
    """How well a drive followed the edge. poses: base_link (x, y), in order.

    error        |signed distance - offset| per pose: mean, p95, max
    within       share of poses with error <= tolerance
    outside      share of poses outside the lawn (signed distance < 0)
    reached      share of the BOUNDARY within offset + tolerance of the drive —
                 the edge actually cut, not just the poses being good
    """
    if len(poses) < 2:
        raise ValueError("need a drive, not a point")
    d = np.array([signed_distance(boundary, x, y) for x, y in poses])
    err = np.abs(d - offset_m)
    track = LineString(poses)
    ring = boundary.exterior
    n = max(4, int(ring.length / reach_step_m))
    samples = [ring.interpolate(i / n, normalized=True) for i in range(n)]
    reached = np.mean([track.distance(p) <= offset_m + tolerance_m for p in samples])
    return {"error_mean": float(err.mean()), "error_p95": float(np.percentile(err, 95)),
            "error_max": float(err.max()), "within": float(np.mean(err <= tolerance_m)),
            "outside": float(np.mean(d < 0)), "reached": float(reached),
            "poses": len(poses)}


def drive_unicycle(expert: EdgeExpert, start: Tuple[float, float, float], dt: float = 0.05,
                   max_s: float = 600.0) -> List[Tuple[float, float, float]]:
    """Integrate the expert's commands on an ideal unicycle (tests, quick
    checks). Stops when the lap is done or after max_s."""
    x, y, yaw = start
    poses = [(x, y, yaw)]
    for _ in range(int(max_s / dt)):
        v, w = expert.step(x, y, yaw)
        if expert.done:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = _wrap(yaw + w * dt)
        poses.append((x, y, yaw))
    return poses
