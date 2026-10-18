"""Teach by driving (docs/missions.md 5b): what the robot drives becomes site
geometry. ROS-free; web_ui feeds it on every /gps/filtered fix.

    trace   area / keepout / route: drive it. The SHAPE is taken in `odom`
            (wheels + IMU: smooth, ~1 % per metre); a pose is kept every
            spacing_m or after a turn of turn_deg, "corner" pins the current
            one so simplification keeps it. finish() simplifies
            (Douglas–Peucker), closes a polygon's ring, PLACES the shape with
            map->odom averaged over the whole drive, and converts to lat/lon
            through the caller's to_ll (navsat_transform's /toLL — decision 4).
    point   station / poi / obstacle: stand still, average `seconds` of
            /gps/filtered fixes (and, for a station, the heading).

⚠️ WHY THE TRACE IS NOT JUST THE FIXES. It was, first. With the simulator's
NEO-6M wander the fixes zig-zagged 1-2 m, the 6 x 4 m lawn crossed itself and
start and end were 3 m apart — unusable. /gps/filtered is map->odom (GPS,
jumping) composed with odom->base_link (smooth). Keeping the two apart gives
the shape odometry's quality and the placement the GPS error averaged over the
drive. Averaging removes jitter, not drift (decision 1): the whole lawn may
still sit a few metres off — but as one rigid piece, not a crumpled one.

What comes out is a CANDIDATE GeoJSON feature without a name. It is never
written to the site here: the page shows it, the person names and corrects it,
and "Sahayı kaydet" sends it through the one validated save path (decision 3).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Polygon as ShapelyPolygon

from robot_missions.site import EARTH_RADIUS_M, LatLon

TRACE_KINDS = ("area", "keepout", "route")
POINT_KINDS = ("station", "poi", "obstacle")


class TeachError(ValueError):
    """The request or the recording cannot make a feature (400/409)."""


def compass_from_yaw(yaw_rad: float) -> float:
    """map's yaw (ENU: 0 = east, counter-clockwise) → compass bearing
    (0 = north, clockwise), in [0, 360). ⚠️ A6 lived in exactly this kind of
    sign: test it all the way round, never at one heading."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def circular_mean_deg(bearings: Sequence[float]) -> Optional[float]:
    if not bearings:
        return None
    s = sum(math.sin(math.radians(b)) for b in bearings)
    c = sum(math.cos(math.radians(b)) for b in bearings)
    if math.hypot(s, c) < 1e-9 * len(bearings):
        return None                       # opposite headings: no mean
    return math.degrees(math.atan2(s, c)) % 360.0


def bearing_difference(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


class _Plane:
    """Equirectangular metres around one origin — for a point's spread only;
    no position is ever converted with it (decision 4)."""

    def __init__(self, origin: LatLon):
        self.origin = origin
        self.k = math.cos(math.radians(origin.lat))

    def xy(self, p: LatLon) -> Tuple[float, float]:
        return (math.radians(p.lon - self.origin.lon) * EARTH_RADIUS_M * self.k,
                math.radians(p.lat - self.origin.lat) * EARTH_RADIUS_M)


XY = Tuple[float, float]
Pose2 = Tuple[float, float, float]          # x, y, yaw (rad)
# map -> lat/lon for a list of points; None when navsat_transform is not ready.
ToLL = Callable[[Sequence[XY]], Optional[List[LatLon]]]


def simplify(points: Sequence[XY], pinned: Sequence[bool], tolerance_m: float) -> List[XY]:
    """Douglas–Peucker in metres, keeping both ends and every pinned point."""
    if len(points) <= 2:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    for i, pin in enumerate(pinned):
        keep[i] = keep[i] or pin
    anchors = [i for i, k in enumerate(keep) if k]
    for start, end in zip(anchors, anchors[1:]):
        _dp(points, start, end, tolerance_m, keep)
    return [p for p, k in zip(points, keep) if k]


def _dp(xy: Sequence[XY], start: int, end: int, tolerance: float, keep: List[bool]) -> None:
    stack = [(start, end)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        (ax, ay), (bx, by) = xy[a], xy[b]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        worst, worst_d = None, tolerance
        for i in range(a + 1, b):
            px, py = xy[i]
            if length < 1e-9:
                d = math.hypot(px - ax, py - ay)
            else:
                d = abs(dx * (ay - py) - dy * (ax - px)) / length
            if d > worst_d:
                worst, worst_d = i, d
        if worst is not None:
            keep[worst] = True
            stack.extend(((a, worst), (worst, b)))


def mean_pose(poses: Sequence[Pose2]) -> Pose2:
    yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    return (sum(p[0] for p in poses) / len(poses), sum(p[1] for p in poses) / len(poses), yaw)


def apply(pose: Pose2, point: XY) -> XY:
    """pose ∘ point: a point in the child frame, in the parent frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * point[0] - s * point[1], pose[1] + s * point[0] + c * point[1])


@dataclass
class _Average:
    seconds: float
    started: Optional[float] = None
    fixes: List[LatLon] = field(default_factory=list)
    headings: List[float] = field(default_factory=list)

    def add(self, fix: LatLon, heading: Optional[float], t: float) -> None:
        if self.started is None:
            self.started = t
        self.fixes.append(fix)
        if heading is not None:
            self.headings.append(heading)

    def done(self, t: float, min_fixes: int) -> bool:
        return (self.started is not None and t - self.started >= self.seconds
                and len(self.fixes) >= min_fixes)

    def progress(self, t: Optional[float]) -> float:
        if self.started is None or t is None:
            return 0.0
        return min(1.0, (t - self.started) / self.seconds)

    def result(self) -> Tuple[LatLon, float, Optional[float]]:
        """Mean position, RMS spread (m) about it, mean heading."""
        mean = LatLon(sum(p.lat for p in self.fixes) / len(self.fixes),
                      sum(p.lon for p in self.fixes) / len(self.fixes))
        plane = _Plane(mean)
        spread = math.sqrt(sum(x * x + y * y for x, y in map(plane.xy, self.fixes))
                           / len(self.fixes))
        return mean, spread, circular_mean_deg(self.headings)


class Teacher:
    """One recording at a time. Thread-safe: HTTP threads command it, the ROS
    callback thread feeds it."""

    def __init__(self, spacing_m: float = 0.5, turn_deg: float = 15.0,
                 tolerance_m: float = 0.3, min_fixes: int = 5,
                 close_within_m: float = 1.0):
        self.spacing_m = spacing_m
        self.turn_deg = turn_deg
        self.tolerance_m = tolerance_m
        self.min_fixes = min_fixes
        self.close_within_m = close_within_m
        self._lock = threading.Lock()
        self._reset()
        self._result: Optional[Dict[str, Any]] = None
        self._seq = 0

    def _reset(self) -> None:
        self._generation = getattr(self, "_generation", 0) + 1
        self._mode: Optional[str] = None          # "trace" | "point"
        self._kind: Optional[str] = None
        self._odom: List[XY] = []                 # the trace's shape
        self._pinned: List[bool] = []
        self._shown: List[LatLon] = []            # the fixes then — display only
        self._last_yaw: Optional[float] = None
        self._latest: Optional[XY] = None         # where the robot is now
        self._anchors: List[Pose2] = []           # map->odom, every fix
        self._no_tf = 0
        self._pin_next = False
        self._average: Optional[_Average] = None
        self._now: Optional[float] = None

    # --- commands (HTTP threads) ----------------------------------------------

    def start_trace(self, kind: Any) -> None:
        if kind not in TRACE_KINDS:
            raise TeachError(f"a trace is one of {TRACE_KINDS}, got {kind!r}")
        with self._lock:
            self._idle_or_raise()
            self._reset()
            self._mode, self._kind = "trace", kind

    def start_point(self, kind: Any, seconds: Any) -> None:
        if kind not in POINT_KINDS:
            raise TeachError(f"a point is one of {POINT_KINDS}, got {kind!r}")
        seconds = self._seconds(seconds)
        with self._lock:
            self._idle_or_raise()
            self._reset()
            self._mode, self._kind = "point", kind
            self._average = _Average(seconds)

    def pin_corner(self) -> None:
        """The next pose becomes a vertex whatever simplification thinks."""
        with self._lock:
            if self._mode != "trace":
                raise TeachError("a corner belongs to a trace: start one first")
            self._pin_next = True

    def finish(self, to_ll: ToLL) -> Dict[str, Any]:
        """End a trace; its candidate becomes the result (also returned).
        A refused trace (crossing itself, too small) keeps recording.

        ⚠️ to_ll is called OUTSIDE the lock. It is a ROS service call, and the
        executor thread that would deliver its answer is the one feeding fixes
        here — holding the lock across it deadlocked until the call timed out
        (every finish on the real stack: "navsat cannot convert")."""
        with self._lock:
            if self._mode != "trace":
                raise TeachError("no trace is being recorded")
            points, quality = self._trace_shape()
            kind, generation = self._kind, self._generation
        lat_lon = to_ll(points)
        with self._lock:
            if self._generation != generation:
                raise TeachError("the recording was cancelled while converting")
            if lat_lon is None or len(lat_lon) != len(points):
                raise TeachError("navsat_transform cannot convert map -> lat/lon yet")
            coords = [[p.lon, p.lat] for p in lat_lon]
            if kind == "route":
                geometry = {"type": "LineString", "coordinates": coords}
            else:
                geometry = {"type": "Polygon", "coordinates": [coords + [coords[0]]]}
            self._reset()
            return self._publish({"type": "Feature", "properties": {"kind": kind},
                                  "geometry": geometry, "quality": quality})

    def cancel(self) -> None:
        with self._lock:
            self._reset()

    def take(self) -> Optional[Dict[str, Any]]:
        """The finished candidate, once: the page that takes it adds it."""
        with self._lock:
            result, self._result = self._result, None
            return result

    # --- the robot (ROS callback thread) -------------------------------------

    def feed(self, fix: LatLon, heading_deg: Optional[float], t: float,
             odom: Optional[Pose2] = None, map_odom: Optional[Pose2] = None) -> None:
        """fix/heading: /gps/filtered and map's compass. odom: base_link in
        odom; map_odom: the map->odom transform — both at this fix, None when
        tf has none (a trace then skips the fix)."""
        with self._lock:
            if self._mode is None:
                return
            self._now = t
            if self._mode == "point":
                self._average.add(fix, heading_deg, t)
                if self._average.done(t, self.min_fixes):
                    self._point_done()
                return
            if odom is None or map_odom is None:
                self._no_tf += 1
                return
            self._anchors.append(map_odom)
            self._sample(fix, odom)

    def _sample(self, fix: LatLon, odom: Pose2) -> None:
        self._latest = (odom[0], odom[1])
        pin, self._pin_next = self._pin_next, False
        if self._odom and not pin:
            last = self._odom[-1]
            moved = math.hypot(odom[0] - last[0], odom[1] - last[1])
            turned = bearing_difference(math.degrees(odom[2]),
                                        math.degrees(self._last_yaw)) >= self.turn_deg
            # A turn on the spot is not a vertex: it would pile points up.
            if moved < self.spacing_m and not (turned and moved >= self.spacing_m / 5):
                return
        self._odom.append((odom[0], odom[1]))
        self._pinned.append(pin)
        self._shown.append(fix)
        self._last_yaw = odom[2]

    def _point_done(self) -> None:
        mean, spread, heading = self._average.result()
        count = len(self._average.fixes)
        properties: Dict[str, Any] = {"kind": self._kind}
        if self._kind == "station":
            if heading is None:
                self._reset()
                self._publish({"error": "no heading while averaging (is map->base_link up?)"})
                return
            # % after round: 359.96 rounds to 360.0, which the site rejects.
            properties["heading_deg"] = round(heading, 1) % 360.0
        if self._kind == "obstacle":
            properties["radius_m"] = 0.3
        self._reset()
        self._publish({"type": "Feature", "properties": properties,
                       "geometry": {"type": "Point", "coordinates": [mean.lon, mean.lat]},
                       "quality": {"fixes": count, "spread_m": round(spread, 3)}})

    # --- helpers ---------------------------------------------------------------

    def _trace_shape(self) -> Tuple[List[XY], Dict[str, Any]]:
        """The simplified, validated shape placed in `map`, and its quality."""
        shape, pinned = list(self._odom), list(self._pinned)
        # The trace ends where the robot stopped, not at the last spaced sample.
        if self._latest is not None and shape and self._latest != shape[-1]:
            shape.append(self._latest)
            pinned.append(False)
        points = simplify(shape, pinned, self.tolerance_m)
        if self._kind == "route":
            if len(points) < 2:
                raise TeachError("a route needs at least two points — drive further")
        else:
            # Driven back to the start: the last vertex is the first again.
            if (len(points) > 3 and math.hypot(points[-1][0] - points[0][0],
                                               points[-1][1] - points[0][1])
                    < self.close_within_m):
                points = points[:-1]
            if len(points) < 3:
                raise TeachError("an area needs at least three corners — drive round it")
            shape = ShapelyPolygon(points)
            if not shape.is_valid:
                raise TeachError("the trace crosses itself — record it again")
            if shape.area < 1.0:
                raise TeachError(f"the traced area is only {shape.area:.2f} m² — drive round it")
        anchor = mean_pose(self._anchors)
        wander = math.sqrt(sum((a[0] - anchor[0]) ** 2 + (a[1] - anchor[1]) ** 2
                               for a in self._anchors) / len(self._anchors))
        return ([apply(anchor, p) for p in points],
                {"fixes": len(self._anchors), "vertices": len(points),
                 "anchor_wander_m": round(wander, 2)})

    def _publish(self, result: Dict[str, Any]) -> Dict[str, Any]:
        self._seq += 1
        self._result = dict(result, id=self._seq)
        return self._result

    def _idle_or_raise(self) -> None:
        if self._mode is not None:
            raise TeachError(f"already recording a {self._kind}: finish or cancel it first")

    @staticmethod
    def _seconds(value: Any) -> float:
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 1.0 <= value <= 300.0):
            raise TeachError(f"seconds must be between 1 and 300, got {value!r}")
        return float(value)

    def state(self) -> Dict[str, Any]:
        with self._lock:
            out: Dict[str, Any] = {"mode": self._mode, "kind": self._kind,
                                   "result_ready": self._result is not None}
            if self._mode == "trace":
                out["points"] = [[p.lon, p.lat] for p in self._shown]
                out["no_tf"] = self._no_tf
            if self._average is not None:
                out["averaging"] = {"seconds": self._average.seconds,
                                    "progress": round(self._average.progress(self._now), 2),
                                    "fixes": len(self._average.fixes)}
            return out
