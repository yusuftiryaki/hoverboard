"""A mission = a task type + what it runs on + its parameters. ROS-free.

    {"type": "coverage", "area": "bahce", "params": {"overlap": 0.3}}
    {"type": "patrol",   "area": "bahce", "params": {"mode": "random", "laps": 0}}
    {"type": "route",    "route": "tur",  "params": {"repeat": 2, "back_and_forth": true}}
    {"type": "goto",     "target": [29.0001, 41.0002], "params": {"heading_deg": 90}}

Task types are REGISTERED (TASK_TYPES): each declares which site object it
runs on, its parameters with defaults and limits, and the planner it calls.
Adding a type is one entry — the runner (phase 3) and the web form (phase 4)
read the registry rather than knowing the types (docs/missions.md).

⚠️ AN UNKNOWN PARAMETER IS AN ERROR. "overlab": 0.3 silently ignored would run
the mission on the default and look exactly like a mission that ran as asked.

Conversion: plan_mission asks for EVERY lat/lon it needs — the survey map's
obstacles (site_map.positions_to_convert) plus the mission's own area, route,
target and heading points — in ONE converter call, the same /fromLLArray
contract as site_map_server (a short answer is NotReady, never geometry).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon

from robot_missions import planner
from robot_missions.planner import Plan, PlanError
from robot_missions.site import LatLon, Site, SiteError, offset
from robot_missions.site_map import (
    HEADING_BASELINE_M, STATION_BODY_OFFSET, STATION_BODY_RADIUS, Converter,
    NotReady, assemble, positions_to_convert)

XY = Tuple[float, float]


@dataclass(frozen=True)
class RobotSpec:
    """The machine, not the mission. ⚠️ Placeholders until B4 measures them."""

    # The trimmer head's cut, chassis-dependent (docs/missions.md decision 2).
    cut_width: float = 0.30
    # nav2.yaml robot_radius. Obstacles grow by robot_radius + obstacle_margin.
    robot_radius: float = 0.4
    obstacle_margin: float = 0.1


@dataclass(frozen=True)
class Param:
    default: Any
    kind: type                          # bool, int, float, str
    low: Optional[float] = None         # inclusive
    high: Optional[float] = None        # inclusive
    choices: Tuple[str, ...] = ()
    optional: bool = False              # default None allowed

    def check(self, name: str, value: Any) -> Any:
        if value is None and self.optional:
            return None
        if self.kind is float and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        if self.kind is bool:
            ok = isinstance(value, bool)
        elif self.kind is int:
            ok = isinstance(value, int) and not isinstance(value, bool)
        elif self.kind is float:
            ok = isinstance(value, float) and math.isfinite(value)
        else:
            ok = isinstance(value, self.kind)
        if not ok:
            raise MissionError(f"{name} must be a {self.kind.__name__}, got {value!r}")
        if self.low is not None and value < self.low:
            raise MissionError(f"{name} must be >= {self.low}, got {value}")
        if self.high is not None and value > self.high:
            raise MissionError(f"{name} must be <= {self.high}, got {value}")
        if self.choices and value not in self.choices:
            raise MissionError(f"{name} must be one of {self.choices}, got {value!r}")
        return value


class MissionError(ValueError):
    """The mission spec is malformed or names something the site lacks."""


@dataclass(frozen=True)
class TaskType:
    name: str
    on: str                             # "area", "route" or "target"
    params: Dict[str, Param]
    # (inputs, params, robot, start) -> Plan; inputs is what plan_mission built
    plan: Callable[["Inputs", Dict[str, Any], RobotSpec, Optional[XY]], Plan]
    describe: str = ""


@dataclass(frozen=True)
class Mission:
    type: TaskType
    target_name: Optional[str]          # area / route name
    target_point: Optional[LatLon]      # goto
    params: Dict[str, Any]


@dataclass(frozen=True)
class Inputs:
    """Everything a planner gets, already in `map` metres."""

    region: Any = None                  # shapely geometry (area types)
    area: Optional[Polygon] = None
    points: Tuple[XY, ...] = ()         # route / goto
    heading: Optional[float] = None     # rad in map, from a compass param


# --- the registry ------------------------------------------------------------

def _plan_coverage(inputs: Inputs, p, robot: RobotSpec, start) -> Plan:
    spacing = robot.cut_width * (1.0 - p["overlap"])
    return planner.coverage(inputs.region, spacing, angle=inputs.heading, start=start,
                            perimeter_pass=p["perimeter_pass"])


def _plan_patrol(inputs: Inputs, p, robot: RobotSpec, start) -> Plan:
    if p["mode"] == "perimeter":
        return planner.patrol_perimeter(inputs.region, step=p["step"], laps=p["laps"],
                                        start=start)
    return planner.patrol_random(inputs.region, points=p["points"], seed=p["seed"],
                                 min_separation=p["min_separation"], laps=p["laps"],
                                 start=start)


def _plan_route(inputs: Inputs, p, robot: RobotSpec, start) -> Plan:
    return planner.route(inputs.points, repeat=p["repeat"], back_and_forth=p["back_and_forth"])


def _plan_goto(inputs: Inputs, p, robot: RobotSpec, start) -> Plan:
    return planner.goto(inputs.points[0], yaw=inputs.heading, start=start)


TASK_TYPES: Dict[str, TaskType] = {t.name: t for t in (
    TaskType("coverage", "area", {
        # GPS wanders 2-5 m (decision 1): overlap is the knob that trades time
        # for fewer uncut strips. Placeholder until phase 3 measures it.
        "overlap": Param(0.2, float, 0.0, 0.9),
        # Compass bearing the rows run along (0 = north-south rows); None =
        # along the area's long side.
        "angle_deg": Param(None, float, 0.0, 360.0, optional=True),
        # Row centre lines stay this far inside the area. None = cut_width / 2,
        # so on a perfect fix the cut's edge just reaches the boundary.
        "boundary_margin": Param(None, float, 0.0, 50.0, optional=True),
        "perimeter_pass": Param(True, bool),
    }, _plan_coverage, "Alanı şerit şerit biç"),
    TaskType("patrol", "area", {
        "mode": Param("perimeter", str, choices=("perimeter", "random")),
        "laps": Param(1, int, 0, 10000),             # 0 = until stopped
        "step": Param(2.0, float, 0.5, 50.0),        # perimeter waypoint spacing
        "points": Param(10, int, 1, 1000),           # random: points per lap
        "min_separation": Param(3.0, float, 0.0, 100.0),
        "seed": Param(0, int),
        "boundary_margin": Param(None, float, 0.0, 50.0, optional=True),
    }, _plan_patrol, "Alanda devriye gez"),
    TaskType("route", "route", {
        "repeat": Param(1, int, 0, 10000),           # 0 = until stopped
        "back_and_forth": Param(False, bool),
    }, _plan_route, "Rotayı sür"),
    TaskType("goto", "target", {
        "heading_deg": Param(None, float, 0.0, 360.0, optional=True),
    }, _plan_goto, "Noktaya git"),
)}


def parse_mission(spec: Any, site: Site) -> Mission:
    if not isinstance(spec, dict):
        raise MissionError("a mission is an object with 'type' and 'params'")
    unknown = set(spec) - {"type", "area", "route", "target", "params"}
    if unknown:
        raise MissionError(f"unknown mission field(s) {sorted(unknown)}")
    task = TASK_TYPES.get(spec.get("type"))
    if task is None:
        raise MissionError(f"type must be one of {sorted(TASK_TYPES)}, got {spec.get('type')!r}")
    for other in {"area", "route", "target"} - {task.on}:
        if other in spec:
            raise MissionError(f"a {task.name} mission runs on '{task.on}', not '{other}'")

    name = point = None
    if task.on == "target":
        target = spec.get("target")
        if (not isinstance(target, list) or len(target) != 2
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           for v in target)):
            raise MissionError("target must be [lon, lat] (GeoJSON order)")
        point = LatLon(lat=float(target[1]), lon=float(target[0]))
    else:
        name = spec.get(task.on)
        pool = site.areas if task.on == "area" else site.routes
        if not any(item.name == name for item in pool):
            known = [item.name for item in pool]
            raise MissionError(f"the site has no {task.on} named {name!r} (has {known})")

    given = spec.get("params", {})
    if not isinstance(given, dict):
        raise MissionError("params must be an object")
    typos = set(given) - set(task.params)
    if typos:
        raise MissionError(f"{task.name} has no parameter(s) {sorted(typos)}; it takes "
                           f"{sorted(task.params)}")
    params = {key: param.check(key, given.get(key, param.default))
              for key, param in task.params.items()}
    return Mission(task, name, point, params)


# --- conversion + planning ---------------------------------------------------

def mission_positions(site: Site, mission: Mission) -> List[LatLon]:
    """The survey geometry's positions, then the mission's own, in the order
    plan_from_answer reads them."""
    asked = positions_to_convert(site)
    task, p = mission.type, mission.params
    if task.on == "area":
        area = _named(site.areas, mission.target_name)
        for ring in area.polygon.rings():
            asked.extend(ring)
        bearing = p.get("angle_deg")
        if bearing is not None:
            anchor = area.polygon.exterior[0]
            asked.extend((anchor, offset(anchor, bearing, HEADING_BASELINE_M)))
    elif task.on == "route":
        asked.extend(_named(site.routes, mission.target_name).points)
    else:
        asked.append(mission.target_point)
        bearing = p.get("heading_deg")
        if bearing is not None:
            asked.append(offset(mission.target_point, bearing, HEADING_BASELINE_M))
    return asked


def plan_from_answer(site: Site, mission: Mission, answer: Sequence[XY],
                     robot: RobotSpec = RobotSpec(), start: Optional[XY] = None) -> Plan:
    expected = len(mission_positions(site, mission))
    if len(answer) != expected:
        raise NotReady(f"asked to convert {expected} positions, got {len(answer)} back")
    survey_count = len(positions_to_convert(site))
    geometry = assemble(site, list(answer[:survey_count]), STATION_BODY_OFFSET,
                        STATION_BODY_RADIUS)
    own = iter(answer[survey_count:])
    task, p = mission.type, mission.params

    inputs = Inputs()
    if task.on == "area":
        area_site = _named(site.areas, mission.target_name)
        rings = [tuple(next(own) for _ in ring) for ring in area_site.polygon.rings()]
        area = Polygon(rings[0], rings[1:])
        heading = _heading(own) if p.get("angle_deg") is not None else None
        margin = p["boundary_margin"]
        if margin is None:
            margin = robot.cut_width / 2.0
        obstacles = [Polygon(r[0], r[1:]) for r in (*geometry.keepouts,
                                                     *geometry.obstacle_polygons)]
        obstacles += [Point(c.x, c.y).buffer(c.radius)
                      for c in (*geometry.circles, *geometry.station_bodies)]
        region = planner.free_region(area, obstacles, margin,
                                     robot.robot_radius + robot.obstacle_margin)
        inputs = Inputs(region=region, area=area, heading=heading)
    elif task.on == "route":
        inputs = Inputs(points=tuple(own))
    else:
        target = next(own)
        heading = _heading_from(target, next(own)) if p.get("heading_deg") is not None else None
        inputs = Inputs(points=(target,), heading=heading)
    return task.plan(inputs, p, robot, start)


def plan_mission(site: Site, mission: Mission, convert: Converter,
                 robot: RobotSpec = RobotSpec(), start: Optional[XY] = None) -> Plan:
    """mission_positions -> ONE converter call -> plan_from_answer."""
    return plan_from_answer(site, mission, list(convert(mission_positions(site, mission))),
                            robot, start)


def _named(items, name):
    for item in items:
        if item.name == name:
            return item
    raise SiteError(f"no {name!r} in the site")


def _heading(points) -> float:
    return _heading_from(next(points), next(points))


def _heading_from(a: XY, b: XY) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


__all__ = ["MissionError", "PlanError", "NotReady", "RobotSpec", "TASK_TYPES",
           "parse_mission", "plan_mission", "plan_from_answer", "mission_positions"]
