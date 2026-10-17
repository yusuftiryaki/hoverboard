"""mission_runner — runs one mission at a time on the site: plans it, drives it
waypoint by waypoint through Nav2, survives the trip to the charger, and keeps
its place on disk.

    ~/start   (robot_missions_interfaces/StartMission)  mission JSON (docs/missions.md)
    ~/pause, ~/resume, ~/abort  (std_srvs/Trigger)
    ~/status  (std_msgs/String, JSON, latched)            state, index/total, lap, skipped
    battery_supervisor/state  ◄── NORMAL / RETURNING / ... / DOCKED_IDLE / SAFE_STOP
    battery_supervisor/resume ──► asked once the robot is full (undock)
    navigate_to_pose  ──► one goal per waypoint (patrol, route, goto, long transits)
    follow_path       ──► coverage rows, straight to the controller, in `odom`
    fromLLArray / toLL ──► navsat_transform (the one lat/lon model, decision 4)

Decisions live in runner_policy.py (ROS-free, tested event by event); this
node is the glue. Why one goal per waypoint and not NavigateThroughPoses:
through-poses passes an intermediate pose once within a radius and never turns
to its heading — at a mowing row's end that cuts the corner. Stopping at each
row end is what a boustrophedon robot does anyway (it turns in place there).

⚠️ COVERAGE ROWS DO NOT GO THROUGH NAVIGATE_TO_POSE. Measured on ground truth
(test_mission_coverage.py), a 4 x 3 m lawn:
  * general profile (1.0 m goal tolerance, for GPS wander): 63.4% cut, 0.71 m
    overrun — it "arrives" a metre short of every row end, and a 0.4 m
    row-to-row transit is inside the tolerance before the robot moves;
  * NavigateToPose with a 0.15 m checker: 91-93% with a quiet GPS, but with
    the NEO-6M's wander the robot chased one row end for 15 minutes and never
    got there — bt_navigator fixes the goal in `map`, and `map` moves under
    the robot by more than 0.15 m between controller cycles.
So a row is a FollowPath goal sent straight to controller_server: a straight
line from the row's start to its end in `odom` (FollowRow + precise_goal_checker,
nav2.yaml). `odom` does not jump, so the end of the row stays put; how the plan
gets into `odom` is the `anchor` parameter (_target). A transit to a row's start
goes through navigate_to_pose when it is long (it plans round obstacles) and
always ends in a short precise JOIN line, so every row starts where it was drawn.
The numbers behind each of these choices: docs/missions.md, 3a.

Persistence: every step writes ~/.robot/mission_state.json (state_file): the
mission, the waypoint index, the lap, the skipped waypoints and where the
robot was when the lap was planned (lat/lon, via /toLL — the plan depends on
it, `map` does not survive a restart). After a restart the mission comes back
PAUSED; ~/resume re-plans from that start and carries on at the saved index.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from types import SimpleNamespace
from typing import Any, Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.srv import FromLLArray, ToLL
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_missions.mission import (
    MissionError, RobotSpec, mission_positions, parse_mission, plan_from_answer)
from robot_missions.planner import PlanError, free_intervals
from robot_missions.runner_policy import (
    Cancel, Persist, Plan, Progress, Result, ResumeSupervisor, RunnerPolicy, Send, State)
from robot_missions.site import LatLon, SiteError, load_site
from robot_missions.site_map import NotReady
from robot_missions_interfaces.srv import StartMission

RESULTS = {GoalStatus.STATUS_SUCCEEDED: Result.SUCCEEDED,
           GoalStatus.STATUS_CANCELED: Result.CANCELED,
           GoalStatus.STATUS_ABORTED: Result.ABORTED}


class MissionRunner(Node):
    def __init__(self, **kwargs):
        super().__init__("mission_runner", **kwargs)
        self.declare_parameter("site_file", "")
        self.declare_parameter("state_file", os.path.expanduser("~/.robot/mission_state.json"))
        robot = RobotSpec()
        self.declare_parameter("cut_width", robot.cut_width)
        self.declare_parameter("robot_radius", robot.robot_radius)
        self.declare_parameter("obstacle_margin", robot.obstacle_margin)
        self.declare_parameter("max_retries", 2)
        # Extra room round a bumper detection, on top of robot_radius +
        # obstacle_margin. A detection is a 0.25 m circle on the near side of
        # something of unknown size, located only as well as the robot knew
        # where it was: rows cut round it at the bare clearance ended inside
        # the costmap's inscribed zone often enough for RPP to refuse them.
        self.declare_parameter("detection_margin", 0.3)
        self.declare_parameter("max_skips_in_a_row", 3)
        self.declare_parameter("server_wait_s", 15.0)
        self.declare_parameter("cancel_grace_s", 3.0)
        # navsat_transform may not have its transform yet: ask again this often,
        # this many times, before the plan counts as failed.
        self.declare_parameter("convert_retry_s", 1.0)
        self.declare_parameter("convert_attempts", 60)
        self.declare_parameter("supervisor", "battery_supervisor")
        # site_map_server's bumper finds (detected.py), in `map`.
        self.declare_parameter("detected_topic", "site_map_server/detected")
        # Coverage rows (see the module docstring).
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("row_controller", "FollowRow")
        self.declare_parameter("row_goal_checker", "precise_goal_checker")
        self.declare_parameter("path_step", 0.1)
        # Transits longer than this go through navigate_to_pose first (it
        # plans round obstacles); the general checker's own tolerance.
        self.declare_parameter("transit_skip_m", 1.0)
        # Further than this from a row's start, the robot drives to it first.
        self.declare_parameter("join_tolerance_m", 0.3)
        # "chain" or "leg" — see _target.
        self.declare_parameter("anchor", "chain")

        p = self.get_parameter
        self._robot = RobotSpec(cut_width=p("cut_width").value,
                                robot_radius=p("robot_radius").value,
                                obstacle_margin=p("obstacle_margin").value)
        self._policy = RunnerPolicy(max_retries=p("max_retries").value,
                                    cancel_grace_s=p("cancel_grace_s").value,
                                    max_skips_in_a_row=p("max_skips_in_a_row").value)
        self._plan = None
        self._site = None
        self._mission = None
        self._start_ll: Optional[LatLon] = None
        # False = the lap was planned with no robot pose (no TF yet), so a
        # re-plan without one reproduces it; True + no _start_ll = the start
        # was used but never made it to lat/lon, and the saved index is moot.
        self._planned_with_start = False
        self._handles: Dict[int, Any] = {}
        # index -> the waypoint's target in `odom` ("chain" anchoring); emptied
        # whenever the mission stops running or is re-planned (re-anchor).
        self._targets: Dict[int, Any] = {}
        # True only while a row's own FollowPath runs — never on the way to
        # it. What a trimmer output will follow; ~/status "work".
        self._cutting = False
        self._planning_lap: Optional[int] = None
        self._convert_tries = 0

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._follow = ActionClient(self, FollowPath, "follow_path")
        self._reverse_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self._from_ll = self.create_client(FromLLArray, "fromLLArray")
        self._to_ll = self.create_client(ToLL, "toLL")
        supervisor = p("supervisor").value
        self._sup_resume = self.create_client(Trigger, f"{supervisor}/resume")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, "~/status", latched)
        self.create_subscription(String, f"{supervisor}/state", self._on_supervisor, latched)
        self._detected = []             # (x, y, r) in `map`
        self.create_subscription(String, p("detected_topic").value, self._on_detected, latched)
        self.create_service(StartMission, "~/start", self._on_start)
        self.create_service(Trigger, "~/pause", self._command(self._policy.pause))
        self.create_service(Trigger, "~/resume", self._command(self._policy.resume))
        self.create_service(Trigger, "~/abort", self._command(self._policy.abort))
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)

        self._restore()
        self._publish_status()

    # --- inputs -------------------------------------------------------------

    def _on_start(self, request, response):
        try:
            spec = json.loads(request.mission)
            site = self._load_site()
            mission = parse_mission(spec, site)
        except (json.JSONDecodeError, SiteError, MissionError, OSError) as exc:
            response.accepted, response.message = False, str(exc)
            return response
        accepted, message, actions = self._policy.start(spec)
        if accepted:
            self._site, self._mission, self._plan = site, mission, None
            self._start_ll = None
        response.accepted, response.message = accepted, message
        self._run(actions)
        return response

    def _command(self, method):
        def handler(_request, response):
            response.success, response.message, actions = method()
            self._run(actions)
            return response
        return handler

    def _on_detected(self, message: String) -> None:
        self._detected = [(d["x"], d["y"], d["radius"]) for d in json.loads(message.data)]

    def _on_supervisor(self, message: String) -> None:
        self._run(self._policy.supervisor_state(message.data, self._now()))

    def _tick(self) -> None:
        self._run(self._policy.tick(self._now()))

    # --- actions ------------------------------------------------------------

    def _run(self, actions) -> None:
        if self._policy.state != State.RUNNING:
            # Whatever happens next (resume after a pause or the dock) starts
            # from a fresh map->odom snapshot: odom has drifted meanwhile.
            self._targets = {}
        for action in actions:
            if isinstance(action, Plan):
                self._start_planning(action.lap)
            elif isinstance(action, Send):
                self._send(action)
            elif isinstance(action, Cancel):
                handle = self._handles.pop(action.token, None)
                if handle is not None:
                    handle.cancel_goal_async()
            elif isinstance(action, ResumeSupervisor):
                self._ask_supervisor()
            elif isinstance(action, Persist):
                self._persist()
        self._publish_status()

    # Planning: robot pose (start) -> one /fromLLArray -> plan.
    def _start_planning(self, lap: int) -> None:
        self._planning_lap = lap
        self._convert_tries = 0
        if self._site is None or self._mission is None:
            try:            # restored from disk: rebuild from the saved mission
                self._site = self._load_site()
                self._mission = parse_mission(self._policy.progress.mission, self._site)
            except (SiteError, MissionError, OSError) as exc:
                self._run(self._policy.plan_failed(str(exc)))
                return
        self._try_convert()

    def _try_convert(self) -> None:
        if self._planning_lap is None or self._policy.state != State.PLANNING:
            self._planning_lap = None       # aborted meanwhile
            return
        restored_start = self._start_ll is not None and self._plan is None and \
            self._policy.progress.index > 0
        asked = mission_positions(self._site, self._mission)
        if restored_start:
            asked = asked + [self._start_ll]
        if not self._from_ll.service_is_ready():
            self._retry_convert("navsat_transform's /fromLLArray is not up")
            return
        request = FromLLArray.Request()
        request.ll_points = [GeoPoint(latitude=q.lat, longitude=q.lon) for q in asked]
        future = self._from_ll.call_async(request)
        future.add_done_callback(lambda f: self._on_converted(f, len(asked), restored_start))

    def _on_converted(self, future, asked: int, restored_start: bool) -> None:
        if self._planning_lap is None or self._policy.state != State.PLANNING:
            return
        response = future.result()
        answer = [(pt.x, pt.y) for pt in response.map_points] if response else []
        if len(answer) != asked:
            self._retry_convert(f"/fromLLArray gave {len(answer)} of {asked} points")
            return
        if restored_start:
            start, answer = answer[-1], answer[:-1]
        else:
            start = self._robot_xy()
        progress = self._policy.progress
        mission = self._mission
        if mission.type.name == "patrol" and mission.params.get("mode") == "random":
            # A new draw every lap: seed + lap (planner.patrol_random).
            params = dict(mission.params, seed=mission.params["seed"] + progress.lap)
            mission = type(mission)(mission.type, mission.target_name,
                                    mission.target_point, params)
        try:
            extra = self.get_parameter("detection_margin").value
            plan = plan_from_answer(self._site, mission, answer, self._robot, start,
                                    extra_obstacles=[(x, y, r + extra)
                                                     for x, y, r in self._detected])
        except NotReady as exc:
            self._retry_convert(str(exc))
            return
        except (PlanError, MissionError, SiteError) as exc:
            self._planning_lap = None
            self._run(self._policy.plan_failed(str(exc)))
            return
        self._planning_lap = None
        self._plan = plan
        self._targets = {}
        self.get_logger().info(
            f"planned {plan.kind}: {len(plan.waypoints)} waypoints, "
            f"{ {k: round(v, 2) for k, v in plan.info.items()} }")
        if not restored_start:
            self._planned_with_start = start is not None
            self._start_ll = None
            if start is not None:
                self._remember_start(start)
        self._run(self._policy.planned(len(plan.waypoints), plan.repeat))

    def _retry_convert(self, why: str) -> None:
        self._convert_tries += 1
        if self._convert_tries >= self.get_parameter("convert_attempts").value:
            self._planning_lap = None
            self._run(self._policy.plan_failed(f"no conversion: {why}"))
            return
        self.get_logger().info(f"waiting to plan: {why}", throttle_duration_sec=10.0)
        timer = None

        def again():
            timer.cancel()
            self._try_convert()
        timer = self.create_timer(self.get_parameter("convert_retry_s").value, again)

    def _send(self, action: Send, waited: float = 0.0, backed_off: bool = False) -> None:
        if action.token != self._policy.in_flight:
            return                       # dropped while waiting for tf
        plan, index = self._plan, action.index
        waypoint = plan.waypoints[index]
        if plan.precise and not self._tf_ready():
            # ⚠️ Not a waypoint failure. A runner whose tf buffer is still
            # empty (it just started) used to abort every goal on the spot —
            # "frame does not exist" — and skip three waypoints in under 20 ms,
            # which paused a mission that had never moved (seen twice in the
            # mission tests before the logs showed why). Wait like for a server.
            limit = self.get_parameter("server_wait_s").value
            if waited >= limit:
                self._run(self._policy.goal_result(action.token, Result.ABORTED, self._now()))
                return
            timer = None

            def again():
                timer.cancel()
                self._send(action, waited + 0.5)
            timer = self.create_timer(0.5, again)
            return
        if not plan.precise:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose("map", waypoint.x, waypoint.y, waypoint.yaw)
            self._dispatch(self._nav, goal, action.token, "navigate_to_pose")
            return
        if not backed_off and self._back_off(action, waited):
            return
        target = self._target(index)
        if target is None:
            self._run(self._policy.goal_result(action.token, Result.ABORTED, self._now()))
            return
        token = action.token
        if waypoint.work and index > 0:
            before = plan.waypoints[index - 1]
            start = self._targets.get(index - 1) or self._target(index - 1, fresh=True)
            # Cut round what the bumper found since the plan was made: the
            # row's free stretches, as fractions along it (planner.
            # free_intervals), so the same split applies in `map` (where the
            # detections are) and in `odom` (where the row is driven).
            grow = (self._robot.robot_radius + self._robot.obstacle_margin
                    + self.get_parameter("detection_margin").value)
            pieces = free_intervals((before.x, before.y), (waypoint.x, waypoint.y),
                                    [(x, y, r + grow) for x, y, r in self._detected])
            pieces = self._not_behind(before, waypoint, pieces)
            self._cut_pieces(token, before, waypoint, start, target, pieces)
            return
        # A transit to the next row's start.
        self._approach(token, waypoint, target,
                       then=lambda: self._run(self._policy.goal_result(
                           token, Result.SUCCEEDED, self._now())))

    def _back_off(self, action: Send, waited: float) -> bool:
        """Back away first if the robot sits inside a detection's footprint.

        After a bumper press the robot stands right against what it hit —
        inside the obstacle's inflation the moment site_map_server draws it.
        From there every straight line "collides ahead" (RPP refuses it) and
        navigate_to_pose cannot plan out of lethal space: measured, three
        waypoints failed in a row and the mission paused, the robot never
        having moved off the rock. So: reverse straight back far enough to
        clear it (_reverse), then carry on. True = backing off."""
        here = self._robot_xy()
        if here is None or not self._detected:
            return False
        try:
            tf = self._tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return False
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Only when TOUCHING it (inside the costmap's inscribed zone, where
        # nothing can be planned): robot radius + a hand's breadth. ⚠️ Not
        # the rows' wider clearance — with that, a robot merely mowing the
        # lawn's edge a metre from a detection reversed before every leg and
        # the perimeter pass failed three times in a row (CI, PR #16).
        clearance = self._robot.robot_radius + 0.15
        need = 0.0
        for x, y, r in self._detected:
            # Only what is AHEAD: backing away from something behind would
            # drive into it.
            if math.cos(math.atan2(y - here[1], x - here[0]) - yaw) <= 0.0:
                continue
            need = max(need, clearance + r - math.hypot(x - here[0], y - here[1]))
        if need <= 0.0:
            return False
        self._reverse(action.token, min(1.0, need + 0.05),
                      then=lambda: self._send(action, waited, backed_off=True))
        return True

    def _reverse(self, token: int, distance: float, then) -> None:
        """Straight back `distance` m at 0.1 m/s on cmd_vel_nav — the same
        input the controller drives, so nav_cmd_gate and twist_mux (teleop
        above it, the supervisor's lock, the deadman below) all still apply.

        ⚠️ Not behavior_server's backup: it collision-checks the robot's own
        footprint first, and a robot touching what it just hit is IN that
        obstacle's inflation by definition — measured, it aborted five times
        in 30 ms without moving. Reversing blind is acceptable here and only
        here: it retraces, at walking pace, the ground the robot just drove
        over, and the firmware's bumper veto never blocks reverse."""
        frame = self.get_parameter("odom_frame").value
        origin = self._robot_in(frame)
        started = self._now()
        self.get_logger().info(f"backing off {distance:.2f} m from a detected obstacle")
        timer = None

        def step():
            here = self._robot_in(frame)
            gone = math.hypot(here[0] - origin[0], here[1] - origin[1]) \
                if here is not None and origin is not None else 0.0
            if token != self._policy.in_flight:          # paused, aborted, handed over
                timer.cancel()
                self._reverse_pub.publish(Twist())
                return
            if gone >= distance or self._now() - started > 15.0:
                timer.cancel()
                self._reverse_pub.publish(Twist())
                then()
                return
            twist = Twist()
            twist.linear.x = -0.1
            self._reverse_pub.publish(twist)
        timer = self.create_timer(0.1, step)

    def _not_behind(self, a, b, pieces):
        """A row resumed mid-way (the bumper stopped it, the goal was retried)
        carries on from where the robot is, not from the row's start: drop
        what lies behind the robot ALONG the row — only when the robot is on
        the row (within half a metre of its line), not back from the dock."""
        here = self._robot_xy()
        length = math.hypot(b.x - a.x, b.y - a.y)
        if here is None or length < 1e-6:
            return pieces
        ux, uy = (b.x - a.x) / length, (b.y - a.y) / length
        along = (here[0] - a.x) * ux + (here[1] - a.y) * uy
        across = abs(-(here[0] - a.x) * uy + (here[1] - a.y) * ux)
        if across > 0.5 or along <= 0.3:
            return pieces
        t_here = along / length
        return [(max(t0, t_here), t1) for t0, t1 in pieces
                if (t1 - max(t0, t_here)) * length >= 0.3]

    def _cut_pieces(self, token: int, a_map, b_map, a_odom, b_odom, pieces) -> None:
        """Mow pieces[0], then the rest, each reached first with the trimmer
        off (_approach: never a slanted row from wherever the robot is)."""
        if not pieces:
            self._run(self._policy.goal_result(token, Result.SUCCEEDED, self._now()))
            return
        (t0, t1), rest = pieces[0], pieces[1:]

        def lerp(p, q, t):
            return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
        heading = math.atan2(b_map.y - a_map.y, b_map.x - a_map.x)
        sx, sy = lerp((a_map.x, a_map.y), (b_map.x, b_map.y), t0)
        piece_start = SimpleNamespace(x=sx, y=sy, yaw=heading)
        start, end = lerp(a_odom, b_odom, t0), lerp(a_odom, b_odom, t1)

        def cut():
            self._cutting = True
            self._publish_status()
            self._send_line(token, start, end,
                            then=(lambda: self._cut_pieces(token, a_map, b_map, a_odom,
                                                           b_odom, rest)) if rest else None)
        self._approach(token, piece_start, start, then=cut)

    def _approach(self, token: int, waypoint, target, then) -> None:
        """Get to `target` (`waypoint` in odom) with the trimmer off, then run
        `then`. Far (> transit_skip_m): navigate_to_pose first — it plans round
        obstacles, which a straight line does not (back from the dock the
        station body itself is in the way) — then a precise straight JOIN, so
        the next row starts where it was drawn rather than wherever the loose
        goal checker stopped (up to 1 m off: 0.77 m of cutting outside the
        lawn, measured, before the join). Within join_tolerance_m: nothing."""
        self._cutting = False
        here = self._robot_in(self.get_parameter("odom_frame").value)
        if here is None:
            self._run(self._policy.goal_result(token, Result.ABORTED, self._now()))
            return
        gap = math.hypot(target[0] - here[0], target[1] - here[1])
        if gap <= self.get_parameter("join_tolerance_m").value:
            then()
            return

        def join():
            now = self._robot_in(self.get_parameter("odom_frame").value)
            if now is None:
                self._run(self._policy.goal_result(token, Result.ABORTED, self._now()))
                return
            if math.hypot(target[0] - now[0], target[1] - now[1]) < 0.05:
                then()
                return
            self._send_line(token, now, target, then=then)

        if gap > self.get_parameter("transit_skip_m").value:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose("map", waypoint.x, waypoint.y, waypoint.yaw)
            self._dispatch(self._nav, goal, token, "navigate_to_pose", then=join)
            return
        join()

    def _tf_ready(self) -> bool:
        odom = self.get_parameter("odom_frame").value
        return (self._tf.can_transform(odom, "map", Time())
                and self._tf.can_transform(odom, "base_link", Time()))

    def _map_to_odom(self):
        """(tx, ty, yaw) of map -> odom right now, or None."""
        frame = self.get_parameter("odom_frame").value
        try:
            tf = self._tf.lookup_transform(frame, "map", Time())
        except TransformException as exc:
            self.get_logger().warn(f"no map->{frame}: {exc}")
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _target(self, index: int, fresh: bool = False):
        """Waypoint `index` in `odom`, per the `anchor` parameter.

        "chain" (default): the previous waypoint's target plus the plan's step
        to this one, rotated by map->odom's CURRENT yaw. So the pattern keeps
        its own geometry — rows exactly a spacing apart — and only the heading
        follows `map` (magnetometer), which odom's gyro-only yaw needs: it
        drifts with the gyro bias (ekf.yaml, ekf_local), and one snapshot for
        a whole lawn was MEASURED to slant the rows ~15 deg over 200 s
        (90.0% cut, quiet GPS). The GPS position enters once, at the first
        waypoint after every (re)start.
        "leg": every waypoint through a fresh map->odom, position included:
        the pattern follows the GPS, and every wander lands between two rows.
        """
        tf = self._map_to_odom()
        if tf is None:
            return None
        tx, ty, yaw = tf
        c, s_ = math.cos(yaw), math.sin(yaw)
        waypoint = self._plan.waypoints[index]
        previous = self._targets.get(index - 1)
        if self.get_parameter("anchor").value == "chain" and previous is not None \
                and not fresh:
            before = self._plan.waypoints[index - 1]
            dx, dy = waypoint.x - before.x, waypoint.y - before.y
            target = (previous[0] + c * dx - s_ * dy, previous[1] + s_ * dx + c * dy)
        else:
            target = (tx + c * waypoint.x - s_ * waypoint.y,
                      ty + s_ * waypoint.x + c * waypoint.y)
        self._targets[index] = target
        return target

    def _send_line(self, token: int, start, end, then=None) -> None:
        frame = self.get_parameter("odom_frame").value
        (x0, y0), (x1, y1) = start, end
        heading = math.atan2(y1 - y0, x1 - x0)
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(length / self.get_parameter("path_step").value)))
        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.frame_id = frame
        goal.path.header.stamp = self.get_clock().now().to_msg()
        goal.path.poses = [self._pose(frame, x0 + (x1 - x0) * k / steps,
                                      y0 + (y1 - y0) * k / steps, heading)
                           for k in range(steps + 1)]
        goal.controller_id = self.get_parameter("row_controller").value
        goal.goal_checker_id = self.get_parameter("row_goal_checker").value
        self._dispatch(self._follow, goal, token, "follow_path", then=then)

    def _pose(self, frame: str, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _dispatch(self, client, goal, token: int, name: str, then=None,
                  waited: float = 0.0) -> None:
        """Send; report the result to the policy — or, when `then` is given
        and the goal SUCCEEDED, run `then` instead (a two-stage leg).

        A server not discovered yet is waited for (server_wait_s), not
        counted as a failure: a runner started a moment after Nav2 has not
        seen its servers yet."""
        if token != self._policy.in_flight:
            return                       # dropped while waiting
        if not client.server_is_ready():
            limit = self.get_parameter("server_wait_s").value
            if waited >= limit:
                self.get_logger().warn(f"{name} not up after {limit} s; counting it as aborted")
                self._run(self._policy.goal_result(token, Result.ABORTED, self._now()))
                return
            timer = None

            def again():
                timer.cancel()
                self._dispatch(client, goal, token, name, then, waited + 0.5)
            timer = self.create_timer(0.5, again)
            return
        sent = client.send_goal_async(goal)
        sent.add_done_callback(lambda f: self._on_goal_response(f, token, then))

    def _on_goal_response(self, future, token: int, then=None) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._run(self._policy.goal_result(token, Result.ABORTED, self._now()))
            return
        if token != self._policy.in_flight:
            # Dropped (paused, aborted, handed to the supervisor) while the
            # goal was on its way: take it back.
            handle.cancel_goal_async()
            return
        self._handles[token] = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, token, then))

    def _on_result(self, future, token: int, then=None) -> None:
        self._handles.pop(token, None)
        if then is None:
            self._cutting = False        # a row (or any last stage) is over
        result = future.result()
        status = RESULTS.get(result.status if result else None, Result.ABORTED)
        if then is not None and status == Result.SUCCEEDED and token == self._policy.in_flight:
            then()
            return
        self._run(self._policy.goal_result(token, status, self._now()))

    def _ask_supervisor(self) -> None:
        if not self._sup_resume.service_is_ready():
            self._run(self._policy.supervisor_refused("its resume service is not up"))
            return

        def done(future):
            response = future.result()
            if response is None or not response.success:
                why = response.message if response else "no answer"
                self._run(self._policy.supervisor_refused(why))
        self._sup_resume.call_async(Trigger.Request()).add_done_callback(done)

    # --- persistence ----------------------------------------------------------

    def _persist(self) -> None:
        progress = self._policy.progress
        data = {"mission": progress.mission, "index": progress.index, "lap": progress.lap,
                "skipped": progress.skipped, "state": self._policy.state.value,
                "start_ll": ([self._start_ll.lat, self._start_ll.lon]
                             if self._start_ll is not None else None),
                "planned_with_start": self._planned_with_start}
        path = self.get_parameter("state_file").value
        if not path:
            return
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        # Atomic: a power cut mid-write must leave the old file, not half of one.
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle)
        os.replace(tmp, path)

    def _restore(self) -> None:
        path = self.get_parameter("state_file").value
        if not path or not os.path.exists(path):
            return
        try:
            with open(path, encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"ignoring unreadable {path}: {exc}")
            return
        if data.get("state") in (None, "IDLE", "DONE", "FAILED") or not data.get("mission"):
            return
        start = data.get("start_ll")
        self._start_ll = LatLon(lat=start[0], lon=start[1]) if start else None
        self._policy.restore(Progress(mission=data["mission"], index=int(data.get("index", 0)),
                                      lap=int(data.get("lap", 0)),
                                      skipped=list(data.get("skipped", []))))
        self._planned_with_start = bool(data.get("planned_with_start", True))
        if (self._start_ll is None and self._planned_with_start
                and self._policy.progress.index > 0):
            # Without the start the re-plan may order the work differently, so
            # the saved index would point elsewhere: start the lap over.
            self.get_logger().warn("restored mission has no saved start; the lap restarts")
            self._policy.progress.index = 0
        self.get_logger().info(f"restored a {data['mission'].get('type')} mission at "
                               f"waypoint {self._policy.progress.index}, PAUSED")

    def _remember_start(self, start) -> None:
        if not self._to_ll.service_is_ready():
            return
        request = ToLL.Request()
        request.map_point = Point(x=float(start[0]), y=float(start[1]))

        def done(future):
            response = future.result()
            ll = response.ll_point if response else None
            # ⚠️ Not ready = (0, 0), not an error (the /fromLL trap again).
            if ll is not None and (ll.latitude, ll.longitude) != (0.0, 0.0):
                self._start_ll = LatLon(lat=ll.latitude, lon=ll.longitude)
                self._persist()
        self._to_ll.call_async(request).add_done_callback(done)

    # --- helpers --------------------------------------------------------------

    def _load_site(self):
        path = self.get_parameter("site_file").value
        if not path:
            raise SiteError("no site_file — missions run on a site (docs/missions.md)")
        return load_site(path)

    def _robot_xy(self):
        return self._robot_in("map")

    def _robot_in(self, frame: str):
        try:
            tf = self._tf.lookup_transform(frame, "base_link", Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        status = self._policy.status()
        if self._plan is not None and 0 <= status["index"] < len(self._plan.waypoints):
            if self._plan.precise:
                status["work"] = self._cutting and status["state"] == "RUNNING"
            else:
                status["work"] = self._plan.waypoints[status["index"]].work
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionRunner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
