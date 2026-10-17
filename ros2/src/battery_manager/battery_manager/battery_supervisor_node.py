"""SP5: /battery -> return to the dock, dock, charge; or stop where it stands.

The decisions are supervisor_policy.py's (ROS-free, unit-tested); this node only
feeds it and carries out its Steps with Nav2's own actions:

    CANCEL_MISSIONS  cancel-all on follow_gps_waypoints, follow_waypoints,
                     navigate_through_poses, navigate_to_pose, follow_path
    GO_TO_STAGING    navigate_to_pose -> the dock's staging pose (map frame)
    FACE_DOCK        spin, by however much turns base_link towards the dock
    DOCK             dock_robot (opennav_docking), navigate_to_staging_pose=False
    UNDOCK           undock_robot
    CANCEL_ALL       all of the above plus spin/dock/undock, fire and forget

and publishes:

    nav_lock (std_msgs/Bool, 2 Hz)   twist_mux lock above Nav2, below teleop
    ~/state  (std_msgs/String, latched) the Phase, for people and tests

and serves ~/return_home (go and charge now) and ~/resume (carry on: undock
from the dock, or clear a SAFE_STOP), both std_srvs/Trigger.
    /diagnostics

⚠️ Why it stages and turns by itself instead of letting DockRobot navigate:
Nav2's goal checker is deliberately loose here (1.0 m, ANY heading — nav2.yaml:
GPS wanders and waypoints have no meaningful heading). DockRobot's own staging
navigation uses that same checker, then demands the marker be in view for its
initial perception, and fails outright — no retry — if the robot happened to
arrive facing away. A spin towards the surveyed dock position puts the marker
in the camera's 62 deg; the docking controller does the precise part.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, Spin, UndockRobot
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from battery_manager.supervisor_policy import (BatterySample, Phase, Step, Supervisor,
                                               Thresholds, load_dock)
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLLArray
from robot_missions.site import load_site
from robot_missions.site_map import (STATION_BODY_OFFSET, STATION_BODY_RADIUS, NotReady,
                                     assemble, positions_to_convert)

# Mission-level first: a waypoint follower whose current navigate_to_pose is
# cancelled under it just moves on to its next waypoint.
# follow_path last: mission_runner drives coverage rows by calling the
# controller directly (no bt_navigator), so cancelling navigate_* alone would
# leave a row being mowed while the robot is meant to be heading home — and
# the staging goal's own FollowPath would then preempt it mid-row.
MISSION_ACTIONS = ("follow_gps_waypoints", "follow_waypoints",
                   "navigate_through_poses", "navigate_to_pose", "follow_path")
OWN_ACTIONS = ("spin", "dock_robot", "undock_robot")
CANCEL_WAIT_S = 3.0
SERVER_WAIT_S = 10.0
# Under this the robot already faces the dock well enough.
FACE_TOLERANCE_RAD = 0.10

DOCK_ERRORS = {getattr(DockRobot.Result, n): n for n in (
    "DOCK_NOT_IN_DB", "DOCK_NOT_VALID", "FAILED_TO_STAGE", "FAILED_TO_DETECT_DOCK",
    "FAILED_TO_CONTROL", "FAILED_TO_CHARGE", "UNKNOWN")}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BatterySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("battery_supervisor")
        defaults = Thresholds()
        names = ("return_soc", "critical_soc", "return_cell_v", "critical_cell_v",
                 "debounce_s", "voltage_filter_s", "stale_s", "charging_current_a",
                 "charge_lost_s")
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("cells", defaults.cells)
        self.declare_parameter("dock_database", "")
        self.declare_parameter("dock_id", "home")
        self.declare_parameter("dock_type", "charging_dock")
        # Must equal -staging_x_offset of the dock plugin in nav2.yaml; the
        # config test holds them together.
        self.declare_parameter("staging_distance", 1.5)
        self.declare_parameter("max_staging_time_s", 300.0)
        # The site (docs/missions.md). When it has a station named dock_id,
        # HOME COMES FROM THE SITE — lat/lon through navsat_transform's
        # /fromLLArray, the same conversion the survey map's station body went
        # through — and docks.yaml's `map` pose is ignored: `map` is anchored
        # on the first GPS fix (ekf.yaml), so a fixed pose there is right only
        # for a robot that boots on the dock, facing the way the file says.
        self.declare_parameter("site_file", "")

        p = self.get_parameter
        thresholds = Thresholds(cells=int(p("cells").value),
                                **{n: float(p(n).value) for n in names})
        self._sup = Supervisor(thresholds)
        self._dock_id = str(p("dock_id").value)
        self._site = self._site_with_station(str(p("site_file").value))
        if self._site is None:
            self._dock = load_dock(str(p("dock_database").value), self._dock_id)
            if self._dock["frame"] != "map":
                raise ValueError("the home dock must be surveyed in `map` (handoff decision 12)")
            self._home_source = "docks.yaml"
        else:
            self._dock = None            # until navsat_transform converts it
            self._home_source = "site (waiting for navsat_transform)"
        self._dock_type = str(p("dock_type").value)
        self._staging_distance = float(p("staging_distance").value)
        self._max_staging_time = float(p("max_staging_time_s").value)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._spin = ActionClient(self, Spin, "spin")
        self._dock_client = ActionClient(self, DockRobot, "dock_robot")
        self._undock = ActionClient(self, UndockRobot, "undock_robot")
        self._cancel = {name: self.create_client(CancelGoal, f"{name}/_action/cancel_goal")
                        for name in MISSION_ACTIONS + OWN_ACTIONS}

        self._last_battery = None
        self._last_sample = None
        self._published_phase: Optional[Phase] = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "~/state", latched)
        self._lock_pub = self.create_publisher(Bool, "nav_lock", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(BatteryState, "battery", self._on_battery, 10)
        self.create_service(Trigger, "~/resume", self._on_resume)
        self.create_service(Trigger, "~/return_home", self._on_return_home)
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._diag_tick)
        self._from_ll = self.create_client(FromLLArray, "fromLLArray")
        self._converting = False
        self._home_timer = self.create_timer(1.0, self._convert_home)
        if self._site is None:
            self._home_timer.cancel()
        # The web page moved the station (or added one): read the site again.
        self.create_service(Trigger, "~/reload_site", self._on_reload_site)
        self._publish_state()

    # ---- inputs -------------------------------------------------------------------
    def _on_battery(self, msg: BatteryState) -> None:
        now = self._now()
        self._last_battery = now
        self._last_sample = BatterySample(percentage=msg.percentage, voltage=msg.voltage,
                                          current=msg.current,
                                          status=msg.power_supply_status)
        self._run(self._sup.on_battery(self._last_sample, now))

    def _tick(self) -> None:
        now = self._now()
        if self._last_battery is None or now - self._last_battery > self._sup.t.stale_s:
            self._run(self._sup.on_stale(now))
        self._lock_pub.publish(Bool(data=self._sup.lock))

    def _on_resume(self, _request, response):
        accepted, message, steps = self._sup.resume()
        self._run(steps)
        response.success, response.message = accepted, message
        return response

    def _on_return_home(self, _request, response):
        accepted, message, steps = self._sup.return_home()
        self._run(steps)
        response.success, response.message = accepted, message
        return response

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- carrying out Steps -----------------------------------------------------
    def _run(self, steps: List[Step]) -> None:
        for step in steps:
            self.get_logger().info(f"{self._sup.phase.value}: {step.value} "
                                   f"({self._sup.reason})")
            getattr(self, f"_do_{step.value.lower()}")()
        self._publish_state()

    def _report(self, step: Step, ok: bool, detail: str = "") -> None:
        if not ok:
            self.get_logger().error(f"{step.value} failed: {detail}")
        self._run(self._sup.on_result(step, ok, detail))

    def _cancel_all_goals(self, names, on_done=None) -> None:
        """Cancel-all (zero goal id, zero stamp) on each action, waiting up to
        CANCEL_WAIT_S for its service to be discovered. An action nobody runs
        (no GPS waypoint follower, say) just times out; one that IS running but
        was not yet discovered must not be skipped — that is a mission left
        driving while the robot is supposed to be going home."""
        pending = list(names)
        futures = []
        deadline = self._now() + CANCEL_WAIT_S

        def poll():
            for name in list(pending):
                if self._cancel[name].service_is_ready():
                    futures.append(self._cancel[name].call_async(CancelGoal.Request()))
                    pending.remove(name)
            answered = all(f.done() for f in futures)
            if (not pending and answered) or self._now() > deadline:
                timer.cancel()
                self.destroy_timer(timer)
                if on_done is not None:
                    on_done(answered)
        timer = self.create_timer(0.1, poll)
        poll()

    def _do_cancel_missions(self) -> None:
        self._cancel_all_goals(MISSION_ACTIONS,
                               lambda ok: self._report(Step.CANCEL_MISSIONS, ok))

    def _do_cancel_all(self) -> None:
        self._cancel_all_goals(OWN_ACTIONS + MISSION_ACTIONS)

    # ---- home from the site ---------------------------------------------------
    def _site_with_station(self, path: str):
        if not path:
            return None
        site = load_site(path, missing_ok=True)   # not saved yet = no station
        if any(st.name == self._dock_id for st in site.stations):
            return site
        self.get_logger().warn(
            f"site {path} has no station named '{self._dock_id}' — home comes from "
            "docks.yaml, and the survey map does not draw the station")
        return None

    def _on_reload_site(self, _request, response):
        if self._sup.phase.value not in ("NORMAL", "CHARGING", "DOCKED_IDLE", "SAFE_STOP"):
            # Mid-return or mid-dock: changing home under a running sequence
            # would send the dock to one place and the staging to another.
            response.success = False
            response.message = f"not while {self._sup.phase.value} — try again when it is done"
            return response
        try:
            site = self._site_with_station(str(self.get_parameter("site_file").value))
        except (OSError, ValueError) as exc:
            response.success, response.message = False, f"not reloaded: {exc}"
            return response
        self._site = site
        if site is None:
            self._dock = load_dock(str(self.get_parameter("dock_database").value),
                                   self._dock_id)
            self._home_source = "docks.yaml"
            self._home_timer.cancel()
        else:
            self._dock = None
            self._home_source = "site (waiting for navsat_transform)"
            self._home_timer.reset()
        response.success, response.message = True, f"home from {self._home_source}"
        return response

    def _convert_home(self) -> None:
        """Ask /fromLLArray until it answers in full (it answers an EMPTY list,
        not an error, before navsat_transform has a transform)."""
        if self._dock is not None or self._converting:
            return
        if not self._from_ll.service_is_ready():
            return
        asked = positions_to_convert(self._site)
        request = FromLLArray.Request()
        request.ll_points = [GeoPoint(latitude=q.lat, longitude=q.lon) for q in asked]
        self._converting = True
        asked_site = self._site

        def done(future):
            self._converting = False
            if asked_site is not self._site:
                return                    # the site was reloaded meanwhile
            response = future.result()
            answer = [(pt.x, pt.y) for pt in response.map_points] if response else []
            try:
                geometry = assemble(self._site, answer, STATION_BODY_OFFSET,
                                    STATION_BODY_RADIUS)
            except NotReady:
                return                    # asked again on the next tick
            station = next(st for st in geometry.stations if st.name == self._dock_id)
            self._dock = {"x": station.x, "y": station.y, "yaw": station.yaw, "frame": "map"}
            self._home_source = "site"
            self._home_timer.cancel()
            self.get_logger().info(
                f"home '{self._dock_id}' from the site: ({station.x:.2f}, {station.y:.2f}) "
                f"in map, yaw {math.degrees(station.yaw):.1f} deg")
        self._from_ll.call_async(request).add_done_callback(done)

    def _home_unknown(self, step: Step) -> bool:
        if self._dock is not None:
            return False
        self._report(step, False, "home is not known yet: the site's station has not "
                                  "been converted (is navsat_transform running?)")
        return True

    def _do_go_to_staging(self) -> None:
        if self._home_unknown(Step.GO_TO_STAGING):
            return
        d, yaw = self._dock, self._dock["yaw"]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = d["x"] - self._staging_distance * math.cos(yaw)
        goal.pose.pose.position.y = d["y"] - self._staging_distance * math.sin(yaw)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._send(self._nav, goal, Step.GO_TO_STAGING,
                   lambda r, status: (status == GoalStatus.STATUS_SUCCEEDED,
                                      f"navigate_to_pose status {status}"))

    def _do_face_dock(self) -> None:
        if self._home_unknown(Step.FACE_DOCK):
            return
        try:
            tf = self._tf.lookup_transform("map", "base_link", Time(),
                                           timeout=Duration(seconds=1.0))
        except TransformException as exc:
            self._report(Step.FACE_DOCK, False, f"no map->base_link: {exc}")
            return
        t = tf.transform.translation
        bearing = math.atan2(self._dock["y"] - t.y, self._dock["x"] - t.x)
        heading = yaw_of(tf.transform.rotation)
        turn = math.atan2(math.sin(bearing - heading), math.cos(bearing - heading))
        if abs(turn) < FACE_TOLERANCE_RAD:
            self._report(Step.FACE_DOCK, True)
            return
        goal = Spin.Goal()
        goal.target_yaw = turn
        goal.time_allowance = Duration(seconds=30.0).to_msg()
        self._send(self._spin, goal, Step.FACE_DOCK,
                   lambda r, status: (status == GoalStatus.STATUS_SUCCEEDED,
                                      f"spin status {status}"))

    def _do_dock(self) -> None:
        if self._home_unknown(Step.DOCK):
            return
        goal = DockRobot.Goal()
        # By pose, not by id: the pose may come from the site (converted at
        # runtime), which docking_server's own database cannot know.
        goal.use_dock_id = False
        goal.dock_type = self._dock_type
        goal.dock_pose = PoseStamped()
        goal.dock_pose.header.frame_id = "map"
        goal.dock_pose.header.stamp = self.get_clock().now().to_msg()
        goal.dock_pose.pose.position.x = self._dock["x"]
        goal.dock_pose.pose.position.y = self._dock["y"]
        goal.dock_pose.pose.orientation.z = math.sin(self._dock["yaw"] / 2.0)
        goal.dock_pose.pose.orientation.w = math.cos(self._dock["yaw"] / 2.0)
        goal.navigate_to_staging_pose = False      # staged and turned already
        goal.max_staging_time = float(self._max_staging_time)

        def judge(result, status):
            if status == GoalStatus.STATUS_SUCCEEDED and result.success:
                return True, ""
            code = DOCK_ERRORS.get(result.error_code, str(result.error_code)) \
                if result is not None else "no result"
            return False, f"{code} after {result.num_retries if result else '?'} retries"
        self._send(self._dock_client, goal, Step.DOCK, judge)

    def _do_undock(self) -> None:
        goal = UndockRobot.Goal()
        goal.dock_type = self._dock_type
        self._send(self._undock, goal, Step.UNDOCK,
                   lambda r, status: (status == GoalStatus.STATUS_SUCCEEDED and r.success,
                                      f"undock status {status}"))

    def _send(self, client: ActionClient, goal, step: Step, judge) -> None:
        """Send once the server is discovered (up to SERVER_WAIT_S), then
        report the outcome as `step`'s result.

        ⚠️ Waiting matters. MEASURED: called right after Nav2 came up, the
        first return_home failed in 1 s with "navigate_to_pose server not
        available" — the server was active, this client had simply not
        discovered it yet — and a robot that should have gone home locked
        itself in SAFE_STOP instead."""
        deadline = self._now() + SERVER_WAIT_S

        def accepted(future):
            handle = future.result()
            if not handle.accepted:
                self._report(step, False, f"{client._action_name} rejected the goal")
                return
            handle.get_result_async().add_done_callback(finished)

        def finished(future):
            response = future.result()
            ok, detail = judge(response.result, response.status)
            self._report(step, ok, detail)

        def poll():
            if self._sup.pending != step:          # superseded (SAFE_STOP) while waiting
                stop()
                return
            if client.server_is_ready():
                stop()
                client.send_goal_async(goal).add_done_callback(accepted)
            elif self._now() > deadline:
                stop()
                self._report(step, False, f"{client._action_name} server not available "
                                          f"after {SERVER_WAIT_S:g} s")

        def stop():
            timer.cancel()
            self.destroy_timer(timer)
        timer = self.create_timer(0.1, poll)
        poll()

    # ---- outputs ----------------------------------------------------------------
    def _publish_state(self) -> None:
        if self._sup.phase != self._published_phase:
            self._published_phase = self._sup.phase
            self._state_pub.publish(String(data=self._sup.phase.value))
            self._lock_pub.publish(Bool(data=self._sup.lock))

    def _diag_tick(self) -> None:
        sup = self._sup
        status = DiagnosticStatus(name="battery_supervisor: return to dock",
                                  hardware_id="battery")
        if sup.phase == Phase.SAFE_STOP:
            status.level, status.message = DiagnosticStatus.ERROR, f"SAFE_STOP: {sup.reason}"
        elif sup.phase in (Phase.RETURNING, Phase.DOCKING, Phase.UNDOCKING):
            status.level, status.message = DiagnosticStatus.WARN, f"{sup.phase.value}: {sup.reason}"
        elif sup.monitor.source != "soc":
            status.level = DiagnosticStatus.WARN
            status.message = (f"{sup.phase.value} on the voltage fallback (no SoC)"
                              if sup.monitor.source == "voltage"
                              else f"{sup.phase.value} with no battery reading")
        else:
            status.level, status.message = DiagnosticStatus.OK, f"{sup.phase.value}: {sup.reason}"
        status.values = [
            KeyValue(key="phase", value=sup.phase.value),
            KeyValue(key="level", value=sup.monitor.level.value),
            KeyValue(key="source", value=sup.monitor.source),
            KeyValue(key="value", value=f"{sup.monitor.value:.3f}"),
            KeyValue(key="nav_lock", value=str(sup.lock)),
            KeyValue(key="home", value=self._home_source),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BatterySupervisor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
