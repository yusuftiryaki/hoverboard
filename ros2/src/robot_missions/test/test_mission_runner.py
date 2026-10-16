"""mission_runner in-process, against stand-ins for Nav2, navsat_transform and
battery_supervisor. Skips without ROS, ~15 s.

runner_policy's tests prove the decisions; this proves the wiring: that a
mission JSON reaches the planner, that the goals Nav2 receives are the
waypoints in `map`, that the supervisor's cancel is not answered with a new
goal, that the runner asks a full robot to undock and carries on with the
interrupted waypoint, and that a restart comes back PAUSED at the saved index.
"""

import json
import math
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from action_msgs.msg import GoalInfo                               # noqa: E402
from action_msgs.srv import CancelGoal                             # noqa: E402
from geographic_msgs.msg import GeoPoint                           # noqa: E402
from geometry_msgs.msg import Point                                # noqa: E402
from nav2_msgs.action import NavigateToPose                        # noqa: E402
from rclpy.action import ActionServer, CancelResponse              # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup           # noqa: E402
from rclpy.executors import MultiThreadedExecutor                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile                 # noqa: E402
from robot_localization.srv import FromLLArray, ToLL               # noqa: E402
from std_msgs.msg import String                                    # noqa: E402
from std_srvs.srv import Trigger                                   # noqa: E402
from geometry_msgs.msg import TransformStamped                     # noqa: E402
from tf2_ros import StaticTransformBroadcaster                     # noqa: E402

from robot_missions.mission_runner import MissionRunner            # noqa: E402
from robot_missions_interfaces.srv import StartMission             # noqa: E402

LAT0, LON0 = 41.0, 29.0
DLAT_10M = 8.993216e-05
DLON_10M = 1.191613e-04
R = 6371000.0
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# The route, in map metres: east 10, then north 10, then back west.
ROUTE = [(0, 0), (10, 0), (10, 10), (0, 10)]


def ll(x, y):
    return [LON0 + DLON_10M * x / 10, LAT0 + DLAT_10M * y / 10]


@pytest.fixture
def context():
    own = not rclpy.ok()
    if own:
        rclpy.init()
    yield
    if own:
        rclpy.shutdown()


class World:
    """Fake Nav2 + navsat_transform + battery_supervisor, one node."""

    def __init__(self):
        self.node = rclpy.create_node("fake_world")
        group = ReentrantCallbackGroup()
        self.goals = []                 # (x, y) per goal received, in order
        self.hold = False               # True: goals stay active until released/canceled
        # Goals beyond this many are held even with hold False (None = none).
        self.free = None
        self.resume_calls = 0
        self._lock = threading.Lock()
        self.node.create_service(FromLLArray, "fromLLArray", self._from_ll, callback_group=group)
        self.node.create_service(ToLL, "toLL", self._to_ll, callback_group=group)
        self.node.create_service(Trigger, "battery_supervisor/resume", self._resume,
                                 callback_group=group)
        self._state = self.node.create_publisher(String, "battery_supervisor/state", LATCHED)
        ActionServer(self.node, NavigateToPose, "navigate_to_pose", self._execute,
                     cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                     callback_group=group)
        self._cancel_all = self.node.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal", callback_group=group)

    def robot_at(self, x, y):
        """map -> base_link, so the runner has a start pose to plan from."""
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_link"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.w = 1.0
        self._tf = StaticTransformBroadcaster(self.node)
        self._tf.sendTransform(tf)

    def supervisor(self, phase):
        self._state.publish(String(data=phase))

    def cancel_all(self):
        """What battery_supervisor's CANCEL_MISSIONS does: a zero GoalInfo."""
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        self._cancel_all.call_async(request)

    def _from_ll(self, request, response):
        for p in request.ll_points:
            response.map_points.append(Point(
                x=math.radians(p.longitude - LON0) * R * math.cos(math.radians(LAT0)),
                y=math.radians(p.latitude - LAT0) * R))
        return response

    def _to_ll(self, request, response):
        x, y = request.map_point.x, request.map_point.y
        response.ll_point = GeoPoint(
            latitude=LAT0 + math.degrees(y / R),
            longitude=LON0 + math.degrees(x / (R * math.cos(math.radians(LAT0)))))
        return response

    def _resume(self, _request, response):
        self.resume_calls += 1
        response.success, response.message = True, "undocking"
        return response

    def _execute(self, goal_handle):
        position = goal_handle.request.pose.pose.position
        with self._lock:
            self.goals.append((round(position.x, 2), round(position.y, 2)))
            held = self.free is not None and len(self.goals) > self.free
        while (self.hold or held) and not goal_handle.is_cancel_requested:
            held = held and self.free is not None
            time.sleep(0.02)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


class Harness:
    def __init__(self, tmp_path, **params):
        self.site = tmp_path / "site.geojson"
        self.site.write_text(json.dumps({"type": "FeatureCollection", "features": [
            {"type": "Feature",
             "geometry": {"type": "LineString", "coordinates": [ll(*p) for p in ROUTE]},
             "properties": {"kind": "route", "name": "tur"}},
            {"type": "Feature",
             "geometry": {"type": "Polygon", "coordinates": [[
                 ll(0, 0), ll(4, 0), ll(4, 2), ll(0, 2), ll(0, 0)]]},
             "properties": {"kind": "area", "name": "bahce"}},
        ]}), encoding="utf-8")
        self.state_file = tmp_path / "state" / "mission_state.json"
        self.params = {"site_file": str(self.site), "state_file": str(self.state_file),
                       "cancel_grace_s": 1.0, **params}
        self.world = World()
        self.client = rclpy.create_node("runner_client")
        self.status = None
        self.client.create_subscription(String, "mission_runner/status",
                                        lambda m: setattr(self, "status", json.loads(m.data)),
                                        LATCHED)
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.world.node, self.client):
            self.executor.add_node(node)
        self.runner = None
        self.start_runner()
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    def start_runner(self):
        self.runner = MissionRunner(parameter_overrides=[
            rclpy.parameter.Parameter(k, value=v) for k, v in self.params.items()])
        self.executor.add_node(self.runner)

    def stop_runner(self):
        self.executor.remove_node(self.runner)
        self.runner.destroy_node()
        self.runner = None

    def call(self, srv_type, name, request, timeout=5.0):
        client = self.client.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f"{name} not up"
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.client.destroy_client(client)
        assert future.done(), f"{name} did not answer"
        return future.result()

    def start(self, mission):
        request = StartMission.Request()
        request.mission = json.dumps(mission)
        return self.call(StartMission, "mission_runner/start", request)

    def trigger(self, name):
        return self.call(Trigger, f"mission_runner/{name}", Trigger.Request())

    def wait(self, predicate, timeout=8.0, what="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError(f"timed out waiting for {what}; status {self.status}, "
                             f"goals {self.world.goals}")

    def close(self):
        # Release any held goal first: its execute thread would otherwise keep
        # the executor's shutdown waiting forever.
        self.world.hold, self.world.free = False, None
        self.executor.shutdown()
        for node in (self.runner, self.world.node, self.client):
            if node is not None:
                node.destroy_node()


@pytest.fixture
def harness(context, tmp_path):
    made = []

    def _make(**params):
        h = Harness(tmp_path, **params)
        made.append(h)
        return h

    yield _make
    for h in made:
        h.close()


def state_is(h, name):
    return lambda: h.status is not None and h.status["state"] == name


def test_a_route_mission_drives_its_waypoints_in_map_and_finishes(harness):
    h = harness()
    h.world.supervisor("NORMAL")
    h.wait(lambda: h.runner._policy.supervisor == "NORMAL", what="supervisor state")
    answer = h.start({"type": "route", "route": "tur"})
    assert answer.accepted, answer.message
    h.wait(state_is(h, "DONE"), what="DONE")
    assert h.world.goals == [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    assert h.status["skipped"] == []
    persisted = json.loads(h.state_file.read_text())
    assert persisted["state"] == "DONE"


@pytest.mark.parametrize("mission, words", [
    ({"type": "route", "route": "yok"}, "no route named"),
    ({"type": "coverage", "area": "bahce", "params": {"overlab": 0.3}}, "no parameter"),
])
def test_a_bad_mission_is_refused_with_the_reason(harness, mission, words):
    h = harness()
    h.world.supervisor("NORMAL")
    h.wait(lambda: h.runner._policy.supervisor == "NORMAL", what="supervisor state")
    answer = h.start(mission)
    assert not answer.accepted and words in answer.message


def test_start_is_refused_while_the_robot_is_charging(harness):
    h = harness()
    h.world.supervisor("CHARGING")
    h.wait(lambda: h.runner._policy.supervisor == "CHARGING", what="supervisor state")
    answer = h.start({"type": "route", "route": "tur"})
    assert not answer.accepted and "CHARGING" in answer.message


def test_the_trip_to_the_charger_and_back(harness):
    h = harness()
    h.world.supervisor("NORMAL")
    h.wait(lambda: h.runner._policy.supervisor == "NORMAL", what="supervisor state")
    h.world.hold = True
    assert h.start({"type": "route", "route": "tur"}).accepted
    h.wait(lambda: len(h.world.goals) == 1, what="first goal")

    # LOW: the supervisor goes RETURNING and cancels every mission goal.
    h.world.supervisor("RETURNING")
    h.world.cancel_all()
    h.wait(state_is(h, "WAITING_DOCK"), what="WAITING_DOCK")
    time.sleep(1.5)                     # longer than cancel_grace_s
    # ⚠️ No goal may follow the cancel: it would preempt the trip home.
    assert len(h.world.goals) == 1 and h.status["state"] == "WAITING_DOCK"

    for phase in ("DOCKING", "CHARGING"):
        h.world.supervisor(phase)
    time.sleep(0.5)
    assert h.world.resume_calls == 0    # never undock a robot that is not full
    h.world.supervisor("DOCKED_IDLE")
    h.wait(lambda: h.world.resume_calls == 1, what="the undock request")
    h.world.supervisor("UNDOCKING")
    h.world.hold = False
    h.world.supervisor("NORMAL")
    h.wait(state_is(h, "DONE"), what="DONE after the dock")
    # The interrupted waypoint is driven again, then the rest.
    assert h.world.goals == [(0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    assert h.world.resume_calls == 1


@pytest.mark.parametrize("with_pose", [True, False])
def test_a_restart_comes_back_paused_at_the_saved_waypoint(harness, with_pose):
    # with_pose: the start goes to lat/lon through /toLL and back through
    # /fromLLArray on resume. Without: the lap was planned with no start, so a
    # re-plan without one is the same plan and the index still holds.
    h = harness()
    if with_pose:
        h.world.robot_at(-1.0, -1.0)
    h.world.supervisor("NORMAL")
    h.wait(lambda: h.runner._policy.supervisor == "NORMAL", what="supervisor state")
    h.world.free = 1                    # waypoint 0 succeeds, waypoint 1 is held
    assert h.start({"type": "route", "route": "tur"}).accepted
    h.wait(lambda: len(h.world.goals) >= 2, what="second goal")
    h.wait(lambda: json.loads(h.state_file.read_text())["index"] == 1, what="index 1 saved")
    saved = json.loads(h.state_file.read_text())
    assert saved["mission"] == {"type": "route", "route": "tur"}
    assert saved["planned_with_start"] is with_pose
    if with_pose:
        h.wait(lambda: json.loads(h.state_file.read_text())["start_ll"] is not None,
               what="the start in lat/lon")

    h.stop_runner()                     # the process dies mid-goal
    h.world.free = None
    h.status = None
    h.start_runner()
    h.wait(state_is(h, "PAUSED"), what="PAUSED after the restart")
    time.sleep(0.5)
    goals_before = len(h.world.goals)
    assert goals_before == 2            # it did not drive off by itself

    assert h.trigger("resume").success
    h.wait(state_is(h, "DONE"), what="DONE after resume")
    assert h.world.goals[goals_before:] == [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


def test_pause_and_resume(harness):
    h = harness()
    h.world.supervisor("NORMAL")
    h.wait(lambda: h.runner._policy.supervisor == "NORMAL", what="supervisor state")
    h.world.hold = True
    assert h.start({"type": "route", "route": "tur"}).accepted
    h.wait(lambda: len(h.world.goals) == 1, what="first goal")
    assert h.trigger("pause").success
    h.wait(state_is(h, "PAUSED"), what="PAUSED")
    time.sleep(1.5)
    assert len(h.world.goals) == 1
    h.world.hold = False
    assert h.trigger("resume").success
    h.wait(state_is(h, "DONE"), what="DONE")
    assert h.world.goals == [(0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
