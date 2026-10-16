"""Harness for the localization tests: the whole stack against a known world.

    cd ros2 && source install/setup.bash
    python3 -m pytest src/robot_sim/test -q

Without ROS the world unit tests still run and these SKIP — world.py is
deliberately ROS-free, so nothing here may import rclpy at module level.

⚠️ `ros2 run`/`ros2 launch` do not forward signals to what they exec. Killing by
process group is the only cleanup that actually works; orphaned nodes from an
earlier run publish over the next one and silently corrupt its readings.
"""

import bisect
import contextlib
import math
import os
import shutil
import signal
import subprocess
import time

import pytest

SIM_STARTUP_S = 3.0
STACK_STARTUP_S = 8.0     # robot_state_publisher + bridge + EKF
# Nav2 is waited FOR, not slept past — see Nav2Stack. Generous because the
# thing being waited on is a five-server lifecycle transition on a box that may
# be running Gazebo at the same time; it returns the moment Nav2 is active, so
# a large number costs nothing except on a real failure.
NAV2_READY_S = 180.0
# ⚠️ ...but the FIRST bring-up gets less, and then one fresh try. MEASURED
# (locally, then twice in CI on 2026-10-16): the lifecycle manager stuck on
# "Configuring controller_server" forever, because controller_server logged
#   failed to send response to /controller_server/change_state (timeout):
#   client will not receive response
# (rmw_fastrtps answering before it has matched the client's reader) and
# nav2_util's change_state(transition) waits for that response with NO
# timeout. Nav2 never activates; waiting longer cannot help. A healthy bring-up
# here takes 2-30 s. So after this long the Nav2 launch is killed and started
# again, once, with a warning — the tests are about navigation and docking,
# not about this. ⚠️ The REAL ROBOT has the same hang and no such retry: see
# docs/handoff.md.
NAV2_FIRST_TRY_S = 75.0
NAV2_PATTERNS = (
    "nav2_controller/controller_server", "nav2_planner/planner_server",
    "nav2_behaviors/behavior_server", "nav2_bt_navigator/bt_navigator",
    "nav2_waypoint_follower/waypoint_follower", "opennav_docking/opennav_docking",
    "nav2_lifecycle_manager/lifecycle_manager", "hoverboard_bridge/nav_cmd_gate",
    "battery_manager/battery_supervisor", "hoverboard_bridge/nav2_watchdog",
)
# One is_active call's budget. It BLOCKS while the manager transitions its
# servers (measured: 4.3 s on an idle box) — but a call that times out
# mid-activation costs nothing: the next round asks again and a live manager
# answers. What a LONG budget costs is real: a request sent before the new
# manager was discovered (only the previous test's killed one was) is never
# answered, and every such round burns the whole budget. MEASURED in CI
# (PR #6, from the bag): Nav2's first command came 14, 32, 52, 72 s after the
# stack started — a base plus steps of ~20 s, which is what whole burned
# budgets look like (inferred: the fixture did not log its rounds then).
# On the 72 s one the gyro's
# 0.1 deg/s residual had turned the EKF 7 deg while the robot stood still, so
# the "straight line" test drove 0.69 m off it. 5 s keeps the misses cheap.
NAV2_CALL_S = 5.0
# Gazebo is waited FOR rather than slept past — see GazeboStack.
GAZEBO_READY_S = 60.0


class PoseTrack:
    """Timestamped pose history, so two topics can be read at ONE instant.

    ⚠️ NOT "the newest message on each topic". Those are not the same moment:
    /odom carries the pty round-trip's latency that /ground_truth does not, so
    at 1 m/s the two newest samples sit about 10 ms and 10 mm apart. That
    artefact is the same size as the wheel slip being measured and it has a
    plausible-looking sign. Interpolating both to one stamp moved the no-slip
    control case from -19 mm to -2 mm.
    """

    def __init__(self):
        self.stamps = []
        self.poses = []

    def add(self, message):
        stamp = message.header.stamp
        seconds = stamp.sec + stamp.nanosec * 1e-9
        # Sim time can repeat a stamp when /clock has not advanced between two
        # publications; a non-increasing key would break the bisect below.
        if self.stamps and seconds <= self.stamps[-1]:
            return
        pose = message.pose.pose
        quaternion = pose.orientation
        self.stamps.append(seconds)
        self.poses.append((
            pose.position.x,
            pose.position.y,
            math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                       1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)),
        ))

    def at(self, seconds):
        """Linearly interpolated pose, or None outside the recorded span."""
        index = bisect.bisect_left(self.stamps, seconds)
        if index == 0 or index >= len(self.stamps):
            return None
        before, after = self.stamps[index - 1], self.stamps[index]
        ratio = (seconds - before) / (after - before)
        first, second = self.poses[index - 1], self.poses[index]
        return tuple(a + ratio * (b - a) for a, b in zip(first, second))

    @property
    def newest(self):
        return self.stamps[-1] if self.stamps else None


@pytest.fixture
def pose_track():
    """A fresh PoseTrack. Shared so the interpolation has ONE definition.

    Both Gazebo test modules compare two odometry topics against each other,
    and getting that comparison right is subtler than it looks — see the class
    above. Two copies of it would be two chances to get it wrong in different
    ways, on the very measurement the physics backend exists to make.
    """
    return PoseTrack


def wait_for_message(rclpy, topic, message_type, timeout_s, qos=10):
    """Block until `topic` delivers one message, or return None.

    A readiness CHECK, not a sleep. A3's bug was a ROS-GZ bridge pointed at gz
    topics nobody published: it came up clean, logged its topic map, and
    delivered nothing forever. A fixed sleep cannot tell that apart from a slow
    machine, so every Gazebo test would have failed somewhere in the middle
    with a symptom that read like broken physics. Waiting for the first real
    message fails at setup instead, pointing at the bridge.
    """
    node = rclpy.create_node(f"wait_{abs(hash(topic)) % 100000}")
    received = []
    node.create_subscription(message_type, topic, received.append, qos)
    try:
        deadline = time.monotonic() + timeout_s
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        return received[0] if received else None
    finally:
        node.destroy_node()


@pytest.fixture(scope="session")
def ros():
    rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")
    # Every package's conftest defines its own session-scoped `ros`, so running
    # pytest across several test dirs at once would call init() twice and error.
    # Whoever gets there first owns the context; the rest just borrow it.
    owner = not rclpy.ok()
    if owner:
        rclpy.init()
    yield rclpy
    if owner:
        rclpy.shutdown()


# Executables that make up a stack. An orphan of any of these publishes on the
# same topics as the stack under test.
STACK_PATTERNS = (
    "lib/robot_sim/sim_node",
    "lib/hoverboard_bridge/hoverboard_bridge",
    "lib/hoverboard_bridge/fake_esp32",
    # publishes /cmd_vel: an orphan would drive the next stack's robot
    "lib/hoverboard_bridge/nav_cmd_gate",
    # the one publisher of /cmd_vel when robot.launch.py runs
    "lib/twist_mux/twist_mux",
    # cancel goals and hold nav_lock: an orphan would stop the next stack
    "lib/battery_manager/battery_supervisor",
    "lib/battery_manager/battery_monitor",
    "robot_localization/ekf_node",
    "robot_localization/navsat_transform_node",
    # latches /obstacle_map: an orphan's map would be the next stack's survey
    "lib/robot_missions/site_map_server",
)


def pids_matching(*patterns):
    found = subprocess.run(["pgrep", "-f", "|".join(patterns)],
                           capture_output=True, text=True)
    return [line for line in found.stdout.split() if line]


def stray_stack_pids():
    """Orphaned stack nodes. There must never be any when a stack starts.

    ⚠️ This file has warned in prose since A1 that "orphaned nodes from an
    earlier run publish over the next one and silently corrupt its readings",
    and nothing enforced it. Silently is the operative word: a second
    hoverboard_bridge on /odom does not error, it interleaves — and the EKF
    downstream then integrates two robots' worth of wheel data.
    ⚠️ A sentence here used to say this was "measured as 1.52 m of EKF error".
    It was not: it was the hypothesis this guard was written to test, put down
    in measurement language before the experiment — which then gave 1.368 m
    with this guard silent. The 1.52 m runs were gz-sim 8.11.0's angular
    velocity spikes (test_gazebo_localization.py). The guard is still right;
    the number was never its evidence.
    Killing a process group is also not instant: the group dies, the grandchild
    node processes are reaped a moment later, so close() has to WAIT for them
    rather than assume.
    """
    return pids_matching(*STACK_PATTERNS)


def stray_gazebo_pids():
    """Any `gz sim` server already running. There may only ever be one.

    ⚠️ A gz server is a SINGLETON on this machine, and a second one is not an
    error — it is a quiet disaster. Both answer on /world/empty/..., so a spawn
    request lands in one world while the robot drives in the other, the ROS-GZ
    bridge subscribes to whichever it found, and the readings come back looking
    like broken physics. Measured while getting this wrong: the obstacle was
    spawned into one server, the robot drove straight past where it should have
    been and the test reported "stopped at x=4.93, expected 2.15".
    """
    return pids_matching("^gz sim")


class SimStack:
    """sim_node (the world) + the real bringup stack driving it.

    use_imu:=false on purpose: sim_node publishes /imu/data itself, standing in
    for what mpu6050_driver would emit. Launching the driver too would fight it
    for the topic.

    Subclasses vary the bringup via LAUNCH_ARGS rather than launching a second
    robot.launch.py: two of them would put two bridges on one pty and two
    ekf_locals on one topic, which does not error — it just quietly makes every
    reading meaningless.
    """

    LAUNCH_ARGS = ("use_localization:=true", "use_imu:=false", "use_gps:=false")
    LINK = "/tmp/fake_esp32"

    def __init__(self, *sim_args):
        self._procs = []
        # ⚠️ ANYTHING THAT RAISES DURING BRING-UP MUST STILL CLEAN UP. The
        # fixtures register a stack for teardown only once the constructor has
        # RETURNED, so a constructor that throws leaves every process it had
        # already spawned running forever — a whole orphaned sim_node, bridge,
        # EKF and Nav2, publishing over every test that follows.
        # That is not hypothetical: it happened the day _await_nav2 replaced a
        # sleep, because a sleep cannot fail and a readiness check can. One
        # Nav2 stack failed to activate, leaked, and took six later tests down
        # with it — including two in-process obstacle tests that received the
        # orphan's latched empty map instead of their own.
        stray = stray_stack_pids()
        assert not stray, (
            f"stack nodes from an earlier test are still running (pids "
            f"{stray}). They publish on the same topics as the stack about to "
            "start, and the result is not an error — it is quietly wrong "
            "numbers. Find the fixture that did not close.")
        with self._closing_on_failure():
            self._start_world(sim_args)
            self._spawn("ros2", "launch", "robot_bringup", "robot.launch.py",
                        f"esp32_port:={self.LINK}", *self.launch_args())
            time.sleep(STACK_STARTUP_S)

    # sim_node publishes /obstacle_map; the robot's site_map_server must not
    # publish a second one (robot.launch.py). SiteStack turns this around.
    SITE_ARGS = ("use_site_map:=false",)

    def launch_args(self):
        return (*self.LAUNCH_ARGS, *self.SITE_ARGS)

    @contextlib.contextmanager
    def _closing_on_failure(self):
        try:
            yield
        except BaseException:
            self.close()
            raise

    def _start_world(self, sim_args):
        """Whatever the ESP32 simulator drives. GazeboStack puts physics here."""
        self._spawn("ros2", "run", "robot_sim", "sim_node", *sim_args)
        time.sleep(SIM_STARTUP_S)

    def _spawn(self, *cmd):
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, start_new_session=True)
        self._procs.append(proc)
        return proc

    def close(self):
        for proc in reversed(self._procs):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=15)
        self._await_shutdown()

    def _await_shutdown(self):
        """Wait for the stack's node processes to actually be gone.

        killpg signals the group and proc.wait() reaps the launcher, but the
        node processes it exec'd are reaped a moment later — so returning from
        close() is not the same as the graph being clean. The next stack's
        assertion would then fire on processes that were merely mid-exit.
        """
        deadline = time.monotonic() + 30.0
        while stray_stack_pids() and time.monotonic() < deadline:
            time.sleep(0.25)


@pytest.fixture
def sim_stack(ros):
    made = []

    def _make(*sim_args):
        stack = SimStack(*sim_args)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class GlobalStack(SimStack):
    """The sim stack with the GLOBAL half switched on: GPS, madgwick, ekf_global.

    SimStack runs use_gps:=false, which leaves ekf_global, navsat_transform and
    imu_filter_madgwick out entirely — so for a long time nothing under test ever
    looked at the robot's ABSOLUTE heading, the one thing the magnetometer is for.
    A mirrored field in the simulator survived a full green suite because of it,
    and only turned up when the fused yaw was measured against ground truth by
    hand. This class exists so that measurement is automated.

    use_mag:=false for the same reason as use_imu: sim_node publishes /imu/mag
    itself, so the real driver would only fight it for the topic.
    """

    LAUNCH_ARGS = ("use_localization:=true", "use_imu:=false", "use_mag:=false",
                   "use_gps:=true", "use_imu_filter:=true")


@pytest.fixture
def global_stack(ros):
    made = []

    def _make(*sim_args):
        stack = GlobalStack(*sim_args)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class SiteStack(GlobalStack):
    """GlobalStack whose survey map comes from a SITE FILE, the way the real
    robot's does: site.geojson -> site_map_server -> navsat_transform's
    /fromLLArray -> /obstacle_map. sim_node's own map is switched off, so there
    is still exactly one publisher.

    `features` are GeoJSON features (docs/missions.md); `sim_args` are "-p",
    "name:=value" pairs for sim_node.
    """

    def __init__(self, workdir, features, *sim_args):
        self.site_file = write_site(workdir, features)
        super().__init__("--ros-args", "-p", "publish_obstacle_map:=false", *sim_args)

    def launch_args(self):
        return (*self.LAUNCH_ARGS, "use_site_map:=true", f"site_file:={self.site_file}")


def write_site(workdir, features):
    import json

    path = os.path.join(workdir, "site.geojson")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"type": "FeatureCollection", "features": list(features)}, handle)
    return path


@pytest.fixture
def site_stack(ros, tmp_path):
    made = []

    def _make(features, *sim_args):
        stack = SiteStack(str(tmp_path), features, *sim_args)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class Nav2Stack(SimStack):
    """The sim stack plus Nav2, with map pinned to odom.

    Nav2 plans in `map`, which normally only exists once ekf_global and
    navsat_transform are running — and those need an absolute heading we do not
    have (see test_nav2.py). Pinning map->odom to identity isolates the
    navigation layer from that gap, so this tests the planner, the controller and
    the /cmd_vel -> protocol -> wheels chain on their own terms.

    The trade-off is honest: map == odom means the goal is expressed in a frame
    that drifts with the wheels. Fine over the tens of metres of a test, and not
    what the real robot will do outdoors.
    """

    def __init__(self, *sim_args):
        super().__init__(*sim_args)
        with self._closing_on_failure():
            self._pin_map()
            self._start_nav2()

    def _pin_map(self):
        self._spawn("ros2", "run", "tf2_ros", "static_transform_publisher",
                    "--frame-id", "map", "--child-frame-id", "odom")

    def _start_nav2(self):
        nav2 = self._spawn("ros2", "launch", "robot_bringup", "nav2.launch.py",
                           *self.nav2_args())
        try:
            self._await_nav2(NAV2_FIRST_TRY_S)
        except AssertionError as stuck:
            import warnings
            warnings.warn(f"Nav2 bring-up hung, restarting it once: {stuck}")
            self._procs.remove(nav2)
            os.killpg(os.getpgid(nav2.pid), signal.SIGKILL)
            nav2.wait(timeout=15)
            deadline = time.monotonic() + 30.0
            while pids_matching(*NAV2_PATTERNS) and time.monotonic() < deadline:
                time.sleep(0.25)
            self._spawn("ros2", "launch", "robot_bringup", "nav2.launch.py",
                        *self.nav2_args())
            self._await_nav2(NAV2_READY_S)

    def nav2_args(self):
        return ()

    def _await_nav2(self, ready_s=NAV2_READY_S):
        """Block until the lifecycle manager says every server is ACTIVE.

        ⚠️ This replaced a fixed 20 s sleep, and the sleep was a real flake. A3c
        trap 5 already knew why: Nav2 opens its action server in CONFIGURE but
        rejects goals until ACTIVE, and a client cannot tell those apart, so a
        stack that had not finished activating produced "Nav2 rejected the
        goal" — which reads like a navigation bug and is really a race. It only
        ever showed with the whole suite running, and it came back the moment
        A3e made the suite longer: two Gazebo worlds now come up before Nav2
        does, and 20 s of sleep plus 45 s of goal retries stopped being enough.
        Two of four Nav2 tests failed in the full suite while all four passed
        when the file ran alone.

        `is_active` is an actual readiness answer rather than a guess about how
        long activation takes. MEASURED: the service appears the instant the
        manager starts (so its existence means nothing), and the call then
        BLOCKS while the manager walks the servers through their transitions —
        4.3 s on an idle box — before answering True. So the timeout has to
        cover the whole activation, and a slow box just answers later.
        """
        import rclpy
        from std_srvs.srv import Trigger

        deadline = time.monotonic() + ready_s
        rounds = unanswered = reported_inactive = 0
        while time.monotonic() < deadline:
            # ⚠️ A FRESH NODE AND CLIENT EVERY ROUND. The previous test's Nav2
            # was killed, not shut down, so its lifecycle manager can linger in
            # DDS discovery under the same name and service; a client that
            # binds to that dead endpoint sends requests nobody will ever
            # answer, and reusing it means every retry goes to the same corpse.
            # Failing here always looked like "Nav2 never activated" while the
            # new Nav2 was perfectly healthy — and it hit the SECOND Nav2 test
            # of a run, never the first, which is the tell.
            rounds += 1
            node = rclpy.create_node(f"nav2_ready_wait_{os.getpid()}_{rounds}")
            client = node.create_client(
                Trigger, "/lifecycle_manager_navigation/is_active")
            try:
                while not client.service_is_ready() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.2)
                if not client.service_is_ready():
                    continue
                future = client.call_async(Trigger.Request())
                call_deadline = min(deadline, time.monotonic() + NAV2_CALL_S)
                while not future.done() and time.monotonic() < call_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not future.done():
                    unanswered += 1
                    continue
                result = future.result()
                if result is not None and result.success:
                    return
                reported_inactive += 1
                time.sleep(1.0)
            finally:
                node.destroy_node()

        # ⚠️ SAY WHICH FAILURE IT WAS. "Nav2 never came up" covers two very
        # different faults — a manager that answered "not active" (a real
        # startup problem: read nav2.launch.py's output) and one that never
        # answered at all (discovery, not Nav2). Guessing between them cost a
        # full suite run.
        raise AssertionError(
            f"Nav2 was not ACTIVE within {ready_s:.0f} s: {rounds} rounds, "
            f"{reported_inactive} answered 'not active', {unanswered} never "
            f"answered. Mostly 'not active' means a server would not transition "
            f"— read nav2.launch.py's output. Mostly unanswered means the "
            f"service was never really reachable.")


@pytest.fixture
def nav2_stack(ros):
    made = []

    def _make(*sim_args):
        stack = Nav2Stack(*sim_args)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class SiteNav2Stack(Nav2Stack):
    """Nav2 on the REAL `map`: GPS, madgwick, ekf_global and navsat_transform
    (GlobalStack's half), with the survey map from a site file (SiteStack's).

    Nav2Stack pins map == odom to keep GPS out of the navigation tests; this
    one does not, because the thing under test is exactly that chain: a
    lat/lon drawn on a map at a desk, through navsat_transform, into the
    costmap the planner routes around.
    """

    LAUNCH_ARGS = GlobalStack.LAUNCH_ARGS

    def __init__(self, workdir, features, *sim_args):
        self.site_file = write_site(workdir, features)
        super().__init__("--ros-args", "-p", "publish_obstacle_map:=false", *sim_args)

    def launch_args(self):
        return (*self.LAUNCH_ARGS, "use_site_map:=true", f"site_file:={self.site_file}")

    def _pin_map(self):
        """No pin: ekf_global publishes map->odom."""


@pytest.fixture
def site_nav2_stack(ros, tmp_path):
    made = []

    def _make(features, *sim_args):
        stack = SiteNav2Stack(str(tmp_path), features, *sim_args)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class DockStack(Nav2Stack):
    """The Nav2 stack with a charging station and a pack that drains in
    seconds (SP3-SP5).

    Three things differ from Nav2Stack, all through files and arguments the
    real launch files take, not test-only switches:
      * fake_battery:=true — battery_monitor's fake INA228 carries the current
        robot_sim's pack model says flows (sim/battery_current).
      * battery_config — a copy of battery_manager.yaml with a tiny capacity on
        BOTH sides (the sim's pack and the monitor's belief) and the test's
        thresholds for the supervisor.
      * dock_database — the test's own docks.yaml, which may deliberately
        disagree with where sim_node really put the station (survey error).
    """

    def __init__(self, workdir, sim_args, monitor=None, supervisor=None, dock_pose=None):
        import yaml
        from ament_index_python.packages import get_package_share_directory

        source = os.path.join(get_package_share_directory("robot_bringup"),
                              "config", "battery_manager.yaml")
        with open(source, encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        config["battery_monitor"]["ros__parameters"].update(monitor or {})
        config["battery_supervisor"]["ros__parameters"].update(supervisor or {})
        self._battery_config = os.path.join(workdir, "battery_manager.yaml")
        with open(self._battery_config, "w", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle)
        self._dock_database = os.path.join(workdir, "docks.yaml")
        with open(self._dock_database, "w", encoding="utf-8") as handle:
            yaml.safe_dump({"docks": {"home": {
                "type": "charging_dock", "frame": "map",
                "pose": [float(v) for v in dock_pose]}}}, handle)
        super().__init__(*sim_args)

    def launch_args(self):
        return (*super().launch_args(), "fake_battery:=true",
                f"battery_config:={self._battery_config}")

    def nav2_args(self):
        return (f"battery_config:={self._battery_config}",
                f"dock_database:={self._dock_database}")


@pytest.fixture
def dock_stack(ros, tmp_path):
    made = []

    def _make(sim_args, **kwargs):
        stack = DockStack(str(tmp_path), sim_args, **kwargs)
        made.append(stack)
        return stack

    yield _make
    for stack in made:
        stack.close()


class GazeboStack(SimStack):
    """The same stack, but with Gazebo physics under the simulated ESP32.

    Three processes instead of two, because the physics comes up first:

        gazebo.launch.py   gz sim (headless) + the model + the ros_gz bridge
        sim_node           backend:=gazebo   — the ESP32 brain, wired to Gazebo
        robot.launch.py    the real robot stack, on sim time

    ⚠️ use_sim_time:=true EVERYWHERE, including sim_node. Gazebo runs on /clock;
    a node left on the wall clock computes every dt against a different clock
    from the one the physics advanced, and nothing errors — the velocities and
    accelerations just come out scaled. Both halves of that were missing before
    A3 was finished: /clock was not bridged AND nothing set use_sim_time, so
    neither absence could show up as a hang.

    ⚠️ Its own pty (LINK), and it waits for real messages instead of sleeping —
    a bridge wired to nothing looks exactly like a slow machine to a sleep.
    """

    LAUNCH_ARGS = ("use_localization:=true", "use_imu:=false", "use_gps:=false",
                   "use_sim_time:=true")
    LINK = "/tmp/fake_esp32_gazebo"

    def _await_shutdown(self):
        """Also wait for the gz server. Killing the launch process group is not
        the same as gz being gone.

        The server takes a moment to die, and the next stack's assertion that
        no gz is running would fire on a corpse. Waited for rather than slept
        past, for the usual reason.
        """
        super()._await_shutdown()
        deadline = time.monotonic() + 30.0
        while stray_gazebo_pids() and time.monotonic() < deadline:
            time.sleep(0.5)
        assert not stray_gazebo_pids(), (
            f"gz server {stray_gazebo_pids()} outlived its launch process group")

    def _start_world(self, sim_args):
        import rclpy
        from nav_msgs.msg import Odometry

        stray = stray_gazebo_pids()
        assert not stray, (
            f"a gz server is already running (pids {stray}) — two of them share "
            "the /world/empty topics and mix into one unreadable world. A "
            "previous test's stack did not shut down.")

        self._spawn("ros2", "launch", "robot_bringup", "gazebo.launch.py")
        # Proof that gz, the model spawn and the bridge ALL work: ground truth
        # only flows if every one of them did. This is the check the A3 bug
        # would have failed instantly.
        if wait_for_message(rclpy, "/gazebo/ground_truth", Odometry,
                            GAZEBO_READY_S) is None:
            raise AssertionError(
                "no /gazebo/ground_truth within {:.0f} s — Gazebo did not "
                "start, the model did not spawn, or bridge.yaml names gz "
                "topics that nothing publishes".format(GAZEBO_READY_S))

        self._spawn("ros2", "run", "robot_sim", "sim_node", "--ros-args",
                    "-p", "backend:=gazebo",
                    "-p", "use_sim_time:=true",
                    "-p", f"link:={self.LINK}",
                    *sim_args)
        # sim_node republishes Gazebo's truth as /ground_truth; waiting on it
        # proves the backend subscribed to the topic the bridge actually emits.
        if wait_for_message(rclpy, "/ground_truth", Odometry,
                            GAZEBO_READY_S) is None:
            raise AssertionError(
                "sim_node published no /ground_truth — check its log for a "
                "backend or obstacle-spawn failure")


@pytest.fixture(scope="module")
def gazebo(ros):
    """Bring up a Gazebo stack, reusing the running one when it would be identical.

    ONE stack at a time, by construction: asking for different sim arguments
    tears the previous world down before building the next. That is not just
    tidiness — see stray_gazebo_pids above.

    Reuse is what keeps this affordable. Gazebo, the model, the bridge, sim_node
    and the robot stack take about half a minute to come up, and the physics
    tests all measure DELTAS between two rest states on 100 x 100 m of empty
    ground, so sharing one obstacle-free world between them is safe. Tests that
    need obstacles ask for them and pay for their own world.

        gazebo()                                    the empty world
        gazebo("-p", "obstacle_centers:=[3.0, 0.0]", ...)   its own world
    """
    if shutil.which("gz") is None:
        pytest.skip("Gazebo (`gz`) is not installed")
    state = {"stack": None, "args": None}

    def _drop():
        if state["stack"] is not None:
            state["stack"].close()
            state["stack"] = None
            state["args"] = None

    def _make(*sim_args):
        if state["stack"] is not None and state["args"] == sim_args:
            return state["stack"]
        _drop()
        state["stack"] = GazeboStack(*sim_args)
        state["args"] = sim_args
        return state["stack"]

    yield _make
    _drop()
