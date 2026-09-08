"""Harness for the localization tests: the whole stack against a known world.

    cd ros2 && source install/setup.bash
    python3 -m pytest src/robot_sim/test -q

Without ROS the world unit tests still run and these SKIP — world.py is
deliberately ROS-free, so nothing here may import rclpy at module level.

⚠️ `ros2 run`/`ros2 launch` do not forward signals to what they exec. Killing by
process group is the only cleanup that actually works; orphaned nodes from an
earlier run publish over the next one and silently corrupt its readings.
"""

import os
import shutil
import signal
import subprocess
import time

import pytest

SIM_STARTUP_S = 3.0
STACK_STARTUP_S = 8.0     # robot_state_publisher + bridge + EKF
NAV2_STARTUP_S = 20.0     # five lifecycle servers, costmaps included
# Gazebo is waited FOR rather than slept past — see GazeboStack.
GAZEBO_READY_S = 60.0


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
        self._start_world(sim_args)
        self._spawn("ros2", "launch", "robot_bringup", "robot.launch.py",
                    f"esp32_port:={self.LINK}", *self.LAUNCH_ARGS)
        time.sleep(STACK_STARTUP_S)

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
        """Nothing to wait for unless a subclass owns a singleton resource."""


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
        self._spawn("ros2", "run", "tf2_ros", "static_transform_publisher",
                    "--frame-id", "map", "--child-frame-id", "odom")
        self._spawn("ros2", "launch", "robot_bringup", "nav2.launch.py")
        # Nav2's lifecycle manager has to walk five servers through
        # configure/activate, and the costmaps are the slow part.
        time.sleep(NAV2_STARTUP_S)


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
    found = subprocess.run(["pgrep", "-f", "^gz sim"],
                           capture_output=True, text=True)
    return [line for line in found.stdout.split() if line]


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
        """Killing the launch process group is not the same as gz being gone.

        The server takes a moment to die, and the next stack's assertion that
        no gz is running would fire on a corpse. Waited for rather than slept
        past, for the usual reason.
        """
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
