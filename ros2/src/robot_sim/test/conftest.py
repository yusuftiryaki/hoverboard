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
import signal
import subprocess
import time

import pytest

SIM_STARTUP_S = 3.0
STACK_STARTUP_S = 8.0     # robot_state_publisher + bridge + EKF
NAV2_STARTUP_S = 20.0     # five lifecycle servers, costmaps included


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
    """

    def __init__(self, *sim_args):
        self._procs = []
        self._spawn("ros2", "run", "robot_sim", "sim_node", *sim_args)
        time.sleep(SIM_STARTUP_S)
        self._spawn("ros2", "launch", "robot_bringup", "robot.launch.py",
                    "esp32_port:=/tmp/fake_esp32", "use_localization:=true",
                    "use_imu:=false", "use_gps:=false")
        time.sleep(STACK_STARTUP_S)

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
