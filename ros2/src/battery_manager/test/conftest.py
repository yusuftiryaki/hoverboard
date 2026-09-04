"""Process harness for battery_monitor ROS integration tests."""

import os
import signal
import subprocess
import time

import pytest

STARTUP_S = 2.5


@pytest.fixture(scope="session")
def ros():
    rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")
    owner = not rclpy.ok()
    if owner:
        rclpy.init()
    yield rclpy
    if owner:
        rclpy.shutdown()


class MonitorHarness:
    def __init__(self, *params):
        args = []
        for parameter in params:
            args += ["-p", parameter]
        command = ["ros2", "run", "battery_manager", "battery_monitor", "--ros-args"] + args
        self._proc = subprocess.Popen(
            [
                "bash",
                "-lc",
                "source /workspaces/hoverboard/ros2/install/setup.bash && exec "
                + " ".join(command),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        time.sleep(STARTUP_S)

    def stop(self):
        if self._proc.poll() is not None:
            return
        os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        try:
            self._proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)


@pytest.fixture
def monitor_harness():
    harnesses = []

    def spawn(*params):
        harness = MonitorHarness(*params)
        harnesses.append(harness)
        return harness

    yield spawn
    for harness in harnesses:
        harness.stop()