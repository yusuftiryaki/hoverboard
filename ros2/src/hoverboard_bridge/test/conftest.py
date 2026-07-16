"""Harness for the integration tests: spawn real nodes, drive them, clean up.

Run them with the workspace sourced:

    cd ros2 && source install/setup.bash
    python3 -m pytest src/hoverboard_bridge/test -q

Without ROS on the path the integration tests SKIP and the protocol unit tests
still run — protocol.py is deliberately ROS-free, and its tests must stay
runnable in a bare interpreter. So nothing here may import rclpy at module
level: that would break collection for the whole package.

⚠️ `ros2 run` does NOT forward signals to the node it execs. Terminating it
leaves the real node orphaned, still publishing, and quietly corrupting the next
test's readings — that failure cost real debugging time here. Every process
therefore gets its own session and is killed by process group.
"""

import os
import signal
import subprocess
import time

import pytest

STARTUP_S = 2.5
BUMP_FILE = "/tmp/fake_esp32.bump"


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


class Harness:
    """A fake ESP32 with the real bridge node talking to it over a pty."""

    def __init__(self, *fake_args):
        self._procs = []
        self._spawn("ros2", "run", "hoverboard_bridge", "fake_esp32", *fake_args)
        time.sleep(STARTUP_S)
        self._spawn("ros2", "run", "hoverboard_bridge", "hoverboard_bridge",
                    "--ros-args", "-p", "port:=/tmp/fake_esp32")
        time.sleep(STARTUP_S)

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
            proc.wait(timeout=10)


@pytest.fixture
def harness(ros):
    made = []

    def _make(*fake_args):
        h = Harness(*fake_args)
        made.append(h)
        return h

    yield _make
    for h in made:
        h.close()


@pytest.fixture(autouse=True)
def bump_file():
    """Path used to toggle the simulated bumper, cleaned either side.

    A fixture rather than an importable constant: test/ is not a package, so a
    relative import from conftest does not work, and duplicating the path in
    each test would be a second source of truth for it.
    """
    if os.path.exists(BUMP_FILE):
        os.remove(BUMP_FILE)
    yield BUMP_FILE
    if os.path.exists(BUMP_FILE):
        os.remove(BUMP_FILE)
