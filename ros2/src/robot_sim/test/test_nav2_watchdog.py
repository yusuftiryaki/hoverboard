"""nav2_watchdog end to end: a Nav2 that never activates takes the bringup down.

The real hang (a dropped change_state response, nav2_watchdog.py) cannot be
produced on demand. What can: a Nav2 that cannot activate because nothing
publishes odom -> base_link, with a watchdog allowance shorter than that wait.
From the watchdog's side the two look identical — a lifecycle manager that
does not report active — and what is under test is everything after that: the
exit code, nav2.launch.py's handler, the launch-wide Shutdown, and that no
Nav2 process is left behind for systemd's restart to collide with.

The healthy case is every other Nav2 test: each stack runs this watchdog
(120 s default) and none of them is shut down.

Spawns processes, ~30 s. SKIPs without ROS.
"""

import os
import signal
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from conftest import NAV2_PATTERNS, pids_matching, stray_stack_pids   # noqa: E402

STUCK_EXIT = 3


def test_the_watchdog_alone_gives_up_on_a_manager_that_never_answers(ros):
    started = time.monotonic()
    result = subprocess.run(
        ["ros2", "run", "hoverboard_bridge", "nav2_watchdog", "--ros-args",
         "-p", "timeout_s:=3.0", "-p", "period_s:=0.5",
         "-r", "__node:=nav2_watchdog_alone",
         "-p", "manager:=no_such_lifecycle_manager"],
        capture_output=True, text=True, timeout=60)
    elapsed = time.monotonic() - started
    assert result.returncode == STUCK_EXIT, (result.returncode, result.stderr[-2000:])
    assert "never became active" in result.stderr and "did not answer" in result.stderr
    assert elapsed < 30.0


def test_a_nav2_that_never_activates_shuts_the_whole_launch_down(ros):
    # A process that has just exited can still be on its way out (the
    # previous test's watchdog, for one): wait, then insist.
    deadline = time.monotonic() + 15.0
    while (stray_stack_pids() or pids_matching(*NAV2_PATTERNS)) and time.monotonic() < deadline:
        time.sleep(0.25)
    assert not stray_stack_pids() and not pids_matching(*NAV2_PATTERNS), \
        "Nav2 or stack processes from an earlier test are still running"
    proc = subprocess.Popen(
        ["ros2", "launch", "robot_bringup", "nav2.launch.py", "watchdog_timeout:=4.0"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    try:
        output, _ = proc.communicate(timeout=90)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        output, _ = proc.communicate()
        pytest.fail("nav2.launch.py was still running 90 s after the watchdog's "
                    "4 s allowance — the exit handler did not shut it down:\n"
                    + output[-3000:])
    assert "shutting the bringup down" in output, output[-3000:]
    # ⚠️ `ros2 launch` reports success after a Shutdown event — the reason
    # robot.service must use Restart=always. Pinned so a change shows up here.
    assert proc.returncode == 0, proc.returncode
    deadline = time.monotonic() + 20.0
    while pids_matching(*NAV2_PATTERNS) and time.monotonic() < deadline:
        time.sleep(0.25)
    assert not pids_matching(*NAV2_PATTERNS), \
        "the launch ended but Nav2 processes survived it — a restart would race them"
