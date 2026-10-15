"""nav_cmd_gate: Nav2 reaches the wheels only while ekf_local is sane.

The first half is the decision, ROS-free and pinned to the numbers CI actually
produced (robot_localization 3.8.3, ekf.yaml). The second half runs the real
node and checks it forwards, blocks, and notices a filter that went quiet.
"""

import math
import os
import signal
import subprocess
import time

import pytest

from hoverboard_bridge.nav_cmd_gate import HealthLimits, estimate_problem

LIMITS = HealthLimits()
#               x     y    qz   qw   vx    wz    var_x var_y
HEALTHY = dict(x=3.2, y=-1.4, yaw_z=0.38, yaw_w=0.92, vx=0.5, wz=0.3,
               var_x=0.4, var_y=0.4)


def problem(**changes):
    return estimate_problem(**{**HEALTHY, **changes}, limits=LIMITS)


def test_a_healthy_estimate_passes():
    assert problem() is None


def test_what_ci_actually_saw_is_stopped():
    # Second EKF output after startup, PR #2's nav2 bag: x 6.4e146, vx 4.1e148,
    # var_x 8.5e281. Any one of them alone must be enough.
    assert "position" in problem(x=6.4e146)
    assert "variance" in problem(var_x=8.5e281)
    assert "speed" in problem(vx=4.1e148)


@pytest.mark.parametrize("field", ["x", "y", "yaw_z", "yaw_w", "vx", "wz",
                                   "var_x", "var_y"])
def test_non_finite_anywhere_is_stopped(field):
    assert problem(**{field: math.nan}) == "EKF output is not finite"
    assert problem(**{field: math.inf}) == "EKF output is not finite"


def test_the_limits_are_far_from_anything_a_healthy_robot_does():
    # A day of driving: kilometres from the odom origin, wheel-only covariance
    # grown to hundreds of m^2, top speed of the board, a fast spin.
    assert problem(x=2500.0, y=-900.0, var_x=900.0, var_y=900.0,
                   vx=2.6, wz=3.0) is None


# ---- the real node --------------------------------------------------------

@pytest.fixture
def gate(ros):
    proc = subprocess.Popen(["ros2", "run", "hoverboard_bridge", "nav_cmd_gate"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    time.sleep(2.5)
    yield proc
    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    proc.wait(timeout=10)


@pytest.fixture
def rig(ros, gate):
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    node = ros.create_node("gate_rig")
    got = []
    node.create_subscription(Twist, "/cmd_vel", got.append, 10)
    nav = node.create_publisher(Twist, "/cmd_vel_nav", 10)
    odom = node.create_publisher(Odometry, "/odometry/filtered/local", 10)

    def run(seconds, odom_fields=None, v=0.4):
        """Publish nav commands (and odometry, unless None) for a while;
        return the /cmd_vel speeds that came out."""
        got.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if odom_fields is not None:
                m = Odometry()
                f = {**HEALTHY, **odom_fields}
                m.pose.pose.position.x, m.pose.pose.position.y = f["x"], f["y"]
                m.pose.pose.orientation.z = f["yaw_z"]
                m.pose.pose.orientation.w = f["yaw_w"]
                m.twist.twist.linear.x, m.twist.twist.angular.z = f["vx"], f["wz"]
                m.pose.covariance[0], m.pose.covariance[7] = f["var_x"], f["var_y"]
                odom.publish(m)
            cmd = Twist()
            cmd.linear.x = v
            nav.publish(cmd)
            ros.spin_once(node, timeout_sec=0.05)
        return [m.linear.x for m in got]

    yield run
    node.destroy_node()


def test_the_real_gate_forwards_blocks_and_notices_silence(rig):
    out = rig(2.0, odom_fields={})
    assert out and out[-1] == pytest.approx(0.4), "healthy EKF: Nav2 must pass"

    out = rig(1.0, odom_fields={"vx": 4.1e148})
    assert out and out[-1] == 0.0, "diverged EKF: Nav2 must be replaced by zero"

    out = rig(1.0, odom_fields={})
    assert out and out[-1] == pytest.approx(0.4), "recovered EKF: forward again"

    out = rig(1.5, odom_fields=None)       # ekf_local stops publishing
    assert out and out[-1] == 0.0, "stale EKF (>0.5 s): Nav2 must be stopped"
