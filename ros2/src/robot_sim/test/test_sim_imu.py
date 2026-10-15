"""The simulated gyro cannot report what a real MPU6050 could not.

gz-sim 8.11.0 could publish ~628 rad/s for one ground-truth sample (upstream
#3834), and sim_node passed it straight into the gyro; ekf_local, trusting the
gyro ~500x over the wheels for yaw rate, keeps whatever heading step that
integrates to. A stand-in spike reproduced the "unexplained" A3e signature
(up to 5.6 m / 161 deg, wheels untouched) — the strongest candidate for those
runs. The real chip saturates at 32767 counts (4.37 rad/s at the robot's 250
dps range), so the fake one must too. In-process; skips without ROS.
"""

import math

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from robot_sim.sim_node import SimNode           # noqa: E402

FULL_SCALE_250_DPS = 4.36559   # rad/s: 32767 / 131 dps, by hand


@pytest.fixture
def sim(tmp_path):
    own = not rclpy.ok()
    if own:
        rclpy.init()
    node = SimNode(parameter_overrides=[
        rclpy.parameter.Parameter("link", value=str(tmp_path / "fake_esp32"))])
    published = []
    node._imu_pub.publish = published.append       # capture, no graph needed
    yield node, published
    node.destroy_node()
    if own:
        rclpy.shutdown()


def gyro_for(node, published, omega):
    node._world.omega = omega
    node._publish_imu()
    return published[-1].angular_velocity.z


@pytest.mark.parametrize("omega", [314.16, -314.16, 1e9, -6.0])
def test_a_rate_beyond_the_chip_saturates_at_its_full_scale(sim, omega):
    node, published = sim
    assert gyro_for(node, published, omega) == pytest.approx(
        math.copysign(FULL_SCALE_250_DPS, omega), abs=1e-4)


def test_a_normal_turn_is_untouched(sim):
    # 0.5 rad/s, the square's turn rate: only the 0.1 dps bias and the noise.
    node, published = sim
    assert gyro_for(node, published, 0.5) == pytest.approx(0.5, abs=0.01)
