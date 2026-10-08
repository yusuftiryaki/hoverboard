"""battery_monitor integration tests against a bridge stand-in."""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BridgeStandIn(Node):
    def __init__(self, voltage=38.0):
        super().__init__("bridge_stand_in")
        self.voltage = voltage
        self.received = []
        self._pub = self.create_publisher(BatteryState, "battery_raw", 10)
        self.create_subscription(BatteryState, "battery", self.received.append, 10)
        self.create_timer(0.1, self._tick)

    def _tick(self):
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.voltage = self.voltage
        self._pub.publish(message)


def spin_for(ros, node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ros.spin_once(node, timeout_sec=0.05)


def test_fake_ina_yields_soc_and_signed_current(ros, monitor_harness):
    stand_in = BridgeStandIn(voltage=38.0)
    try:
        monitor_harness("use_fake_bus:=true", "fake_current_a:=-2.0", "init_window_s:=1.0")
        spin_for(ros, stand_in, 6.0)
        assert stand_in.received, "no /battery published"
        last = stand_in.received[-1]
        assert last.voltage == pytest.approx(38.0, abs=0.1)
        assert last.current == pytest.approx(-2.0, abs=0.1)
        assert last.percentage == pytest.approx(0.45, abs=0.02)
        assert last.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    finally:
        stand_in.destroy_node()


def test_the_simulated_sensor_is_built_with_the_shunt_the_driver_believes_in(
        ros, monitor_harness):
    """One shunt value, not two.

    The fake INA228 turns a true current into a shunt voltage, and the driver
    turns that voltage back into a current. Each needs the shunt's resistance,
    and they used to get it from different places: the driver from the
    `shunt_ohms` parameter, the fake from its own default of 1.5 milliohm. The
    test above passed only because the parameter's default was ALSO 1.5 — the
    two agreed by coincidence. The day the real board turned out to carry a
    2 milliohm shunt (R002) and the config said so, every simulated current
    would have read 0.75x the truth, with nothing failing anywhere.

    Deliberately not 1.5 milliohm, so a coincidence cannot pass it.
    """
    stand_in = BridgeStandIn(voltage=38.0)
    try:
        monitor_harness("use_fake_bus:=true", "fake_current_a:=-2.0",
                        "shunt_ohms:=0.002", "init_window_s:=1.0")
        spin_for(ros, stand_in, 6.0)
        assert stand_in.received, "no /battery published"
        assert stand_in.received[-1].current == pytest.approx(-2.0, abs=0.1), (
            "a 2.0 A simulated discharge did not read back as 2.0 A — the fake "
            "bus and the driver disagree about the shunt resistance")
    finally:
        stand_in.destroy_node()


def test_no_sensor_falls_back_to_voltage_only(ros, monitor_harness):
    stand_in = BridgeStandIn(voltage=38.0)
    try:
        monitor_harness("use_fake_bus:=false", "init_window_s:=1.0")
        spin_for(ros, stand_in, 6.0)
        assert stand_in.received, "no /battery published in fallback mode"
        last = stand_in.received[-1]
        assert last.voltage == pytest.approx(38.0, abs=0.1)
        assert math.isnan(last.current)
        assert math.isnan(last.percentage)
        assert last.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    finally:
        stand_in.destroy_node()