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