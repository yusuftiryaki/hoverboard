"""INA228 current plus bridge voltage to the authoritative /battery topic."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from battery_manager.soc_estimator import SocEstimator
from ina228_driver.ina228 import DEVICE_ID_VALUE, INA228

IDLE_CURRENT_A = 0.05
MAX_CONSECUTIVE_ERRORS = 20


class BatteryMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_monitor")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("shunt_ohms", 0.0015)
        self.declare_parameter("invert_current", False)
        self.declare_parameter("capacity_ah", 4.4)
        self.declare_parameter("cells", 10)
        self.declare_parameter("full_voltage_per_cell", 4.15)
        self.declare_parameter("taper_current_a", 0.3)
        self.declare_parameter("full_hold_s", 30.0)
        self.declare_parameter("init_window_s", 5.0)
        self.declare_parameter("use_fake_bus", False)
        self.declare_parameter("fake_current_a", -2.0)

        get_parameter = self.get_parameter
        self._invert = bool(get_parameter("invert_current").value)
        self._init_window_s = float(get_parameter("init_window_s").value)
        self._capacity_ah = float(get_parameter("capacity_ah").value)
        self._estimator = SocEstimator(
            capacity_ah=self._capacity_ah,
            cells=int(get_parameter("cells").value),
            full_voltage_per_cell=float(get_parameter("full_voltage_per_cell").value),
            taper_current_a=float(get_parameter("taper_current_a").value),
            full_hold_s=float(get_parameter("full_hold_s").value),
        )

        self._ina: Optional[INA228] = None
        self._bus = None
        self._setup_sensor()

        self._raw_voltage = float("nan")
        self._raw_stamp = None
        self._init_samples: list[float] = []
        self._init_started = None
        self._current_a = float("nan")
        self._read_errors = 0
        self._consecutive_errors = 0
        self._is_full = False

        self.create_subscription(BatteryState, "battery_raw", self._on_raw, 10)
        self._battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._dt = 1.0 / float(get_parameter("rate_hz").value)
        self.create_timer(self._dt, self._tick)
        self.create_timer(1.0, self._diag_tick)

    def _setup_sensor(self) -> None:
        get_parameter = self.get_parameter
        address = int(get_parameter("address").value)
        shunt_ohms = float(get_parameter("shunt_ohms").value)
        try:
            if bool(get_parameter("use_fake_bus").value):
                from ina228_driver.fake_bus import FakeINA228Bus

                self.get_logger().warn("SİMÜLE INA228 kullanılıyor")
                # The SAME shunt the driver is about to divide by. The fake
                # used to keep its own 1.5 mOhm default, which agreed with the
                # parameter's default by coincidence and with nothing else.
                self._bus = FakeINA228Bus(address=address, rshunt_ohms=shunt_ohms)
                self._bus.true_current_a = float(get_parameter("fake_current_a").value)
            else:
                import smbus2

                self._bus = smbus2.SMBus(int(get_parameter("i2c_bus").value))
            ina = INA228(
                self._bus,
                address=address,
                shunt_ohms=shunt_ohms,
            )
            device_id = ina.probe()
            if device_id != DEVICE_ID_VALUE:
                raise OSError(
                    f"0x{address:02x} device id 0x{device_id:03x} "
                    f"!= 0x{DEVICE_ID_VALUE:03x}"
                )
            ina.reset()
            ina.configure()
            self._ina = ina
            self.get_logger().info("INA228 hazır - SoC aktif")
        except (ImportError, OSError, ValueError) as exc:
            self._ina = None
            self.get_logger().warn(f"INA228 yok ({exc}) - voltaj-yalnız mod")

    def _on_raw(self, msg: BatteryState) -> None:
        self._raw_voltage = msg.voltage
        self._raw_stamp = self.get_clock().now()
        if self._estimator.initialized:
            return
        now = self.get_clock().now()
        if self._init_started is None:
            self._init_started = now
        self._init_samples.append(msg.voltage)
        elapsed = (now - self._init_started).nanoseconds * 1e-9
        if elapsed >= self._init_window_s:
            rest_voltage = sum(self._init_samples) / len(self._init_samples)
            soc = self._estimator.initialize_from_rest_voltage(rest_voltage)
            self.get_logger().info(f"SoC başlatıldı: {rest_voltage:.1f} V -> %{soc * 100.0:.0f}")

    def _tick(self) -> None:
        if self._ina is not None:
            try:
                sample = self._ina.read()
                self._current_a = -sample.current if self._invert else sample.current
                self._consecutive_errors = 0
            except OSError as exc:
                self._read_errors += 1
                self._consecutive_errors += 1
                self._current_a = float("nan")
                self.get_logger().warn(f"INA228 okuması başarısız: {exc}")
                if self._consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    self._ina = None
                    self.get_logger().error("INA228 okunamadı - voltaj-yalnız moda geçildi")

        if (self._estimator.initialized and not math.isnan(self._current_a)
                and not math.isnan(self._raw_voltage)):
            self._estimator.update(self._current_a, self._raw_voltage, self._dt)
            if self._estimator.just_reached_full:
                self._is_full = True
            elif self._current_a < -IDLE_CURRENT_A:
                self._is_full = False
        self._publish()

    def _publish(self) -> None:
        soc = self._estimator.soc
        have_soc = soc is not None and not math.isnan(self._current_a)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self._raw_voltage
        msg.current = self._current_a
        msg.percentage = float(soc) if have_soc else float("nan")
        msg.charge = float(soc) * self._capacity_ah if have_soc else float("nan")
        msg.capacity = self._capacity_ah
        msg.design_capacity = self._capacity_ah
        if math.isnan(self._current_a):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif self._is_full:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif self._current_a > IDLE_CURRENT_A:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self._current_a < -IDLE_CURRENT_A:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.location = "hoverboard"
        self._battery_pub.publish(msg)

    def _diag_tick(self) -> None:
        status = DiagnosticStatus(name="battery_monitor: batarya", hardware_id="ina228")
        raw_age = None
        if self._raw_stamp is not None:
            raw_age = (self.get_clock().now() - self._raw_stamp).nanoseconds * 1e-9
        if raw_age is None or raw_age > 2.0:
            status.level = DiagnosticStatus.WARN
            status.message = "battery_raw gelmiyor - köprü çalışıyor mu?"
        elif self._ina is None:
            status.level = DiagnosticStatus.WARN
            status.message = "INA228 yok - voltaj-yalnız mod (SoC yok)"
        elif not self._estimator.initialized:
            status.level = DiagnosticStatus.OK
            status.message = "dinlenim voltajı toplanıyor"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "izliyor"
        soc = self._estimator.soc
        status.values = [
            KeyValue(key="voltage_v", value=f"{self._raw_voltage:.2f}"),
            KeyValue(key="current_a", value=f"{self._current_a:.2f}"),
            KeyValue(key="soc", value=f"{soc:.3f}" if soc is not None else "nan"),
            KeyValue(key="read_errors", value=str(self._read_errors)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> bool:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BatteryMonitorNode] = None
    try:
        node = BatteryMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()