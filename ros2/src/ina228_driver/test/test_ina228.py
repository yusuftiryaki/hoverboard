"""INA228 register and measurement tests; no I2C or ROS required."""

import pytest

from ina228_driver.fake_bus import FakeINA228Bus
from ina228_driver.ina228 import (
    ADC_CONFIG_VALUE,
    DEVICE_ID_VALUE,
    REG_ADC_CONFIG,
    INA228,
    _to_signed20,
)


def make(shunt_ohms=0.0015, **kwargs):
    bus = FakeINA228Bus(rshunt_ohms=0.0015, noise_current_a=0.0, **kwargs)
    ina = INA228(bus, shunt_ohms=shunt_ohms)
    ina.reset()
    ina.configure()
    return bus, ina


def test_signed20_decoding():
    assert _to_signed20(0x000010) == 1
    assert _to_signed20(0x000000) == 0
    assert _to_signed20(0xFFFFF0) == -1
    assert _to_signed20(0x800000) == -524288
    assert _to_signed20(0x7FFFF0) == 524287


def test_probe_returns_device_id():
    _, ina = make()
    assert ina.probe() == DEVICE_ID_VALUE


def test_wrong_address_raises_like_a_real_bus():
    ina = INA228(FakeINA228Bus(), address=0x44)
    with pytest.raises(OSError):
        ina.probe()


def test_configure_writes_continuous_mode_with_averaging():
    bus, _ = make()
    assert bus.reg16(REG_ADC_CONFIG) == ADC_CONFIG_VALUE


def test_current_scale_round_trip():
    bus, ina = make()
    bus.true_current_a = -12.5
    assert ina.read().current == pytest.approx(-12.5, rel=1e-3)
    bus.true_current_a = 1.8
    assert ina.read().current == pytest.approx(1.8, rel=1e-3)


def test_mismatched_shunt_value_reads_wrong_current():
    bus, ina = make(shunt_ohms=0.003)
    bus.true_current_a = -10.0
    assert ina.read().current == pytest.approx(-5.0, rel=1e-3)


def test_bus_voltage_and_temperature_scale():
    bus, ina = make()
    bus.true_bus_v = 37.2
    bus.true_temp_c = 41.5
    sample = ina.read()
    assert sample.bus_voltage == pytest.approx(37.2, abs=0.001)
    assert sample.temperature == pytest.approx(41.5, abs=0.01)


def test_rejects_nonpositive_shunt():
    with pytest.raises(ValueError):
        INA228(FakeINA228Bus(), shunt_ohms=0.0)