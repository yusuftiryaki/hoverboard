"""SoC estimator tests; pure Python and deterministic."""

import pytest

from battery_manager.soc_estimator import SocEstimator


def make(capacity_ah=4.4, **kwargs):
    return SocEstimator(capacity_ah=capacity_ah, **kwargs)


def test_starts_uninitialized_and_update_refuses():
    estimator = make()
    assert not estimator.initialized
    assert estimator.soc is None
    with pytest.raises(RuntimeError):
        estimator.update(-1.0, 38.0, 0.1)


def test_rest_voltage_initialization_interpolates_the_ocv_table():
    assert make().initialize_from_rest_voltage(38.0) == pytest.approx(0.45, abs=0.01)
    assert make().initialize_from_rest_voltage(41.0) == pytest.approx(0.90, abs=0.01)


def test_rest_voltage_clamps_outside_the_table():
    assert make().initialize_from_rest_voltage(45.0) == pytest.approx(1.0)
    assert make().initialize_from_rest_voltage(25.0) == pytest.approx(0.0)


def test_discharge_integrates_down():
    estimator = make()
    estimator.initialize_from_rest_voltage(41.0)
    for _ in range(3600):
        estimator.update(-2.2, 37.0, 1.0)
    assert estimator.soc == pytest.approx(0.90 - 0.50, abs=1e-6)


def test_charge_integrates_up_and_clamps_at_one():
    estimator = make()
    estimator.initialize_from_rest_voltage(41.0)
    for _ in range(3600):
        estimator.update(2.2, 41.0, 1.0)
    assert estimator.soc == pytest.approx(1.0)


def test_discharge_clamps_at_zero():
    estimator = make()
    estimator.initialize_from_rest_voltage(30.0)
    estimator.update(-10.0, 30.0, 60.0)
    assert estimator.soc == pytest.approx(0.0)


def test_charge_complete_resets_to_full():
    estimator = make(full_voltage_per_cell=4.15, taper_current_a=0.3, full_hold_s=30.0)
    estimator.initialize_from_rest_voltage(38.0)
    for _ in range(30):
        estimator.update(0.2, 41.6, 1.0)
    assert estimator.soc == pytest.approx(1.0)
    assert estimator.just_reached_full


def test_brief_taper_does_not_reset():
    estimator = make(full_hold_s=30.0)
    estimator.initialize_from_rest_voltage(38.0)
    for _ in range(10):
        estimator.update(0.2, 41.6, 1.0)
    estimator.update(-3.0, 37.0, 1.0)
    for _ in range(20):
        estimator.update(0.2, 41.6, 1.0)
    assert estimator.soc < 0.6
    assert not estimator.just_reached_full