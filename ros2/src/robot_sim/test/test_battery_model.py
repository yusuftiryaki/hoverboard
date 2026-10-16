"""PackModel against numbers worked out by hand (handoff decision 10).

ROS-free; runs in the `unit` stage.
"""

import pytest

from robot_sim.battery_model import PackModel, cell_ocv


def test_rest_voltage_is_the_table_times_the_cells():
    # 0.5 -> 3.85 V/cell straight off the table; 0.25 halfway 3.65..3.72.
    assert cell_ocv(0.5) == pytest.approx(3.85)
    assert cell_ocv(0.25) == pytest.approx(3.685)
    assert PackModel(soc=0.5).open_circuit_voltage == pytest.approx(38.5)


def test_standing_still_draws_the_idle_current_and_sags_by_ir():
    pack = PackModel(soc=0.5)
    voltage, current = pack.step(1.0, 0.0, 0.0, on_charger=False)
    assert current == pytest.approx(-0.2)
    # 38.5 V - 0.2 A * 0.15 Ohm
    assert voltage == pytest.approx(38.47)


def test_driving_drains_by_coulomb_count():
    pack = PackModel(soc=0.5)
    # 1 m/s: 0.2 + 1.5 = 1.7 A out, for one hour, of 4.4 Ah.
    voltage, current = pack.step(3600.0, 1.0, 0.0, on_charger=False)
    assert current == pytest.approx(-1.7)
    assert voltage == pytest.approx(38.5 - 1.7 * 0.15)
    assert pack.soc == pytest.approx(0.5 - 1.7 / 4.4)


def test_turning_costs_current_in_either_direction():
    _, left = PackModel(soc=0.5).step(1.0, 0.0, 1.0, on_charger=False)
    _, right = PackModel(soc=0.5).step(1.0, 0.0, -1.0, on_charger=False)
    assert left == right == pytest.approx(-0.7)


def test_on_the_charger_the_brick_runs_constant_current_and_the_pack_reads_high():
    pack = PackModel(soc=0.5)
    voltage, current = pack.step(1.0, 0.0, 0.0, on_charger=True)
    # 2 A in, 0.2 A of it straight back out to the idle board.
    assert current == pytest.approx(1.8)
    assert voltage == pytest.approx(38.5 + 1.8 * 0.15)


def test_near_full_the_charger_tapers_at_its_cv_setpoint():
    pack = PackModel(soc=0.995)
    ocv = pack.open_circuit_voltage            # 41.93 V
    _, current = pack.step(0.0, 0.0, 0.0, on_charger=True)
    # (42.0 - 41.93) / 0.15 = 0.467 A from the charger, minus the 0.2 A idle.
    assert current == pytest.approx((42.0 - ocv) / 0.15 - 0.2)
    assert current < 0.5


def test_soc_is_clamped():
    pack = PackModel(soc=0.001)
    pack.step(3600.0, 1.0, 0.0, on_charger=False)
    assert pack.soc == 0.0
    with pytest.raises(ValueError):
        PackModel(capacity_ah=0.0)
