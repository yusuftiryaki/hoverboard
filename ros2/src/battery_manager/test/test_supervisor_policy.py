"""SP5's decisions, without ROS. Spec test list: thresholds, every-state
CRITICAL, charging recognised, voltage fallback when the INA228 is gone."""

import math

import pytest

from battery_manager.supervisor_policy import (STATUS_CHARGING, STATUS_DISCHARGING,
                                               STATUS_FULL, STATUS_UNKNOWN, BatterySample,
                                               Level, LevelMonitor, Phase, Step,
                                               Supervisor, Thresholds)

NAN = float("nan")
T = Thresholds()


def soc(p, current=-1.0, status=STATUS_DISCHARGING, voltage=38.0):
    return BatterySample(percentage=p, voltage=voltage, current=current, status=status)


def volts(v):
    """Voltage-only mode: what battery_monitor publishes without an INA228."""
    return BatterySample(percentage=NAN, voltage=v, current=NAN, status=STATUS_UNKNOWN)


def feed(sup, sample, start, seconds, rate=10.0):
    steps = []
    n = int(seconds * rate)
    for i in range(n + 1):
        steps += sup.on_battery(sample, start + i / rate)
    return steps, start + n / rate


def low_battery_until_docking():
    sup = Supervisor(T)
    steps, _ = feed(sup, soc(0.20), 0.0, T.debounce_s + 0.5)
    assert steps == [Step.CANCEL_MISSIONS]
    assert sup.on_result(Step.CANCEL_MISSIONS, True) == [Step.GO_TO_STAGING]
    assert sup.on_result(Step.GO_TO_STAGING, True) == [Step.FACE_DOCK]
    assert sup.on_result(Step.FACE_DOCK, True) == [Step.DOCK]
    assert sup.phase == Phase.DOCKING
    return sup


# ---- thresholds ---------------------------------------------------------------
def test_a_healthy_battery_does_nothing():
    sup = Supervisor(T)
    steps, _ = feed(sup, soc(0.80), 0.0, 10.0)
    assert steps == [] and sup.phase == Phase.NORMAL and not sup.lock


def test_low_must_hold_for_the_debounce_before_anything_happens():
    sup = Supervisor(T)
    steps, _ = feed(sup, soc(0.20), 0.0, T.debounce_s - 0.5)
    assert steps == [] and sup.phase == Phase.NORMAL
    steps, _ = feed(sup, soc(0.20), T.debounce_s - 0.4, 1.0)
    assert steps == [Step.CANCEL_MISSIONS] and sup.phase == Phase.RETURNING


def test_a_single_dip_does_not_send_the_robot_home():
    sup = Supervisor(T)
    for i in range(100):
        p = 0.20 if i % 20 == 0 else 0.30
        assert sup.on_battery(soc(p), i * 0.1) == []
    assert sup.phase == Phase.NORMAL


def test_the_full_return_sequence_ends_charging_and_locked():
    sup = low_battery_until_docking()
    assert not sup.lock                     # docking drives through the Nav2 input
    assert sup.on_result(Step.DOCK, True) == []
    assert sup.phase == Phase.CHARGING and sup.lock


def test_a_late_result_for_a_superseded_step_is_ignored():
    sup = low_battery_until_docking()
    assert sup.on_result(Step.GO_TO_STAGING, False, "late") == []
    assert sup.phase == Phase.DOCKING


@pytest.mark.parametrize("failing", [Step.GO_TO_STAGING, Step.FACE_DOCK])
def test_failing_to_get_home_is_a_safe_stop(failing):
    sup = Supervisor(T)
    feed(sup, soc(0.20), 0.0, T.debounce_s + 0.5)
    sup.on_result(Step.CANCEL_MISSIONS, True)
    if failing == Step.FACE_DOCK:
        sup.on_result(Step.GO_TO_STAGING, True)
    assert sup.on_result(failing, False, "boom") == [Step.CANCEL_ALL]
    assert sup.phase == Phase.SAFE_STOP and sup.lock and "boom" in sup.reason


def test_an_unanswered_cancel_still_goes_home():
    sup = Supervisor(T)
    feed(sup, soc(0.20), 0.0, T.debounce_s + 0.5)
    assert sup.on_result(Step.CANCEL_MISSIONS, False) == [Step.GO_TO_STAGING]


def test_docking_failure_is_a_safe_stop_naming_the_error():
    sup = low_battery_until_docking()
    assert sup.on_result(Step.DOCK, False, "FAILED_TO_DETECT_DOCK") == [Step.CANCEL_ALL]
    assert sup.phase == Phase.SAFE_STOP and "FAILED_TO_DETECT_DOCK" in sup.reason


# ---- CRITICAL in every state ----------------------------------------------------
def _in_phase(phase):
    if phase == Phase.NORMAL:
        return Supervisor(T), 0.0
    if phase == Phase.RETURNING:
        sup = Supervisor(T)
        _, t = feed(sup, soc(0.20), 0.0, T.debounce_s + 0.5)
        return sup, t
    if phase == Phase.DOCKING:
        return low_battery_until_docking(), T.debounce_s + 1.0
    sup = low_battery_until_docking()
    sup.on_result(Step.DOCK, True)
    sup.resume()
    assert sup.phase == Phase.UNDOCKING
    return sup, T.debounce_s + 1.0


@pytest.mark.parametrize("phase", [Phase.NORMAL, Phase.RETURNING, Phase.DOCKING,
                                   Phase.UNDOCKING])
def test_critical_stops_a_robot_that_may_be_moving(phase):
    sup, t = _in_phase(phase)
    steps, _ = feed(sup, soc(0.05), t + 0.1, T.debounce_s + 0.5)
    assert Step.CANCEL_ALL in steps
    assert sup.phase == Phase.SAFE_STOP and sup.lock and sup.pending is None


def test_critical_while_charging_does_not_stop_a_robot_that_is_not_moving():
    """Deliberate: a robot charging at 8 % is doing the right thing. SAFE_STOP
    there would only cancel nothing and raise an ERROR about it."""
    sup = low_battery_until_docking()
    sup.on_result(Step.DOCK, True)
    steps, _ = feed(sup, soc(0.05, current=1.8, status=STATUS_CHARGING), 10.0, 10.0)
    assert steps == [] and sup.phase == Phase.CHARGING


# ---- charging -------------------------------------------------------------------
def test_put_on_the_charger_by_hand_is_recognised():
    sup = Supervisor(T)
    steps, _ = feed(sup, soc(0.60, current=1.8, status=STATUS_CHARGING), 0.0, 1.0)
    assert steps == [] and sup.phase == Phase.CHARGING and sup.lock


def test_a_trickle_is_not_charging():
    sup = Supervisor(T)
    feed(sup, soc(0.60, current=0.2, status=STATUS_CHARGING), 0.0, 1.0)
    assert sup.phase == Phase.NORMAL


def test_full_waits_on_the_dock_and_resume_undocks():
    sup = low_battery_until_docking()
    sup.on_result(Step.DOCK, True)
    feed(sup, soc(1.0, current=0.0, status=STATUS_FULL), 10.0, 1.0)
    assert sup.phase == Phase.DOCKED_IDLE and sup.lock
    ok, _, steps = sup.resume()
    # The lock must be off BEFORE undocking: the dock server drives on the
    # Nav2 input it would otherwise block.
    assert ok and steps == [Step.UNDOCK] and not sup.lock
    assert sup.on_result(Step.UNDOCK, True) == [] and sup.phase == Phase.NORMAL


def test_losing_the_charger_on_the_dock_is_reported():
    sup = low_battery_until_docking()
    sup.on_result(Step.DOCK, True)
    lost = soc(0.50, current=-0.2, status=STATUS_DISCHARGING)
    steps, t = feed(sup, lost, 10.0, T.charge_lost_s - 1.0)
    assert steps == [] and sup.phase == Phase.CHARGING
    steps, _ = feed(sup, lost, t + 0.1, 2.0)
    assert steps == [Step.CANCEL_ALL] and sup.phase == Phase.SAFE_STOP
    assert "discharging" in sup.reason


def test_safe_stop_is_latched_until_resumed():
    sup, t = _in_phase(Phase.NORMAL)
    feed(sup, soc(0.05), t, T.debounce_s + 0.5)
    feed(sup, soc(0.90), 100.0, 10.0)           # someone swapped the pack
    assert sup.phase == Phase.SAFE_STOP
    ok, _, steps = sup.resume()
    assert ok and steps == [] and sup.phase == Phase.NORMAL and not sup.lock
    assert sup.resume()[0] is False


# ---- voltage fallback --------------------------------------------------------------
def test_without_soc_the_filtered_cell_voltage_decides():
    sup = Supervisor(T)
    steps, t = feed(sup, volts(38.0), 0.0, 5.0)
    assert steps == [] and sup.monitor.source == "voltage"
    # 3.45 V/cell is under 3.50: but the filter takes its time.
    steps, t = feed(sup, volts(34.5), t + 0.1, 5.0)
    assert steps == [], "a 5 s sag sent the robot home"
    steps, _ = feed(sup, volts(34.5), t + 0.1, 120.0)
    assert Step.CANCEL_MISSIONS in steps and sup.phase == Phase.RETURNING


def test_a_sag_under_load_is_not_an_empty_battery():
    """5 s climbing a hill at 33 V (3.3 V/cell, CRITICAL if taken at face
    value) then 10 s at 37.5 V, for ten minutes. Longer than the debounce, so
    only the voltage filter keeps the robot working — checked: with a 0.5 s
    filter this test fails."""
    mon = LevelMonitor(T)
    t = 0.0
    for cycle in range(40):
        for i in range(150):
            v = 33.0 if i >= 100 else 37.5
            assert mon.update(volts(v), t) == Level.OK, (cycle, i)
            t += 0.1


def test_stale_battery_is_unknown_and_harmless():
    sup = Supervisor(T)
    feed(sup, soc(0.20), 0.0, T.debounce_s - 1.0)
    assert sup.on_stale(10.0) == []
    assert sup.monitor.level == Level.UNKNOWN and sup.phase == Phase.NORMAL


@pytest.mark.parametrize("kwargs", [dict(return_soc=0.1, critical_soc=0.2),
                                    dict(critical_cell_v=3.6),
                                    dict(debounce_s=math.nan), dict(cells=0)])
def test_nonsense_thresholds_are_refused(kwargs):
    with pytest.raises(ValueError):
        Thresholds(**kwargs)


# ---- on request ---------------------------------------------------------------------
def test_return_home_on_request_runs_the_low_battery_sequence():
    sup = Supervisor(T)
    feed(sup, soc(0.80), 0.0, 1.0)
    ok, _, steps = sup.return_home()
    assert ok and steps == [Step.CANCEL_MISSIONS] and sup.phase == Phase.RETURNING
    assert sup.on_result(Step.CANCEL_MISSIONS, True) == [Step.GO_TO_STAGING]
    # A healthy battery does not call it off half way.
    feed(sup, soc(0.80), 2.0, 5.0)
    assert sup.phase == Phase.RETURNING and sup.pending == Step.GO_TO_STAGING


def test_return_home_is_refused_unless_normal():
    sup = low_battery_until_docking()
    assert sup.return_home()[0] is False and sup.phase == Phase.DOCKING
