"""teleop.TeleopGate: the deadman, the lease and the speed limits, on a stepped
clock. ROS-free. The HTTP side is in test_web_api.py, the real stack (mux,
bridge, wheels) in robot_sim/test/test_web_stack.py."""

import pytest

from robot_missions.teleop import TeleopConflict, TeleopError, TeleopGate


class Clock:
    def __init__(self):
        self.now = 50.0

    def __call__(self):
        return self.now


@pytest.fixture
def gate():
    clock = Clock()
    g = TeleopGate(max_linear=0.6, max_angular=1.0, deadman_s=0.3, lease_s=2.0, clock=clock)
    g.clock = clock
    return g


def test_nothing_is_published_until_someone_drives(gate):
    assert gate.output() is None
    assert gate.state()["held"] is False


def test_a_fresh_lease_holds_the_robot_still(gate):
    gate.acquire()
    assert gate.output() == (0.0, 0.0)
    state = gate.state()
    assert state["held"] and not state["active"] and state["linear"] == 0.0


@pytest.mark.parametrize("ratio, expected", [(1.0, 0.6), (-1.0, -0.6), (0.5, 0.3),
                                             (7.0, 0.6), (-1e9, -0.6)])
def test_the_page_cannot_exceed_the_limits(gate, ratio, expected):
    token = gate.acquire()
    gate.command(token, ratio, ratio)
    linear, angular = gate.output()
    assert linear == pytest.approx(expected)
    assert angular == pytest.approx(expected / 0.6 * 1.0)


def test_a_steady_stream_keeps_driving_and_keeps_the_lease(gate):
    token = gate.acquire()
    for _ in range(100):                     # 10 s at 10 Hz, far past lease_s
        gate.command(token, 0.5, 0.0)
        gate.clock.now += 0.1
        assert gate.output() == pytest.approx((0.3, 0.0))


def test_silence_stops_at_deadman_then_releases_at_lease(gate):
    token = gate.acquire()
    gate.command(token, 1.0, 1.0)
    gate.clock.now += 0.299
    assert gate.output() == (0.6, 1.0)
    gate.clock.now += 0.002
    assert gate.output() == (0.0, 0.0)
    gate.clock.now += 1.69                   # 1.991 s since the last command
    assert gate.output() == (0.0, 0.0)
    gate.clock.now += 0.02
    assert gate.output() is None
    with pytest.raises(TeleopConflict):
        gate.command(token, 1.0, 0.0)        # a late packet does not revive it


def test_an_abandoned_lease_frees_the_wheels_for_the_next_tab(gate):
    gate.acquire()
    with pytest.raises(TeleopConflict):
        gate.acquire()
    gate.clock.now += 2.01
    second = gate.acquire()
    gate.command(second, 0.2, 0.0)
    assert gate.output() == pytest.approx((0.12, 0.0))


def test_releasing_needs_the_token_and_stops_publishing(gate):
    token = gate.acquire()
    gate.command(token, 1.0, 0.0)
    assert gate.release("someone-else") is False
    assert gate.output() == (0.6, 0.0)
    assert gate.release(token) is True
    assert gate.output() is None
    with pytest.raises(TeleopConflict):
        gate.command(token, 1.0, 0.0)


@pytest.mark.parametrize("value", [None, "0.5", True, float("nan"), float("inf"), [0.5]])
def test_a_malformed_command_keeps_the_last_good_one(gate, value):
    token = gate.acquire()
    gate.command(token, 0.5, 0.0)
    with pytest.raises(TeleopError):
        gate.command(token, value, 0.0)
    with pytest.raises(TeleopError):
        gate.command(token, 0.0, value)
    assert gate.output() == pytest.approx((0.3, 0.0))


@pytest.mark.parametrize("kwargs", [{"max_linear": 0}, {"max_angular": -1},
                                    {"deadman_s": 0}, {"deadman_s": 2.0, "lease_s": 2.0}])
def test_nonsense_limits_are_refused(kwargs):
    with pytest.raises(ValueError):
        TeleopGate(**kwargs)
