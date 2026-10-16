"""nav2_watchdog's decision, without ROS: when does a Nav2 that is not
active stop being "still starting" and become "stuck, restart it"?"""

import pytest

from hoverboard_bridge.nav2_watchdog import BringupWatch, Verdict

T = 120.0


def run(watch, answers, start=0.0, period=2.0):
    verdict = None
    for i, answer in enumerate(answers):
        verdict = watch.feed(answer, start + i * period)
    return verdict


def test_a_normal_bring_up_is_waited_for():
    watch = BringupWatch(T, 0.0)
    assert run(watch, [None] * 10 + [False] * 5 + [True]) == Verdict.ACTIVE
    assert watch.ever_active and watch.reason == ""


def test_a_manager_that_never_answers_is_the_stuck_signature():
    """The measured hang: is_active is never answered at all."""
    watch = BringupWatch(T, 0.0)
    assert run(watch, [None] * 60) == Verdict.WAITING          # 118 s
    assert watch.feed(None, 120.0) == Verdict.FAILED
    assert "never became active" in watch.reason
    assert "did not answer" in watch.reason


def test_a_server_that_will_not_transition_is_named_differently():
    watch = BringupWatch(T, 0.0)
    run(watch, [None] * 5 + [False] * 60)
    assert watch.feed(False, 200.0) == Verdict.FAILED
    assert "answered 'not active'" in watch.reason


def test_losing_nav2_after_it_was_up_also_restarts_but_only_after_the_allowance():
    watch = BringupWatch(T, 0.0)
    run(watch, [True] * 30)                                   # active until t=58
    assert watch.feed(None, 100.0) == Verdict.WAITING
    assert watch.feed(None, 177.0) == Verdict.WAITING         # 119 s after last active
    assert watch.feed(None, 179.0) == Verdict.FAILED
    assert "stopped being active" in watch.reason


def test_one_answer_resets_the_clock():
    watch = BringupWatch(T, 0.0)
    watch.feed(None, 100.0)
    assert watch.feed(True, 110.0) == Verdict.ACTIVE
    assert watch.feed(None, 200.0) == Verdict.WAITING          # 90 s since 110


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan")])
def test_nonsense_timeouts_are_refused(bad):
    with pytest.raises(ValueError):
        BringupWatch(bad, 0.0)
