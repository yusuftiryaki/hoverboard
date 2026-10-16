"""mission_runner's state machine, event by event. ROS-free."""

import pytest

from robot_missions.runner_policy import (
    Cancel, Persist, Plan, Progress, Result, ResumeSupervisor, RunnerPolicy, Send, State)

MISSION = {"type": "route", "route": "tur"}


def sends(actions):
    return [a for a in actions if isinstance(a, Send)]


def running(total=3, repeat=1, **kwargs):
    """A policy RUNNING waypoint 0 of `total`, supervisor NORMAL."""
    policy = RunnerPolicy(**kwargs)
    policy.supervisor_state("NORMAL", 0.0)
    ok, _, actions = policy.start(MISSION)
    assert ok and Plan(lap=0) in actions
    actions = policy.planned(total, repeat)
    assert policy.state == State.RUNNING
    (send,) = sends(actions)
    assert send.index == 0
    return policy, send.token


def finish(policy, token):
    """Succeed the live goal, return the next Send (or None)."""
    actions = policy.goal_result(token, Result.SUCCEEDED, 0.0)
    nxt = sends(actions)
    return nxt[0] if nxt else None


def test_waypoints_run_in_order_and_the_mission_finishes():
    policy, token = running(total=3)
    seen = []
    send = Send(token, 0)
    while send is not None:
        seen.append(send.index)
        send = finish(policy, send.token)
    assert seen == [0, 1, 2]
    assert policy.state == State.DONE


def test_every_step_is_persisted():
    policy, token = running(total=2)
    assert Persist() in policy.goal_result(token, Result.SUCCEEDED, 0.0)
    assert policy.progress.index == 1


def test_start_is_refused_unless_the_supervisor_says_normal():
    policy = RunnerPolicy()
    ok, why, _ = policy.start(MISSION)
    assert not ok and "not heard from" in why
    policy.supervisor_state("CHARGING", 0.0)
    ok, why, _ = policy.start(MISSION)
    assert not ok and "CHARGING" in why


def test_a_second_start_is_refused_while_one_runs():
    policy, _ = running()
    ok, why, _ = policy.start(MISSION)
    assert not ok and "RUNNING" in why


def test_an_aborted_goal_is_retried_then_skipped():
    policy, token = running(total=3, max_retries=1)
    (retry,) = sends(policy.goal_result(token, Result.ABORTED, 0.0))
    assert retry.index == 0 and retry.token != token
    (nxt,) = sends(policy.goal_result(retry.token, Result.ABORTED, 0.0))
    assert nxt.index == 1
    assert policy.progress.skipped == [0]


# --- the supervisor -----------------------------------------------------------

def test_low_battery_hands_the_robot_over_and_never_resends():
    policy, token = running(total=3)
    assert finish(policy, token).index == 1
    live = policy._in_flight
    # LOW: the supervisor goes RETURNING and cancels every mission goal.
    assert sends(policy.supervisor_state("RETURNING", 1.0)) == []
    assert policy.state == State.WAITING_DOCK
    # ⚠️ The cancel it caused must not be answered with a new goal: that goal
    # would preempt the supervisor's own trip to the charger.
    assert policy.goal_result(live, Result.CANCELED, 1.1) == []
    for phase in ("DOCKING", "CHARGING"):
        assert policy.supervisor_state(phase, 2.0) == []
    # Full: ask it to undock, once.
    assert policy.supervisor_state("DOCKED_IDLE", 3.0) == [ResumeSupervisor()]
    assert policy.supervisor_state("DOCKED_IDLE", 3.5) == []
    assert policy.supervisor_state("UNDOCKING", 4.0) == []
    # Undocked: carry on with the waypoint that was interrupted.
    (again,) = sends(policy.supervisor_state("NORMAL", 5.0))
    assert again.index == 1 and policy.state == State.RUNNING


def test_the_cancel_can_arrive_before_the_state_does():
    # Same story, other order: CANCELED first, RETURNING a moment later.
    policy, token = running(total=3)
    assert policy.goal_result(token, Result.CANCELED, 1.0) == []
    assert policy.state == State.RUNNING            # not yet explained
    policy.supervisor_state("RETURNING", 1.2)
    assert policy.state == State.WAITING_DOCK
    assert policy.tick(10.0) == []                  # the grace timer is gone


def test_a_cancel_nobody_explains_pauses_the_mission():
    policy, token = running(total=3, cancel_grace_s=3.0)
    policy.goal_result(token, Result.CANCELED, 1.0)
    assert policy.tick(3.9) == []
    assert policy.tick(4.1) == [Persist()]
    assert policy.state == State.PAUSED
    (again,) = sends(policy.resume()[2])
    assert again.index == 0


def test_no_goal_is_sent_while_the_supervisor_is_not_normal():
    policy, token = running(total=3)
    policy.supervisor = "RETURNING"     # the state lands just as the goal succeeds
    actions = policy.goal_result(token, Result.SUCCEEDED, 1.0)
    assert sends(actions) == [] and policy.state == State.WAITING_DOCK
    assert policy.progress.index == 1


def test_safe_stop_parks_the_mission_for_a_person():
    policy, _ = running()
    policy.supervisor_state("SAFE_STOP", 1.0)
    assert policy.state == State.PAUSED and "SAFE_STOP" in policy.reason
    # Resuming while the supervisor is still stopped waits rather than drives.
    ok, _, actions = policy.resume()
    assert ok and sends(actions) == [] and policy.state == State.WAITING_DOCK


def test_a_supervisor_that_will_not_undock_pauses():
    policy, token = running()
    policy.supervisor_state("RETURNING", 1.0)
    policy.supervisor_state("DOCKED_IDLE", 2.0)
    assert policy.supervisor_refused("nothing to resume") == [Persist()]
    assert policy.state == State.PAUSED


# --- commands -------------------------------------------------------------------

def test_pause_cancels_our_goal_and_its_answer_is_ignored():
    policy, token = running(total=3)
    ok, _, actions = policy.pause()
    assert ok and Cancel(token) in actions and policy.state == State.PAUSED
    assert policy.goal_result(token, Result.CANCELED, 1.0) == []
    assert policy.tick(100.0) == []
    (again,) = sends(policy.resume()[2])
    assert again.index == 0 and policy.state == State.RUNNING


def test_resume_on_the_dock_asks_the_supervisor_first():
    policy, _ = running()
    policy.pause()
    policy.supervisor_state("DOCKED_IDLE", 1.0)
    ok, _, actions = policy.resume()
    assert ok and actions == [ResumeSupervisor()] and policy.state == State.WAITING_DOCK


def test_abort_clears_everything():
    policy, token = running()
    ok, _, actions = policy.abort()
    assert ok and Cancel(token) in actions
    assert policy.state == State.IDLE and policy.progress.mission is None
    ok, _, _ = policy.start(MISSION)
    assert ok


def test_an_old_goals_answer_changes_nothing():
    policy, token = running(total=3)
    nxt = finish(policy, token)
    assert policy.goal_result(token, Result.ABORTED, 1.0) == []
    assert policy.progress.index == 1 and policy._in_flight == nxt.token


# --- laps, restarts ---------------------------------------------------------------

def test_each_lap_is_replanned_and_repeat_zero_never_ends():
    policy, token = running(total=2, repeat=0)
    nxt = finish(policy, token)
    actions = policy.goal_result(nxt.token, Result.SUCCEEDED, 0.0)
    assert Plan(lap=1) in actions and policy.state == State.PLANNING
    (first,) = sends(policy.planned(2, 0))
    assert first.index == 0 and policy.progress.lap == 1


def test_repeat_two_ends_after_two_laps():
    policy, token = running(total=1, repeat=2)
    policy.goal_result(token, Result.SUCCEEDED, 0.0)
    (send,) = sends(policy.planned(1, 2))
    policy.goal_result(send.token, Result.SUCCEEDED, 0.0)
    assert policy.state == State.DONE and policy.progress.lap == 2


def test_a_restored_mission_waits_and_replans_at_its_index():
    policy = RunnerPolicy()
    policy.supervisor_state("NORMAL", 0.0)
    policy.restore(Progress(mission=MISSION, index=4, lap=1, skipped=[2]))
    assert policy.state == State.PAUSED          # never drives off by itself
    ok, _, actions = policy.resume()
    assert ok and actions == [Plan(lap=1)]
    (send,) = sends(policy.planned(10, 3))
    assert send.index == 4 and policy.state == State.RUNNING
    assert policy.progress.skipped == [2]


def test_a_failed_plan_is_reported():
    policy = RunnerPolicy()
    policy.supervisor_state("NORMAL", 0.0)
    policy.start(MISSION)
    policy.plan_failed("no row of the area is longer than 0.3 m")
    assert policy.state == State.FAILED and "no row" in policy.reason
    assert policy.start(MISSION)[0]


@pytest.mark.parametrize("state_fn", ["pause", "resume", "abort"])
def test_commands_in_idle_are_refused(state_fn):
    ok, why, actions = getattr(RunnerPolicy(), state_fn)()
    assert not ok and actions == [] and "IDLE" in why


def test_skips_in_a_row_pause_the_mission():
    # Nav2 down: every goal aborts. Three skips in a row is not "the lawn has
    # three unreachable corners", it is a robot that is not driving.
    policy, token = running(total=10, max_retries=0, max_skips_in_a_row=3)
    for expected_index in (1, 2):
        (token_send,) = sends(policy.goal_result(token, Result.ABORTED, 0.0))
        assert token_send.index == expected_index
        token = token_send.token
    assert sends(policy.goal_result(token, Result.ABORTED, 0.0)) == []
    assert policy.state == State.PAUSED and "in a row" in policy.reason
    assert policy.progress.skipped == [0, 1, 2]


def test_a_success_resets_the_skip_count():
    policy, token = running(total=10, max_retries=0, max_skips_in_a_row=2)
    (send,) = sends(policy.goal_result(token, Result.ABORTED, 0.0))     # skip 0
    (send,) = sends(policy.goal_result(send.token, Result.SUCCEEDED, 0.0))
    (send,) = sends(policy.goal_result(send.token, Result.ABORTED, 0.0))  # skip 2
    assert policy.state == State.RUNNING and send.index == 3
