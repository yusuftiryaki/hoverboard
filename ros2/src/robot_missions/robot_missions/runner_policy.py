"""mission_runner's decisions. ROS-free, no I/O — the battery_supervisor pattern.

    IDLE ──start──▶ PLANNING ──planned──▶ RUNNING ──last waypoint──▶ DONE
                                           │  ▲
            battery_supervisor leaves      │  │ supervisor back in NORMAL
            NORMAL (LOW, return_home)      ▼  │ (after charge + undock)
                                        WAITING_DOCK ──DOCKED_IDLE──▶ (asks the supervisor to resume)
    RUNNING / WAITING_DOCK ──pause, SAFE_STOP, a cancel nobody explains──▶ PAUSED ──resume──▶ RUNNING
    anything ──abort──▶ IDLE

The node feeds events (start, planned, goal results, supervisor states,
commands, time) and executes the Actions that come back.

Rules that are easy to get wrong, and why:

  * ⚠️ A CANCELED GOAL IS NEVER RE-SENT ON ITS OWN. The supervisor cancels every
    mission goal before it drives home — and its own staging goal goes to the
    same navigate_to_pose server, where a new goal PREEMPTS the running one. A
    runner that answered "canceled" with "send it again" would hijack the trip
    to the charger. A cancel the supervisor does not explain within
    cancel_grace_s (someone ran `ros2 action ... cancel`) parks the mission in
    PAUSED for a person to resume.
  * Goals are only sent while the supervisor's last word was NORMAL. The
    remaining race — LOW arrives between the check and the send — makes the
    supervisor's staging fail into SAFE_STOP: the robot stops. Safe, and
    rare; documented rather than papered over.
  * An ABORTED goal (Nav2 could not get there: inside an obstacle's inflation,
    no path) is retried max_retries times, then SKIPPED and recorded. One
    unreachable row end must not end a mowing job.
  * ⚠️ ...but max_skips_in_a_row skips in a row PAUSE the mission. That is not
    a waypoint problem, it is Nav2 (down, not discovered yet, a broken
    profile). Seen for real: a runner that started before it had discovered
    Nav2's servers "finished" a 23-waypoint lawn in one second, every waypoint
    skipped, state DONE.
  * Results carry the token of the goal they answer. A result for an older
    goal (canceled, superseded) changes nothing.
  * The supervisor is asked to undock only once per wait, and only from
    DOCKED_IDLE (full) — never from CHARGING.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class State(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    RUNNING = "RUNNING"
    WAITING_DOCK = "WAITING_DOCK"
    PAUSED = "PAUSED"
    DONE = "DONE"
    FAILED = "FAILED"


class Result(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"


# battery_supervisor phases (supervisor_policy.Phase values).
SUP_NORMAL = "NORMAL"
SUP_DOCKED_IDLE = "DOCKED_IDLE"
SUP_SAFE_STOP = "SAFE_STOP"


# --- actions ------------------------------------------------------------------

@dataclass(frozen=True)
class Plan:
    """Ask the node to (re)plan: convert the site, build the waypoints."""
    lap: int


@dataclass(frozen=True)
class Send:
    token: int
    index: int


@dataclass(frozen=True)
class Cancel:
    token: int


@dataclass(frozen=True)
class ResumeSupervisor:
    pass


@dataclass(frozen=True)
class Persist:
    pass


Action = Any


@dataclass
class Progress:
    """What survives a restart (written by the node on every Persist)."""

    mission: Optional[Dict[str, Any]] = None
    index: int = 0
    lap: int = 0
    skipped: List[int] = field(default_factory=list)


class RunnerPolicy:
    def __init__(self, max_retries: int = 1, cancel_grace_s: float = 3.0,
                 max_skips_in_a_row: int = 3):
        self.max_retries = max_retries
        self.cancel_grace_s = cancel_grace_s
        self.max_skips_in_a_row = max_skips_in_a_row
        self._skips_in_a_row = 0
        self.state = State.IDLE
        self.reason = "no mission"
        self.progress = Progress()
        self.total = 0                  # waypoints in the current plan
        self.repeat = 1                 # 0 = until stopped
        self.supervisor: Optional[str] = None
        self._token = 0
        # Token of the live goal. Cancelling OUR goal (pause, abort) or
        # handing the robot to the supervisor drops it, so that goal's answer
        # arrives as an old token and is ignored — no "expected cancel" flag.
        self._in_flight: Optional[int] = None
        self._attempts: Dict[int, int] = {}
        self._unexplained_cancel_at: Optional[float] = None
        self._asked_supervisor = False
        self._resume_after_plan = False

    # --- commands -------------------------------------------------------------

    def start(self, mission: Dict[str, Any]) -> tuple:
        if self.state in (State.PLANNING, State.RUNNING, State.WAITING_DOCK, State.PAUSED):
            return False, f"a mission is already {self.state.value}", []
        if self.supervisor != SUP_NORMAL:
            return False, (f"battery_supervisor is {self.supervisor or 'not heard from'}, "
                           "not NORMAL"), []
        self.progress = Progress(mission=mission)
        self._attempts = {}
        self._enter(State.PLANNING, "planning")
        return True, "planning", [Plan(lap=0), Persist()]

    def restore(self, progress: Progress) -> List[Action]:
        """After a restart: the mission comes back PAUSED. A robot that wakes
        up and drives off on its own after a crash, with nobody told, is not a
        default anyone should get by accident."""
        if progress.mission is None:
            return []
        self.progress = progress
        self._enter(State.PAUSED, "restored after a restart — resume to continue")
        return []

    def pause(self) -> tuple:
        if self.state == State.RUNNING:
            actions = self._cancel_in_flight()
            self._enter(State.PAUSED, "paused by request")
            return True, "paused", actions + [Persist()]
        if self.state == State.WAITING_DOCK:
            self._enter(State.PAUSED, "paused by request (was waiting on the dock)")
            return True, "paused", [Persist()]
        return False, f"nothing to pause in {self.state.value}", []

    def resume(self) -> tuple:
        if self.state != State.PAUSED:
            return False, f"nothing to resume in {self.state.value}", []
        if self.total == 0:
            # Restored from disk: the plan is gone with the old process (and
            # the old `map`). Re-plan, then carry on at the saved index.
            self._resume_after_plan = True
            self._enter(State.PLANNING, "re-planning to resume")
            return True, "re-planning", [Plan(lap=self.progress.lap)]
        return (True, "resuming", self._carry_on("resumed by request"))

    def abort(self) -> tuple:
        if self.state in (State.IDLE, State.DONE, State.FAILED):
            return False, f"nothing to abort in {self.state.value}", []
        actions = self._cancel_in_flight()
        self.progress = Progress()
        self.total = 0
        self._enter(State.IDLE, "aborted by request")
        return True, "aborted", actions + [Persist()]

    # --- events ---------------------------------------------------------------

    def planned(self, total: int, repeat: int) -> List[Action]:
        if self.state != State.PLANNING:
            return []
        if total < 1:
            return self.plan_failed("the plan has no waypoints")
        self.total, self.repeat = total, repeat
        if self.progress.index >= total:
            self.progress.index = 0
        if self._resume_after_plan:
            self._resume_after_plan = False
            return self._carry_on("resumed after re-planning")
        self._enter(State.RUNNING, "running")
        return self._send_current()

    def plan_failed(self, why: str) -> List[Action]:
        self._resume_after_plan = False
        self.total = 0
        self._enter(State.FAILED, f"planning failed: {why}")
        return [Persist()]

    def goal_result(self, token: int, result: Result, now: float) -> List[Action]:
        if token != self._in_flight:
            return []                   # an older goal's answer
        self._in_flight = None
        if result == Result.CANCELED:
            # Not ours (ours drop the token first): the supervisor's, or a
            # person's at the command line.
            if self.state == State.RUNNING:
                if self.supervisor != SUP_NORMAL:
                    return self._wait_dock()
                self._unexplained_cancel_at = now
                self.reason = "goal canceled — waiting to see if the supervisor explains it"
            return []
        if self.state != State.RUNNING:
            return []
        index = self.progress.index
        if result == Result.ABORTED:
            self._attempts[index] = self._attempts.get(index, 0) + 1
            if self._attempts[index] <= self.max_retries:
                return self._send_current()
            self.progress.skipped.append(index)
            self._skips_in_a_row += 1
            if self._skips_in_a_row >= self.max_skips_in_a_row:
                self._skips_in_a_row = 0
                self._enter(State.PAUSED, f"{self.max_skips_in_a_row} waypoints in a row "
                                          "failed — Nav2 is not driving; resume to retry")
                return [Persist()]
        else:
            self._skips_in_a_row = 0
        return self._advance()

    def supervisor_state(self, phase: str, now: float) -> List[Action]:
        self.supervisor = phase
        if phase == SUP_SAFE_STOP and self.state in (State.RUNNING, State.WAITING_DOCK):
            self._in_flight = None      # the supervisor cancels everything
            self._enter(State.PAUSED, "battery_supervisor is in SAFE_STOP — a person "
                                      "must look; resume to continue")
            return [Persist()]
        if self.state == State.RUNNING and phase != SUP_NORMAL:
            return self._wait_dock()
        if self.state == State.WAITING_DOCK:
            if phase == SUP_NORMAL:
                return self._carry_on("back from the dock")
            if phase == SUP_DOCKED_IDLE and not self._asked_supervisor:
                self._asked_supervisor = True
                self.reason = "charged — asking the supervisor to undock"
                return [ResumeSupervisor()]
        return []

    def supervisor_refused(self, why: str) -> List[Action]:
        """The supervisor would not undock (or did not answer)."""
        if self.state != State.WAITING_DOCK:
            return []
        self._enter(State.PAUSED, f"battery_supervisor would not undock: {why}")
        return [Persist()]

    def tick(self, now: float) -> List[Action]:
        if (self._unexplained_cancel_at is not None and self.state == State.RUNNING
                and now - self._unexplained_cancel_at >= self.cancel_grace_s):
            self._unexplained_cancel_at = None
            self._enter(State.PAUSED, "a goal was canceled by someone other than the "
                                      "supervisor — resume to continue")
            return [Persist()]
        return []

    # --- internals ------------------------------------------------------------

    def _advance(self) -> List[Action]:
        self.progress.index += 1
        if self.progress.index >= self.total:
            self.progress.lap += 1
            if self.repeat != 0 and self.progress.lap >= self.repeat:
                self._enter(State.DONE, f"done ({len(self.progress.skipped)} skipped)")
                return [Persist()]
            self.progress.index = 0
            self._attempts = {}
            # Each lap is re-planned: a random patrol draws new points with
            # seed + lap; the others come back the same.
            self.total = 0
            self._enter(State.PLANNING, f"planning lap {self.progress.lap + 1}")
            return [Persist(), Plan(lap=self.progress.lap)]
        return [Persist(), *self._send_current()]

    def _send_current(self) -> List[Action]:
        if self.supervisor != SUP_NORMAL:
            return self._wait_dock()
        self._token += 1
        self._in_flight = self._token
        self._unexplained_cancel_at = None
        return [Send(self._token, self.progress.index)]

    def _carry_on(self, why: str) -> List[Action]:
        self._asked_supervisor = False
        if self.supervisor == SUP_DOCKED_IDLE:
            self._enter(State.WAITING_DOCK, "asking the supervisor to undock")
            self._asked_supervisor = True
            return [ResumeSupervisor()]
        if self.supervisor != SUP_NORMAL:
            return self._wait_dock()
        self._enter(State.RUNNING, why)
        return self._send_current()

    def _wait_dock(self) -> List[Action]:
        # The supervisor has (or is about to) cancel the live goal; its answer
        # is not ours to act on.
        self._in_flight = None
        self._unexplained_cancel_at = None
        self._asked_supervisor = False
        self._enter(State.WAITING_DOCK, f"battery_supervisor is {self.supervisor} — "
                                        "waiting for it to finish")
        return [Persist()]

    def _cancel_in_flight(self) -> List[Action]:
        if self._in_flight is None:
            return []
        token, self._in_flight = self._in_flight, None
        return [Cancel(token)]

    def _enter(self, state: State, reason: str) -> None:
        self.state, self.reason = state, reason

    @property
    def in_flight(self) -> Optional[int]:
        """The token whose result the policy is waiting for (None: none)."""
        return self._in_flight

    def status(self) -> Dict[str, Any]:
        return {"state": self.state.value, "reason": self.reason,
                "mission": self.progress.mission, "index": self.progress.index,
                "total": self.total, "lap": self.progress.lap, "repeat": self.repeat,
                "skipped": list(self.progress.skipped), "supervisor": self.supervisor}
