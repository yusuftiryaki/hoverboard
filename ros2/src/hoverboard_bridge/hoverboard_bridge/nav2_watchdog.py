"""Nav2 must come up — and stay up — or the whole bringup restarts.

    nav2_watchdog ──is_active?──► lifecycle_manager_navigation
          │  no answer / "not active" for longer than `timeout_s`
          ▼
       exits 3 ─► nav2.launch.py: Shutdown ─► ros2 launch ends ─► systemd restarts robot.service

Why: Nav2 1.3.13's lifecycle manager can wait FOREVER. MEASURED (dev machine,
then twice in CI, 2026-10-16): it logged "Configuring controller_server" and
nothing else, ever, while controller_server had logged

    failed to send response to /controller_server/change_state (timeout):
    client will not receive response

rmw_fastrtps answered before it had matched the manager's reader, the answer
was dropped, and nav2_util's change_state(transition) waits for it with no
timeout. Nav2 never activates and nothing says so: the bridge, the EKF and
teleop are all fine, every goal is simply rejected. On the Pi at boot that is
a robot that looks alive and will never navigate.

Waiting longer cannot fix it and the manager cannot be asked to retry — while
it is stuck it does not answer at all — so the only cure is new processes.
This node does not restart anything itself; it says why, loudly, and exits.
nav2.launch.py turns that exit into a launch-wide Shutdown, and systemd
(docs/deployment.md step 8, Restart=always) brings everything back. The robot
is standing still at that point anyway: Nav2 was never driving it, and the
bridge going down stops the wheels through the ESP32 watchdog.

Not only at boot: a Nav2 that was active and then stops answering, or reports
inactive, for `timeout_s` gets the same treatment.

The decision is BringupWatch, ROS-free and unit-tested; main() is the plumbing.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional


# What nav2.launch.py's exit handler listens for (and nothing else): a crash
# of this node exits 1 and must not restart a working robot.
NAV2_STUCK_EXIT = 3


class Verdict(str, Enum):
    WAITING = "WAITING"      # not active yet, still inside the allowance
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"        # give up: restart the bringup


class BringupWatch:
    """Feed it every is_active outcome; it says when to give up.

    `answer` is True (active), False (answered "not active") or None (no
    answer within the call's own timeout — the stuck-manager signature)."""

    def __init__(self, timeout_s: float, start: float) -> None:
        if not timeout_s > 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._since = start              # start of the current not-active stretch
        self.ever_active = False
        self.unanswered = 0
        self.inactive = 0
        self.reason = ""

    def feed(self, answer: Optional[bool], now: float) -> Verdict:
        if answer is True:
            self.ever_active = True
            self._since = now
            self.unanswered = self.inactive = 0
            return Verdict.ACTIVE
        if answer is None:
            self.unanswered += 1
        else:
            self.inactive += 1
        if now - self._since < self.timeout_s:
            return Verdict.WAITING
        phase = "stopped being active" if self.ever_active else "never became active"
        if self.unanswered >= self.inactive:
            why = (f"the lifecycle manager did not answer is_active "
                   f"({self.unanswered}x) — it is stuck, most likely on a dropped "
                   f"change_state response")
        else:
            why = (f"the lifecycle manager answered 'not active' ({self.inactive}x) "
                   f"— a server would not transition; read its log")
        self.reason = f"Nav2 {phase} within {self.timeout_s:g} s: {why}"
        return Verdict.FAILED


def main(args=None) -> None:
    import sys

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class Nav2Watchdog(Node):
        def __init__(self) -> None:
            super().__init__("nav2_watchdog")
            # A Pi 4 brings Nav2 up in tens of seconds; CI's worst healthy
            # bring-up measured here was ~30 s. Generous, because a false
            # alarm restarts the whole robot.
            self.declare_parameter("timeout_s", 120.0)
            self.declare_parameter("period_s", 2.0)
            # One call's budget. A healthy manager answers in milliseconds once
            # active; during activation it may block for seconds.
            self.declare_parameter("call_timeout_s", 5.0)
            self.declare_parameter("manager", "lifecycle_manager_navigation")
            p = self.get_parameter
            self._call_timeout = float(p("call_timeout_s").value)
            self._watch = BringupWatch(float(p("timeout_s").value), self._now())
            self._client = self.create_client(Trigger, f"{p('manager').value}/is_active")
            self._diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._pending = None
            self._sent_at = 0.0
            self._verdict = Verdict.WAITING
            self.create_timer(float(p("period_s").value), self._tick)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _tick(self) -> None:
            now = self._now()
            if self._pending is not None:
                if self._pending.done():
                    result = self._pending.result()
                    self._judge(bool(result and result.success), now)
                    self._pending = None
                elif now - self._sent_at > self._call_timeout:
                    self._pending = None
                    self._judge(None, now)
                return
            if self._client.service_is_ready():
                self._pending = self._client.call_async(Trigger.Request())
                self._sent_at = now
            else:
                self._judge(None, now)

        def _judge(self, answer, now) -> None:
            verdict = self._watch.feed(answer, now)
            if verdict != self._verdict:
                if verdict == Verdict.ACTIVE:
                    self.get_logger().info("Nav2 is active")
                elif self._verdict == Verdict.ACTIVE:
                    self.get_logger().warn("Nav2 is not active any more — watching")
            self._verdict = verdict
            self._publish(verdict)
            if verdict == Verdict.FAILED:
                self.get_logger().fatal(
                    f"{self._watch.reason}. Exiting so the bringup restarts "
                    f"(nav2.launch.py -> Shutdown -> systemd).")
                raise SystemExit(NAV2_STUCK_EXIT)

        def _publish(self, verdict) -> None:
            status = DiagnosticStatus(name="nav2_watchdog: Nav2 bring-up",
                                      hardware_id="nav2")
            status.level = {Verdict.ACTIVE: DiagnosticStatus.OK,
                            Verdict.WAITING: DiagnosticStatus.WARN,
                            Verdict.FAILED: DiagnosticStatus.ERROR}[verdict]
            status.message = (self._watch.reason if verdict == Verdict.FAILED
                              else f"Nav2 {verdict.value.lower()}")
            status.values = [
                KeyValue(key="unanswered", value=str(self._watch.unanswered)),
                KeyValue(key="answered_inactive", value=str(self._watch.inactive)),
                KeyValue(key="ever_active", value=str(self._watch.ever_active)),
            ]
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [status]
            self._diag.publish(array)

    rclpy.init(args=args)
    node = Nav2Watchdog()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as stop:
        code = int(stop.code or 0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
