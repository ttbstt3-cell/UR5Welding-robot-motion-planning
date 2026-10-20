from __future__ import annotations

import time
from enum import Enum
from typing import Callable

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class MotionState(str, Enum):
    """Explicit lifecycle of the single in-flight FollowJointTrajectory goal."""

    IDLE = "IDLE"
    SENDING = "SENDING"
    EXECUTING = "EXECUTING"
    CANCELING = "CANCELING"
    FAULT_UNKNOWN = "FAULT_UNKNOWN"


# action_msgs/msg/GoalStatus constants used to classify a returned result.
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

# If a cancel request gets no result within this time the robot motion state is
# unknown.  This stage intentionally does NOT auto-unlock: `busy` stays True and
# motion stays blocked until the HMI is restarted by the operator.
CANCEL_RESULT_TIMEOUT_SEC = 10.0

FAULT_CANCEL_TIMEOUT_MESSAGE = (
    "[FAULT] Cancel result timeout; robot motion state is unknown. Motion remains blocked."
)


class TrajectoryClient:
    """Small non-blocking FollowJointTrajectory wrapper.

    All callbacks execute on the ROS worker thread.  No call waits in the Qt
    GUI thread, so a lost driver connection cannot freeze the interface.

    The client owns two pieces of state that the GUI reads directly:

    * ``busy``   - True from the moment a goal is sent until its outcome is
                   known (SUCCESS / ABORTED / CANCELED / rejected).  A rejected
                   goal never existed, so it does not stay busy.
    * ``motion_state`` / ``current_goal_label`` - operator-facing detail.

    ``busy`` never returns to False on a timeout: an unknown outcome keeps the
    robot blocked on purpose.
    """

    def __init__(
        self,
        node: Node,
        action_name: str,
        on_active: Callable[[bool], None],
        on_done: Callable[[bool, str], None],
        on_motion: Callable[[str, bool, str], None] | None = None,
        on_log: Callable[[str], None] | None = None,
        on_feedback: Callable[[dict], None] | None = None,
    ):
        self.node = node
        self.on_active = on_active
        self.on_done = on_done
        self.on_motion = on_motion if on_motion is not None else (lambda *_args: None)
        self.on_log = on_log if on_log is not None else (lambda _message: None)
        self.on_feedback = on_feedback
        self._feedback_generation = 0
        self.client = ActionClient(node, FollowJointTrajectory, action_name)
        self.goal_handle = None
        self._goal_future = None
        self._result_future = None
        # motion_busy: True from the moment a goal is sent until that goal
        # reaches a known terminal outcome.
        self._busy = False
        self._state = MotionState.IDLE
        self._goal_label = ""
        self._sequences = {"JOG": 0, "TRAJ": 0}
        self._cancel_deadline = None
        self._cancel_timed_out = False
        self._timeout_timer = node.create_timer(0.5, self._check_cancel_timeout)

    # ------------------------------------------------------------------
    # State readable from the GUI thread (plain atomic attribute reads).
    # ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        """True while a goal is in flight (sent but outcome not yet returned)."""
        return self._busy

    @property
    def motion_state(self) -> MotionState:
        return self._state

    @property
    def current_goal_label(self) -> str:
        return self._goal_label

    @property
    def cancel_timed_out(self) -> bool:
        return self._cancel_timed_out

    def ready(self) -> bool:
        return self.client.server_is_ready()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _set_state(self, state: MotionState) -> None:
        self._state = state
        self.on_motion(state.value, self._busy, self._goal_label)

    def _emit_log(self, message: str) -> None:
        self.on_log(message)

    def _clear_cancel_watch(self) -> None:
        self._cancel_deadline = None

    def _check_cancel_timeout(self) -> None:
        """Timer callback: detect a cancel that never produced a result."""
        if self._cancel_deadline is None or self._cancel_timed_out:
            return
        if time.monotonic() < self._cancel_deadline:
            return
        self._cancel_timed_out = True
        self._clear_cancel_watch()
        # Deliberately keep `_busy` True: the robot may still be moving.
        self._set_state(MotionState.FAULT_UNKNOWN)
        self._emit_log(FAULT_CANCEL_TIMEOUT_MESSAGE)

    # ------------------------------------------------------------------
    # Goal lifecycle
    # ------------------------------------------------------------------
    def send(self, trajectory: JointTrajectory, kind: str = "JOG", detail: str = "") -> bool:
        # `goal_handle` is only assigned in `_goal_response`, i.e. after the
        # action server has accepted the goal.  That happens asynchronously
        # after `send_goal_async()` has already returned, so guarding on
        # `goal_handle` alone leaves a window in which a second
        # `send_goal_async()` (and therefore a second concurrent
        # FollowJointTrajectory goal) could be issued.  `_busy` is set
        # synchronously here and is cleared only on a known outcome, so at most
        # one goal can ever be outstanding.
        if self._busy or self.goal_handle is not None:
            return False
        kind = str(kind).upper()
        if kind not in self._sequences:
            self._sequences[kind] = 0
        self._sequences[kind] += 1
        self._goal_label = f"{kind} #{self._sequences[kind]:03d}"
        self._cancel_timed_out = False
        self._clear_cancel_watch()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._busy = True
        self._feedback_generation += 1
        generation = self._feedback_generation
        self._set_state(MotionState.SENDING)
        suffix = f" {detail}" if detail else ""
        self._emit_log(f"[{self._goal_label}] SEND{suffix}")
        try:
            self._goal_future = self.client.send_goal_async(goal,
                feedback_callback=lambda message: self._feedback_callback(message, generation, kind))
        except Exception as exc:  # pragma: no cover - middleware error path
            # Submission failed: we cannot know whether a goal was created.
            self._set_state(MotionState.FAULT_UNKNOWN)
            self._emit_log(f"[{self._goal_label}] SEND FAILED: {exc}")
            self.on_done(False, f"goal submission failed: {exc}")
            return False
        self._goal_future.add_done_callback(self._goal_response)
        return True

    def _feedback_callback(self, message, generation, kind):
        if (generation != self._feedback_generation or not self._busy
                or kind != "TRAJ" or self.on_feedback is None):
            return
        try:
            from trajectory.execution_feedback import decode_feedback
            record = decode_feedback(message, self._goal_label, time.monotonic())
        except (AttributeError, ValueError, TypeError):
            return
        self.on_feedback(record)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # pragma: no cover - middleware error path
            self._set_state(MotionState.FAULT_UNKNOWN)
            self._emit_log(f"[{self._goal_label}] GOAL RESPONSE FAILED: {exc}")
            self.on_done(False, f"goal submission failed: {exc}")
            return
        if handle is None or not handle.accepted:
            # No goal exists, so nothing will ever produce a result.
            self._busy = False
            self._set_state(MotionState.IDLE)
            self._emit_log(f"[{self._goal_label}] REJECTED")
            self.on_done(False, "trajectory goal rejected")
            return
        self.goal_handle = handle
        self._set_state(MotionState.EXECUTING)
        self._emit_log(f"[{self._goal_label}] ACCEPTED")
        self.on_active(True)
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        self._clear_cancel_watch()
        try:
            wrapped = future.result()
            status = int(getattr(wrapped, "status", -1))
            result = wrapped.result
            error_code = int(getattr(result, "error_code", -1))
        except Exception as exc:  # pragma: no cover
            # Result could not be interpreted: outcome unknown, stay blocked.
            self._set_state(MotionState.FAULT_UNKNOWN)
            self._emit_log(f"[{self._goal_label}] RESULT FAILED: {exc}")
            self.on_done(False, f"trajectory result failed: {exc}")
            return
        if status == STATUS_CANCELED:
            outcome, ok, message = "CANCELED", False, "trajectory canceled"
        elif status == STATUS_ABORTED:
            outcome, ok = "ABORTED", False
            message = f"trajectory failed, error_code={error_code}"
        else:
            ok = error_code == 0
            outcome = "SUCCESS" if ok else "ABORTED"
            message = "trajectory completed" if ok else f"trajectory failed, error_code={error_code}"
        # Only here is the goal really finished (SUCCESS / ABORTED / CANCELED).
        self._busy = False
        self.goal_handle = None
        self._result_future = None
        self._set_state(MotionState.IDLE)
        self._emit_log(f"[{self._goal_label}] {outcome}")
        self.on_active(False)
        self.on_done(ok, message)

    def cancel(self) -> bool:
        if self.goal_handle is None:
            return False
        self.goal_handle.cancel_goal_async()
        # `_busy` deliberately stays True: the goal is not finished until the
        # server returns its CANCELED/ABORTED/SUCCESS result, and a new goal
        # must not be sent before that.
        self.goal_handle = None
        self._cancel_deadline = time.monotonic() + CANCEL_RESULT_TIMEOUT_SEC
        self._cancel_timed_out = False
        self._set_state(MotionState.CANCELING)
        self._emit_log(f"[{self._goal_label}] CANCEL REQUESTED")
        self.on_active(False)
        self.on_done(False, "trajectory goal cancel requested")
        return True
