from __future__ import annotations

import time
from dataclasses import dataclass

from safety.joint_limits import within_limits


@dataclass
class SafetyResult:
    ok: bool
    reasons: list[str]


def check_motion_ready(state, target=None, max_start_error_rad=0.035) -> SafetyResult:
    reasons = []
    if len(state.positions) != 6 or time.monotonic() - state.joint_stamp > 1.0:
        reasons.append("joint_states 未持续更新")
    if state.program_running is not True:
        reasons.append("External Control/robot_program_running 不是 true")
    if state.robot_mode != 7:
        reasons.append(f"robot_mode={state.robot_mode}，不是 RUNNING(7)")
    if state.safety_mode != 1:
        reasons.append(f"safety_mode={state.safety_mode}，不是 NORMAL(1)")
    if state.speed_scaling is None or state.speed_scaling <= 0.0:
        reasons.append("speed_scaling 不大于 0")
    if not state.controller_active:
        reasons.append("scaled_joint_trajectory_controller 未 active")
    if not state.action_server_ready:
        reasons.append("FollowJointTrajectory action server 未就绪")
    if target is not None:
        if not within_limits(target):
            reasons.append("目标关节角超出软件限位")
        if len(state.positions) == 6 and max(abs(float(a) - float(b)) for a, b in zip(target, state.positions)) > max_start_error_rad:
            reasons.append("目标轨迹起点与最新真实关节角不一致")
    return SafetyResult(ok=not reasons, reasons=reasons)
