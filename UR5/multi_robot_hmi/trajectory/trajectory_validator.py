from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from safety.joint_limits import JOINT_NAMES, LOWER, UPPER
from trajectory.trajectory_utils import TrajectoryData, normalize_trajectory


@dataclass
class ValidationReport:
    ok: bool
    issues: list[str] = field(default_factory=list)
    point_count: int = 0
    duration: float | None = None
    start_error_rad: np.ndarray | None = None
    max_joint_travel_rad: np.ndarray | None = None
    max_adjacent_step_rad: float = 0.0
    estimated_max_velocity: float | None = None
    estimated_max_acceleration: float | None = None
    time_valid: bool = False


def validate_trajectory(
    trajectory: TrajectoryData,
    q_actual=None,
    start_tolerance_rad: float = math.radians(1.0),
    max_step_rad: float = 0.8,
    joint_names=None,
    joint_limits=None,
) -> ValidationReport:
    expected_names = list(joint_names or JOINT_NAMES)
    traj = normalize_trajectory(trajectory, joint_names=expected_names)
    issues: list[str] = []
    q = np.asarray(traj.positions, dtype=float)
    n = int(q.shape[0]) if q.ndim == 2 else 0
    if q.ndim != 2 or q.shape[1] != 6 or n < 1:
        issues.append("positions 必须是至少一行的 N×6")
        return ValidationReport(False, issues, n)
    if traj.joint_names != expected_names:
        issues.append("joint order 不符合 ROS/URDF 标准顺序")
    if not np.isfinite(q).all():
        issues.append("positions 含 NaN/Inf")
    limits = tuple(joint_limits or zip(LOWER, UPPER))
    for i, row in enumerate(q):
        if not all(float(lo) <= float(value) <= float(hi)
                   for value, (lo, hi) in zip(row, limits)):
            issues.append(f"point {i + 1}: software joint limit")

    jumps = np.max(np.abs(np.diff(q, axis=0)), axis=1) if n > 1 else np.zeros(0)
    max_jump = float(np.max(jumps)) if jumps.size else 0.0
    if jumps.size:
        for i in np.where(jumps > max_step_rad)[0]:
            issues.append(f"edge {int(i) + 1}: jump {jumps[i]:.4f} rad")

    time_valid = traj.time_from_start is not None
    duration = None
    max_velocity = None
    max_acceleration = None
    if not time_valid:
        issues.append("TIME PARAMETERIZATION REQUIRED")
    else:
        t = np.asarray(traj.time_from_start, dtype=float).reshape(-1)
        if t.size != n or not np.isfinite(t).all():
            issues.append("time_from_start 长度或数值无效")
            time_valid = False
        elif t[0] < 0 or np.any(np.diff(t) <= 0):
            issues.append("time_from_start 必须严格递增且从非负时间开始")
            time_valid = False
        else:
            duration = float(t[-1])
            if n > 1:
                dt = np.diff(t)
                vel = np.diff(q, axis=0) / dt[:, None]
                max_velocity = float(np.max(np.abs(vel)))
                if np.any(~np.isfinite(vel)):
                    issues.append("estimated velocity contains NaN/Inf")
                if n > 2:
                    acc = np.diff(vel, axis=0) / dt[1:, None]
                    max_acceleration = float(np.max(np.abs(acc)))
                    if np.any(~np.isfinite(acc)):
                        issues.append("estimated acceleration contains NaN/Inf")
    if traj.velocities is not None:
        if traj.velocities.shape != q.shape or not np.isfinite(traj.velocities).all():
            issues.append("velocities 必须为 N×6 且不能含 NaN/Inf")
    if traj.accelerations is not None:
        if traj.accelerations.shape != q.shape or not np.isfinite(traj.accelerations).all():
            issues.append("accelerations 必须为 N×6 且不能含 NaN/Inf")

    start_error = None
    if q_actual is not None:
        actual = np.asarray(q_actual, dtype=float).reshape(-1)
        if actual.size != 6 or not np.isfinite(actual).all():
            issues.append("当前实时关节反馈无效")
        else:
            start_error = q[0] - actual
            if np.max(np.abs(start_error)) > start_tolerance_rad:
                issues.append(
                    "trajectory q_start 与实时 q_actual 超过 %.1f°" % math.degrees(start_tolerance_rad)
                )

    max_travel = np.max(np.abs(q - q[0]), axis=0)
    return ValidationReport(
        ok=not issues,
        issues=issues,
        point_count=n,
        duration=duration,
        start_error_rad=start_error,
        max_joint_travel_rad=max_travel,
        max_adjacent_step_rad=max_jump,
        estimated_max_velocity=max_velocity,
        estimated_max_acceleration=max_acceleration,
        time_valid=time_valid,
    )


def validate_path(path, max_step_rad=0.8):
    """Legacy API retained for existing callers."""
    traj = normalize_trajectory(path)
    report = validate_trajectory(traj, max_step_rad=max_step_rad)
    return traj.positions, report.issues
