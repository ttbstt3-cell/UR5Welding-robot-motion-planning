"""Fixed-point TCP position calibration from flange poses.

The solver deliberately accepts only base-to-flange transforms.  A TCP pose
topic is not an input because it already contains the controller's active TCP
offset and would make the calibration circular.

For each pose ``T_i = [R_i, t_i]`` and a fixed contact point ``p_b`` in the
robot base frame, the flange-frame TCP offset ``p_f`` satisfies::

    R_i p_f + t_i = p_b

The six unknowns are solved by least squares from four or more distinct
orientations.  The module contains no ROS or motion-control code.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


class PivotCalibrationError(ValueError):
    """Raised when a set of flange poses cannot support a reliable fit."""


@dataclass(frozen=True)
class PivotFitResult:
    tcp_flange_m: np.ndarray
    pivot_base_m: np.ndarray
    residuals_m: np.ndarray
    rms_residual_m: float
    max_residual_m: float
    matrix_rank: int
    condition_number: float
    orientation_span_deg: float
    pose_count: int

    @property
    def rms_residual_mm(self) -> float:
        return float(self.rms_residual_m * 1000.0)

    @property
    def max_residual_mm(self) -> float:
        return float(self.max_residual_m * 1000.0)

    @property
    def valid(self) -> bool:
        """Whether the four-point least-squares solve is numerically usable.

        RMS/max residual are reported as a quality band by the HMI.  They are
        intentionally not part of this basic solve flag so a 2--5 mm result
        can be saved as a draft for review.  Hardware/planning application
        gates apply their own stricter precision checks.
        """
        return (
            self.pose_count >= 4
            and self.matrix_rank >= 6
            and self.orientation_span_deg >= 10.0
            and np.isfinite(self.condition_number)
            and np.all(np.isfinite(self.tcp_flange_m))
            and np.all(np.isfinite(self.pivot_base_m))
        )

    def to_dict(self) -> dict:
        return {
            "tcp_flange_m": self.tcp_flange_m.tolist(),
            "pivot_base_m": self.pivot_base_m.tolist(),
            "residuals_mm": (self.residuals_m * 1000.0).tolist(),
            "rms_residual_mm": self.rms_residual_mm,
            "max_residual_mm": self.max_residual_mm,
            "matrix_rank": int(self.matrix_rank),
            "condition_number": float(self.condition_number),
            "orientation_span_deg": float(self.orientation_span_deg),
            "pose_count": int(self.pose_count),
            "valid": bool(self.valid),
        }


def _as_transform(value, index: int) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise PivotCalibrationError(f"flange pose {index + 1} is not a finite 4x4 transform")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise PivotCalibrationError(f"flange pose {index + 1} has an invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5) or np.linalg.det(rotation) <= 0.0:
        raise PivotCalibrationError(f"flange pose {index + 1} rotation is not orthonormal")
    return matrix


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _orientation_span_deg(rotations: list[np.ndarray]) -> float:
    if len(rotations) < 2:
        return 0.0
    return max(
        _rotation_angle_deg(a.T @ b)
        for i, a in enumerate(rotations)
        for b in rotations[i + 1 :]
    )


def fit_pivot_calibration(flange_poses: Iterable[np.ndarray], *, min_span_deg: float = 10.0) -> PivotFitResult:
    """Fit flange->TCP position from 4--6 base->flange poses.

    The pose count is intentionally bounded.  A caller can capture more raw
    samples, but must select a small, controlled calibration set before
    solving so one accidental duplicate pose cannot silently dominate it.
    """
    poses = [_as_transform(item, i) for i, item in enumerate(flange_poses)]
    if not 4 <= len(poses) <= 6:
        raise PivotCalibrationError("TCP pivot calibration requires 4 to 6 flange poses")
    rotations = [pose[:3, :3] for pose in poses]
    span_deg = _orientation_span_deg(rotations)
    if span_deg < float(min_span_deg):
        raise PivotCalibrationError(
            f"姿态变化不足：最大相对旋转仅 {span_deg:.2f}°，至少需要 {min_span_deg:.1f}°"
        )
    blocks = []
    rhs = []
    for pose in poses:
        blocks.append(np.hstack((pose[:3, :3], -np.eye(3))))
        rhs.append(-pose[:3, 3])
    A = np.vstack(blocks)
    b = np.concatenate(rhs)
    solution, _, rank, singular = np.linalg.lstsq(A, b, rcond=None)
    if int(rank) < 6:
        raise PivotCalibrationError(f"求解矩阵秩不足：rank={int(rank)}，需要 6")
    if singular.size == 0 or singular[-1] <= 1e-12:
        condition = float("inf")
    else:
        condition = float(singular[0] / singular[-1])
    tcp_flange = solution[:3]
    pivot_base = solution[3:]
    residuals = np.asarray(
        [pose[:3, :3] @ tcp_flange + pose[:3, 3] - pivot_base for pose in poses],
        dtype=float,
    )
    norms = np.linalg.norm(residuals, axis=1)
    return PivotFitResult(
        tcp_flange_m=tcp_flange,
        pivot_base_m=pivot_base,
        residuals_m=norms,
        rms_residual_m=float(np.sqrt(np.mean(norms**2))),
        max_residual_m=float(np.max(norms)),
        matrix_rank=int(rank),
        condition_number=condition,
        orientation_span_deg=float(span_deg),
        pose_count=len(poses),
    )


def verify_pivot_sample(flange_pose: np.ndarray, result: PivotFitResult) -> float:
    """Return independent sample error in metres using a new flange pose."""
    pose = _as_transform(flange_pose, 0)
    predicted = pose[:3, :3] @ result.tcp_flange_m + pose[:3, 3]
    return float(np.linalg.norm(predicted - result.pivot_base_m))
