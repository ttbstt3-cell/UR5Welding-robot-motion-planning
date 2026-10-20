"""Pure frame conversion and validation for a position-only UR TCP result.

The URDF attachment frame ``flange`` is not the controller's all-zero TCP
frame.  The latter is represented by the URDF ``tool0`` frame.  Every helper
here operates on supplied data only; none connects to or writes to a robot.
"""
from __future__ import annotations

import math
import re

import numpy as np

from calibration.tcp_pivot import fit_pivot_calibration, verify_pivot_sample
from safety.joint_limits import JOINT_NAMES


def _vector(value, size: int, label: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite {size}-vector") from exc
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must be a finite {size}-vector")
    return vector.copy()


def _transform(value, label: str) -> np.ndarray:
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f"{label} has an invalid homogeneous row")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5)
            or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-5)):
        raise ValueError(f"{label} must contain a proper rotation")
    return transform


def _model_frames(model, q) -> dict:
    if list(getattr(model, "revolute_names", [])) != list(JOINT_NAMES):
        raise ValueError("URDF model joint order does not match the six UR joints")
    return model.link_transforms_in_base(_vector(q, 6, "q"))


def flange_offset_to_controller_tcp(model, offset_flange_m, current_tcp6) -> np.ndarray:
    """Convert URDF flange XYZ to controller/tool0 XYZ, preserving RX/RY/RZ.

    ``current_tcp6`` is the current controller TCP offset, in metres and a
    rotation vector in radians, read from one fresh RTDE sample.  Its rotation
    is copied unchanged because fixed-point calibration solves only position.
    """
    offset = _vector(offset_flange_m, 3, "flange TCP position")
    current = _vector(current_tcp6, 6, "controller TCP offset")
    frames = _model_frames(model, np.zeros(6))
    if "flange" not in frames or "tool0" not in frames:
        raise ValueError("URDF must provide both flange and tool0 frames")
    T_base_flange = _transform(frames["flange"], "base->flange")
    T_base_tool0 = _transform(frames["tool0"], "base->tool0")
    T_flange_tool0 = np.linalg.inv(T_base_flange) @ T_base_tool0
    position_tool0 = (np.linalg.inv(T_flange_tool0) @ np.r_[offset, 1.0])[:3]
    return np.r_[position_tool0, current[3:]]


def ur_pose_to_transform(pose6) -> np.ndarray:
    """Convert UR XYZ + rotation-vector (m/rad) to a homogeneous transform."""
    pose = _vector(pose6, 6, "UR pose")
    vector = pose[3:]
    angle = float(np.linalg.norm(vector))
    skew = np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]], dtype=float)
    if angle < 1e-8:
        a = 1.0 - angle * angle / 6.0
        b = 0.5 - angle * angle / 24.0
    else:
        a = math.sin(angle) / angle
        b = (1.0 - math.cos(angle)) / (angle * angle)
    transform = np.eye(4)
    transform[:3, :3] = np.eye(3) + a * skew + b * (skew @ skew)
    transform[:3, 3] = pose[:3]
    return transform


def transform_to_ur_pose(transform) -> np.ndarray:
    """Convert a homogeneous transform to UR XYZ + rotation-vector form.

    The conversion is deliberately local and deterministic so the HMI can
    persist the controller's complete active TCP without relying on a TCP
    topic that already contains the offset.  The returned translation is in
    metres and the rotation vector is in radians.
    """
    T = _transform(transform, "homogeneous transform")
    R = T[:3, :3]
    cosine = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(math.acos(cosine))
    if angle < 1e-9:
        # First order approximation is numerically stable near identity.
        vector = 0.5 * np.array([R[2, 1] - R[1, 2],
                                  R[0, 2] - R[2, 0],
                                  R[1, 0] - R[0, 1]], dtype=float)
    elif abs(math.pi - angle) < 1e-6:
        # Near pi the skew denominator is ill-conditioned.  Recover the
        # eigen-axis from the diagonal and fix its signs from the off-diagonal
        # terms; the resulting vector is equivalent for UR's rotvec format.
        diagonal = np.maximum(0.0, (np.diag(R) + 1.0) * 0.5)
        axis = np.sqrt(diagonal)
        idx = int(np.argmax(axis))
        if axis[idx] < 1e-8:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            if idx == 0:
                axis[1] = math.copysign(axis[1], R[0, 1] + R[1, 0])
                axis[2] = math.copysign(axis[2], R[0, 2] + R[2, 0])
            elif idx == 1:
                axis[0] = math.copysign(axis[0], R[0, 1] + R[1, 0])
                axis[2] = math.copysign(axis[2], R[1, 2] + R[2, 1])
            else:
                axis[0] = math.copysign(axis[0], R[0, 2] + R[2, 0])
                axis[1] = math.copysign(axis[1], R[1, 2] + R[2, 1])
            axis /= max(float(np.linalg.norm(axis)), 1e-12)
        vector = axis * angle
    else:
        vector = np.array([R[2, 1] - R[1, 2],
                           R[0, 2] - R[2, 0],
                           R[1, 0] - R[0, 1]], dtype=float)
        vector *= angle / (2.0 * math.sin(angle))
    return np.r_[T[:3, 3], vector]


def controller_snapshot_to_flange_tcp(model, snapshot: dict) -> dict:
    """Derive the controller's active 6D TCP in the calibrated URDF flange.

    ``actual_TCP_pose`` and ``actual_q`` are required from the same RTDE
    packet.  This avoids guessing how a raw ``tcp_offset`` rotation is
    oriented and avoids applying the tool0/flange transform twice.  The
    result is a complete ``T_flange_tcp`` plus UR pose fields, all in the
    HMI/MATLAB URDF frame.  It is read-only and never writes a controller.
    """
    if not isinstance(snapshot, dict):
        raise ValueError("RTDE snapshot must be a dictionary")
    if snapshot.get("frame_id", "base") != "base":
        raise ValueError("RTDE snapshot must use controller base frame")
    q = _vector(snapshot.get("actual_q", snapshot.get("q")), 6, "actual_q")
    actual_pose = _vector(snapshot.get("actual_TCP_pose"), 6, "actual_TCP_pose")
    frames = _model_frames(model, q)
    if "flange" not in frames:
        raise ValueError("URDF must provide flange")
    T_base_flange = _transform(frames["flange"], "FK base->flange")
    T_base_tcp = ur_pose_to_transform(actual_pose)
    T_flange_tcp = _transform(np.linalg.inv(T_base_flange) @ T_base_tcp,
                               "flange->controller TCP")
    pose = transform_to_ur_pose(T_flange_tcp)
    return {
        "q_rad": q.tolist(),
        "actual_TCP_pose_base": actual_pose.tolist(),
        "T_base_flange": T_base_flange.tolist(),
        "T_base_tcp": T_base_tcp.tolist(),
        "T_flange_tcp": T_flange_tcp.tolist(),
        "position_flange_tcp_m": pose[:3].tolist(),
        "rotation_flange_tcp_rotvec_rad": pose[3:].tolist(),
        "rotation_source": "controller actual_TCP_pose relative to calibrated URDF flange",
        "controller_offset": _vector(snapshot.get("tcp_offset"), 6, "tcp_offset").tolist(),
    }


def validate_rtde_model_alignment(
    model, snapshot: dict, *, max_position_error_mm: float = 3.0,
    max_orientation_error_deg: float = 0.5,
) -> dict:
    """Check calibrated FK against controller tool0 from one RTDE sample.

    Caller must obtain ``actual_q``, ``actual_TCP_pose`` and ``tcp_offset``
    together in the same RTDE data packet.  Independent ROS topic snapshots
    are deliberately not substituted here.  All transforms are in UR base.
    """
    if not isinstance(snapshot, dict):
        raise ValueError("RTDE snapshot must be a dictionary")
    if snapshot.get("frame_id", "base") != "base":
        raise ValueError("RTDE snapshot must use controller base frame")
    q = _vector(snapshot.get("actual_q", snapshot.get("q")), 6, "actual_q")
    actual_pose = _vector(snapshot.get("actual_TCP_pose"), 6, "actual_TCP_pose")
    tcp_offset = _vector(snapshot.get("tcp_offset"), 6, "tcp_offset")
    frames = _model_frames(model, q)
    if "tool0" not in frames:
        raise ValueError("URDF must provide tool0")
    T_fk = _transform(frames["tool0"], "FK base->tool0")
    T_measured = ur_pose_to_transform(actual_pose) @ np.linalg.inv(ur_pose_to_transform(tcp_offset))
    position_error_mm = float(np.linalg.norm(T_fk[:3, 3] - T_measured[:3, 3]) * 1000)
    relative_rotation = T_fk[:3, :3].T @ T_measured[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1))
    orientation_error_deg = math.degrees(math.acos(cosine))
    return {
        "pass": bool(position_error_mm <= max_position_error_mm
                     and orientation_error_deg <= max_orientation_error_deg),
        "position_error_mm": position_error_mm,
        "orientation_error_deg": orientation_error_deg,
        "max_position_error_mm": float(max_position_error_mm),
        "max_orientation_error_deg": float(max_orientation_error_deg),
        "frame_id": "base",
        "fk_tool0_position_m": T_fk[:3, 3].tolist(),
        "controller_tool0_position_m": T_measured[:3, 3].tolist(),
    }


def calibration_write_quality(flange_poses, result=None) -> dict:
    """Estimate whether pose coverage supports writing the TCP position.

    Standard deviations use an isotropic, independent contact/FK residual
    model with covariance ``sigma² (A.T A)^-1`` and ``3*N-6`` degrees of
    freedom.  They are conditioning diagnostics, not an absolute-accuracy
    guarantee; common systematic errors can have small residuals.
    """
    poses = [_transform(pose, "sample base->flange") for pose in flange_poses]
    if result is None:
        result = fit_pivot_calibration(poses)
    A = np.vstack([np.hstack((pose[:3, :3], -np.eye(3))) for pose in poses])
    rank = int(np.linalg.matrix_rank(A))
    condition = float(np.linalg.cond(A))
    std_xyz_mm = np.full(3, float("inf"))
    if rank == 6 and len(poses) >= 4:
        residuals = np.array([
            pose[:3, :3] @ result.tcp_flange_m + pose[:3, 3] - result.pivot_base_m
            for pose in poses
        ])
        variance = float(np.sum(residuals ** 2) / (3 * len(poses) - 6))
        covariance = variance * np.linalg.inv(A.T @ A)
        std_xyz_mm = np.sqrt(np.maximum(0.0, np.diag(covariance)[:3])) * 1000
    max_std_mm = float(np.max(std_xyz_mm))
    checks = {
        "rank": rank == 6,
        "condition": math.isfinite(condition) and condition <= 100.0,
        "residual": result.max_residual_mm <= 2.0,
        "estimated_uncertainty": math.isfinite(max_std_mm) and max_std_mm <= 2.0,
    }
    reasons = []
    if not checks["rank"]:
        reasons.append("pose set does not constrain all three TCP coordinates")
    if not checks["condition"]:
        reasons.append("pose orientations are too poorly conditioned (condition >100)")
    if not checks["residual"]:
        reasons.append("maximum fixed-point residual exceeds 2 mm")
    if not checks["estimated_uncertainty"]:
        reasons.append("estimated position standard deviation exceeds 2 mm; add diverse poses")
    return {
        "pass": bool(all(checks.values())),
        "std_xyz_mm": std_xyz_mm.tolist(),
        "max_std_mm": max_std_mm,
        "condition_number": condition,
        "matrix_rank": rank,
        "max_residual_mm": result.max_residual_mm,
        "checks": checks,
        "reason": "; ".join(reasons) if reasons else "pose coverage and residual checks passed",
        "estimate_method": "isotropic residual covariance; not an absolute-accuracy guarantee",
    }


def validate_saved_pivot_payload(payload: dict, model, expected_urdf_sha256: str) -> dict:
    """Rebuild a saved fit under the current model and check its quality.

    A saved ``valid`` flag is not sufficient: the model fingerprint, sample
    dimensions, reproduced offset, fit rank/conditioning/residuals and (when
    present) saved independent errors are checked.  Independent verification
    is optional for the ordinary four-point draft.  Legacy files record only
    validation residuals, so the return value states whether their validation
    poses can be re-evaluated; it does not claim those missing poses were
    reconstructed.
    """
    if not isinstance(payload, dict) or payload.get("valid") is not True:
        raise ValueError("saved TCP calibration is not marked valid")
    if (not isinstance(expected_urdf_sha256, str)
            or re.fullmatch(r"[0-9a-fA-F]{64}", expected_urdf_sha256) is None):
        raise ValueError("current calibrated URDF fingerprint is required")
    if payload.get("urdf_sha256") != expected_urdf_sha256:
        raise ValueError("saved TCP calibration uses a different calibrated URDF")
    frames = payload.get("coordinate_frames", {})
    if (not isinstance(frames, dict)
            or frames.get("base") not in ("URDF base", "base")
            or frames.get("flange") not in ("URDF flange", "flange")):
        raise ValueError("saved calibration frame must be URDF base / flange")
    offset = _vector(payload.get("position_flange_tcp_m"), 3, "saved flange TCP position")
    samples = payload.get("pose_samples")
    if not isinstance(samples, list) or not 4 <= len(samples) <= 6:
        raise ValueError("saved calibration must contain 4 to 6 joint samples")
    poses = []
    fit_q = []
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError("saved joint sample must be a dictionary")
        q = _vector(sample.get("q_rad"), 6, "saved calibration q_rad")
        fit_q.append(q)
        model_frames = _model_frames(model, q)
        if "flange" not in model_frames:
            raise ValueError("URDF must provide flange")
        poses.append(_transform(model_frames["flange"], "sample base->flange"))
    result = fit_pivot_calibration(poses)
    difference_mm = float(np.linalg.norm(offset - result.tcp_flange_m) * 1000)
    if difference_mm > 0.01:
        raise ValueError(f"saved TCP fit cannot be reproduced: {difference_mm:.6f} mm difference")
    if (not result.valid or result.matrix_rank != 6
            or not math.isfinite(result.condition_number) or result.condition_number > 100.0):
        raise ValueError("TCP fit failed rank or conditioning checks")
    fit = payload.get("fit")
    if not isinstance(fit, dict) or fit.get("valid") is not True:
        raise ValueError("saved fit quality metadata is missing or invalid")
    fit_numbers = {}
    for field in ("condition_number", "rms_residual_mm", "max_residual_mm", "orientation_span_deg"):
        try:
            value = float(fit[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"saved fit {field} must be finite") from exc
        if not math.isfinite(value):
            raise ValueError(f"saved fit {field} must be finite")
        fit_numbers[field] = value
    if (fit.get("matrix_rank") != 6 or not 0 < fit_numbers["condition_number"] <= 100
            or fit_numbers["rms_residual_mm"] < 0
            or fit_numbers["max_residual_mm"] < 0):
        raise ValueError("saved fit quality metadata failed checks")
    # Independent verification is deliberately optional for the ordinary
    # four-point workflow.  A saved draft may contain no verification samples
    # yet; when samples are present, every error must still be finite and
    # non-negative and (if joint samples are stored) reproducible.
    try:
        raw_independent = payload.get("independent_verification_mm", [])
        independent = np.asarray([] if raw_independent is None else raw_independent,
                                 dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("independent TCP verification errors must be finite") from exc
    if (independent.ndim != 1 or not np.all(np.isfinite(independent))
            or np.any(independent < 0)):
        raise ValueError("independent TCP verification errors must be finite and nonnegative")
    independent_recomputed = False
    if "verification_samples" in payload and independent.size:
        verification_samples = payload["verification_samples"]
        if (not isinstance(verification_samples, list) or not verification_samples
                or len(verification_samples) != independent.size):
            raise ValueError("verification joint samples must match saved independent error count")
    elif "verification_samples" in payload and payload["verification_samples"]:
        raise ValueError("verification samples cannot exist without independent errors")
    if independent.size and "verification_samples" in payload:
        verification_samples = payload["verification_samples"]
        recomputed_errors = []
        for sample in verification_samples:
            if not isinstance(sample, dict):
                raise ValueError("verification joint sample must be a dictionary")
            q = _vector(sample.get("q_rad"), 6, "verification q_rad")
            model_frames = _model_frames(model, q)
            if "flange" not in model_frames:
                raise ValueError("URDF must provide flange")
            pose = _transform(model_frames["flange"], "verification base->flange")
            if (any(np.allclose(q, fitted_q, atol=1e-6, rtol=0) for fitted_q in fit_q)
                    or any(np.allclose(pose, fitted_pose, atol=1e-6, rtol=0) for fitted_pose in poses)):
                raise ValueError("independent verification pose duplicates a fitting pose")
            recomputed_errors.append(verify_pivot_sample(pose, result) * 1000)
        recomputed = np.asarray(recomputed_errors, dtype=float)
        if not np.allclose(recomputed, independent, atol=0.01, rtol=0):
            raise ValueError("saved independent errors cannot be reproduced from verification joints")
        if not np.all(np.isfinite(recomputed)) or np.any(recomputed < 0):
            raise ValueError("recomputed independent verification is invalid")
        independent = recomputed
        independent_recomputed = True
    return {
        "valid": True,
        "position_flange_tcp_m": result.tcp_flange_m.tolist(),
        "offset_reproduction_error_mm": difference_mm,
        "fit": result.to_dict(),
        "independent_verification_max_mm": float(independent.max()) if independent.size else None,
        "independent_verification_mm": independent.tolist(),
        "independent_verification_recomputed": independent_recomputed,
        "write_quality": calibration_write_quality(poses, result),
        "urdf_sha256": expected_urdf_sha256,
    }
