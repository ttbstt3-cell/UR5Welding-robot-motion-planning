"""Local planning mirror of the controller's active TCP; no robot I/O.

Use tcp_offset, not a pose-dependent inverse-FK fit, for the fixed full 6D
transform. actual_q/actual_TCP_pose from the same packet independently check
the frame conversion. Parameter consistency is not a tool-tip accuracy test.
"""
from __future__ import annotations

import hashlib
import json
import os
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from calibration.tcp_frames import (
    _model_frames, _transform, _vector, transform_to_ur_pose,
    ur_pose_to_transform, validate_rtde_model_alignment,
)

AUTHORITY = "UR_CONTROLLER_ACTIVE_TCP"


def fixed_flange_tcp(model, offset):
    frames = _model_frames(model, np.zeros(6))
    flange_tool0 = np.linalg.inv(_transform(frames["flange"], "base->flange")) @ _transform(
        frames["tool0"], "base->tool0")
    return _transform(flange_tool0 @ ur_pose_to_transform(offset), "flange->TCP")


def make_controller_tcp_record(model, snapshot, urdf_sha256):
    alignment = validate_rtde_model_alignment(model, snapshot)
    if not alignment["pass"]:
        raise ValueError("URDF/控制器坐标核对 FAIL：%.3f mm / %.3f°" % (
            alignment["position_error_mm"], alignment["orientation_error_deg"]))
    offset = _vector(snapshot.get("tcp_offset"), 6, "tcp_offset")
    T = fixed_flange_tcp(model, offset)
    pose = transform_to_ur_pose(T)
    fingerprint = hashlib.sha256(json.dumps({
        "tcp_offset": offset.tolist(), "urdf_sha256": urdf_sha256,
        "conversion_version": 1,
    }, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return {
        "schema_version": 2, "calibration_type": "controller_active_tcp_mirror_6d",
        "tcp_authority": AUTHORITY, "source": "RTDE tcp_offset (controller currently active TCP)",
        "coordinate_frames": {"base": "URDF base", "flange": "URDF flange",
                              "controller_offset": "UR controller tool0", "tcp": "controller active TCP"},
        "controller_tcp_offset": offset.tolist(), "T_flange_tcp": T.tolist(),
        "position_flange_tcp_m": pose[:3].tolist(),
        "position_flange_tcp_mm": (pose[:3] * 1000).tolist(),
        "rotation_flange_tcp_rotvec_rad": pose[3:].tolist(),
        "rotation_source": "full controller rotvec via fixed URDF flange<-tool0 transform",
        "urdf_sha256": urdf_sha256, "version": "CONTROLLER-TCP-" + fingerprint,
        "status": "ACTIVE", "valid": True, "active_for_planning": True,
        "applied_to_planning_model": True, "sync_policy": "AUTO_CONTROLLER_AUTHORITATIVE",
        "quality_level": "CONTROLLER_DEFINED", "physical_accuracy_status": "NOT_MEASURED_BY_HMI",
        "quality_note": "Controller parameters are authoritative; frame consistency is not calibration accuracy.",
        "frame_alignment": alignment, "source_snapshot": snapshot,
        "requires_replan": True, "requires_collision_revalidation": True,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }


def validate_controller_tcp_record(record, model, urdf_sha256):
    """Validate stored source, URDF version and complete rigid conversion."""
    if record.get("tcp_authority") != AUTHORITY:
        raise ValueError("规划 TCP 不是控制器当前生效 TCP 的镜像")
    if (record.get("status") != "ACTIVE" or record.get("valid") is not True
            or record.get("active_for_planning") is not True
            or record.get("applied_to_planning_model") is not True):
        raise ValueError("控制器规划 TCP 镜像未激活")
    if record.get("urdf_sha256") != urdf_sha256:
        raise ValueError("TCP 镜像绑定的 URDF 已改变")
    frames = record.get("coordinate_frames", {})
    if (frames.get("flange") != "URDF flange" or frames.get("base") != "URDF base"
            or frames.get("controller_offset") != "UR controller tool0"):
        raise ValueError("TCP 镜像 flange/base/tool0 坐标声明不一致")
    offset = _vector(record.get("controller_tcp_offset"), 6, "saved tcp_offset")
    T = _transform(record.get("T_flange_tcp"), "saved flange->TCP")
    if not np.allclose(T, fixed_flange_tcp(model, offset), atol=1e-9, rtol=0):
        raise ValueError("TCP 镜像 flange/tool0 完整变换不一致")
    if not np.allclose(_vector(record.get("position_flange_tcp_m"), 3, "saved TCP position"),
                       T[:3, 3], atol=1e-9, rtol=0):
        raise ValueError("TCP 镜像位置字段不一致")
    sample = record.get("source_snapshot", {})
    if not np.allclose(_vector(sample.get("tcp_offset"), 6, "snapshot tcp_offset"),
                       offset, atol=1e-7, rtol=0):
        raise ValueError("TCP 镜像与源快照不一致")
    if not validate_rtde_model_alignment(model, sample)["pass"]:
        raise ValueError("TCP 镜像源快照坐标核对失败")
    return T


def _atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=path.parent, delete=False) as stream:
            temp = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if temp is not None and temp.exists():
            temp.unlink()


def sync_controller_tcp_record(root, model, snapshot, urdf_sha256):
    """Archive + atomically replace local ACTIVE file only when TCP changes.

    Caller enforces live freshness and defers during planning/motion. Repeated
    reads (or changing q with the same TCP) never churn dependency hashes.
    Historical four-point and frozen simulation files are never overwritten.
    """
    record = make_controller_tcp_record(model, snapshot, urdf_sha256)
    path = Path(root) / "real_tcp_calibration_active.json"
    old_raw = path.read_bytes() if path.is_file() else None
    if old_raw is not None:
        try:
            old = json.loads(old_raw)
            validate_controller_tcp_record(old, model, urdf_sha256)
            if np.allclose(old["controller_tcp_offset"], record["controller_tcp_offset"], atol=1e-7, rtol=0):
                return old, False
        except (ValueError, TypeError, KeyError):
            pass
    history = Path(root) / "real_tcp_calibration_versions"
    if old_raw is not None:
        _atomic_write(history / ("before_auto_sync_" + uuid.uuid4().hex + ".json"), old_raw)
    raw = (json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
    _atomic_write(history / (record["version"] + ".json"), raw)
    _atomic_write(path, raw)
    return record, True
