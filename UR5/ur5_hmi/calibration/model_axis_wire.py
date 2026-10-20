"""Read-only model-reference -> applied TCP wire, in STL_SOURCE metres.

This is a model-defined straight cylinder, not a measured nozzle axis.  It
does not use a display marker radius, external endpoint file or RTDE writes.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

WIRE_STATUS = "MODEL_AXIS_TO_TCP_ACTIVE"


def build_model_axis_wire(root: Path, manifest: dict, *, preview_only: bool = False) -> dict:
    if not isinstance(manifest, dict):
        raise ValueError("无效焊枪 manifest")
    status = str(manifest.get("status", "")).upper()
    if status != "ACTIVE" and not (preview_only and status == "STAGED_ALIGNMENT_REQUIRED"):
        raise ValueError("焊枪模型不是 ACTIVE")
    wire = manifest.get("wire", {})
    if not isinstance(wire, dict):
        raise ValueError("缺少模型枪口参考点记录")
    # Reuse the recorded CAD reference, not a new nearest-hull estimate.
    start = np.asarray(wire.get("gun_tip_model", wire.get("tip_source", [])), dtype=float)
    if start.shape != (3,) or not np.isfinite(start).all():
        raise ValueError("缺少有效的模型枪口参考点 gun_tip_model（STL_SOURCE m）")
    T = np.asarray(manifest.get("visual", {}).get("T_flange_source", []), dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-6)):
        raise ValueError("无效的 flange <- STL_SOURCE 安装变换")
    # The controller mirror is the only REAL planning source. Never fall
    # back to a historical four-point TCP while waiting for controller sync.
    path = Path(root) / "real_tcp_calibration_active.json"
    raw = path.read_bytes()
    tcp = json.loads(raw)
    if (not tcp.get("valid") or not tcp.get("applied_to_planning_model")
            or tcp.get("coordinate_frames", {}).get("flange") != "URDF flange"):
        raise ValueError("真实 TCP 未应用或不是 URDF flange 坐标系")
    point = np.asarray(tcp.get("position_flange_tcp_m", []), dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("真实 TCP 位置无效")
    end = (np.linalg.inv(T) @ np.r_[point, 1.0])[:3]
    delta = end - start
    length = float(np.linalg.norm(delta))
    diameter = 0.0012
    if not np.isfinite(length) or length <= 0:
        raise ValueError("MODEL_DEFINED_WIRE_INVALID: wire length must be > 0")
    axis = delta / length
    if diameter <= 0 or not np.isclose(np.linalg.norm(axis), 1, atol=1e-6):
        raise ValueError("MODEL_DEFINED_WIRE_INVALID: diameter/axis invalid")
    return {
        "status": "MODEL_AXIS_TO_TCP_PREVIEW" if preview_only else WIRE_STATUS,
        "preview_only": bool(preview_only), "definition": "MODEL_AXIS_TO_TCP",
        "coordinate_frame": "STL_SOURCE", "gun_tip_model": start.tolist(),
        "tip_source": start.tolist(), "tcp_source": end.tolist(),
        "wire_end_source": end.tolist(), "axis_source": axis.tolist(),
        "length_m": length, "diameter_m": diameter,
        "tip_definition": wire.get("tip_definition", "Existing model-defined CAD reference"),
        "axis_definition": "normalize(TCP - gun_tip_model); model-defined, not measured nozzle axis",
        "tcp_source_reference": str(path),
        "tcp_calibration_sha256": hashlib.sha256(raw).hexdigest(),
        "tcp_calibration_version": tcp.get("version"),
        "gun_source_sha256": manifest.get("source", {}).get("sha256"),
        "physical_measurement_required": False,
        "contact_tip_calibration_required": False,
        "collision_model": "fixed cylinder child of WeldingGunMount; model reference to applied TCP",
    }
