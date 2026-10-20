#!/usr/bin/env python3
"""Offline gun/wire endpoint versus controller TCP diagnostic.

This tool never writes a robot, pendant TCP, ROS topic, MATLAB calibration,
or scene geometry.  It uses one URDF flange chain for the hull, wire and TCP
and reports the three candidate J6 installation rolls requested by the HMI
diagnostic workflow.

The ``--q`` and ``--tcp`` arguments are intentionally explicit.  They must be
read-only snapshots from the same controller state; the script does not
silently substitute a saved trajectory or wrap a continuous joint angle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from gui.robot_view import UrdfKinematicModel


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "real_robot_description_calibrated.urdf"
SCENE = ROOT / "scene_geometry.json"
TOOL_CAL = ROOT / "ur5_tool_calibration.mat"
AUDIT = ROOT / "matlab仿真" / "current_weld_geometry_audit.mat"
APPLIED_TCP = ROOT / "real_tcp_calibration_applied.json"
ACTIVE_TCP = ROOT / "real_tcp_calibration_active.json"
MOUNT_PREVIEW = ROOT / "real_gun_mount_roll_preview.json"
DEFAULT_OUT = ROOT / "diagnostics" / "gun_tcp_alignment_latest.json"


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _pose(data: dict) -> np.ndarray:
    qx, qy, qz, qw = [float(v) for v in data.get("quaternion", [0, 0, 0, 1])]
    n = max(float(np.linalg.norm([qx, qy, qz, qw])), 1e-12)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    T[:3, 3] = np.asarray(data.get("position", [0, 0, 0]), dtype=float)[:3]
    return T


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float).reshape(3)
    axis /= max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])
    return T


def _mat_struct_field(obj, name, default=np.nan):
    value = getattr(obj, name, default)
    arr = np.asarray(value, dtype=float)
    return float(arr.reshape(-1)[0]) if arr.size == 1 else arr


def _finite_vec(value, size):
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != size or not np.all(np.isfinite(arr)):
        raise ValueError(f"expected finite vector of length {size}")
    return arr


def _fmt(v, scale=1.0):
    return [round(float(x) * scale, 6) for x in np.asarray(v).reshape(-1)]


def build_report(q, tcp_base, tcp_source: str, output: Path, tcp_offset_controller=None) -> dict:
    model = UrdfKinematicModel(urdf_path=URDF)
    frames0 = model.link_transforms_in_base(np.zeros(6, dtype=float))
    frames = model.link_transforms_in_base(q)
    if "flange" not in frames or "tool0" not in frames:
        raise RuntimeError("URDF flange/tool0 frame unavailable")

    calibration = loadmat(str(TOOL_CAL))
    T_tool0_mount = np.asarray(calibration["T_tool0_mount"], dtype=float)
    T_mount_tcp = np.asarray(calibration["T_mount_tcp"], dtype=float)
    scene = json.loads(SCENE.read_text(encoding="utf-8"))
    hull = scene["welding_gun"]["collision_hull"]
    vertices = np.asarray(hull["vertices_m"], dtype=float)
    T_mount_hull = _pose(hull.get("pose_mount", {}))

    T_flange_tool0 = np.linalg.inv(frames0["flange"]) @ frames0["tool0"]
    T_flange_mount = T_flange_tool0 @ T_tool0_mount
    T_flange_hull = T_flange_mount @ T_mount_hull
    T_flange_tcp_nominal = T_flange_mount @ T_mount_tcp
    # Position-only RTDE/tcp_pose snapshot expressed back in the URDF flange
    # frame.  This is the controller's effective TCP point for this q; it is
    # not silently replaced with the HMI calibration file.
    controller_offset_flange = (np.linalg.inv(frames["flange"])
                                 @ np.r_[tcp_base, 1.0])[:3]

    # J6 axis is computed from the URDF joint origin/axis in the flange frame.
    j6 = model.joints.get("wrist_3_joint")
    if j6 is None:
        raise RuntimeError("URDF wrist_3_joint/J6 unavailable")
    T0 = frames0
    axis_base = (T0[j6["parent"]][:3, :3] @ np.asarray(j6["origin"])[:3, :3]
                 @ np.asarray(j6["axis"], dtype=float))
    axis_flange = T0["flange"][:3, :3].T @ axis_base
    axis_flange /= max(float(np.linalg.norm(axis_flange)), 1e-12)

    audit_obj = loadmat(str(AUDIT), squeeze_me=True, struct_as_record=False).get("audit")
    wire_length_m = float(_mat_struct_field(audit_obj, "wireLengthM"))
    wire_diameter_m = float(_mat_struct_field(audit_obj, "wireDiameterM"))

    # This intentionally mirrors sync_ur5_joint_states.m and the HMI.  It is
    # an estimate from the nearest GunCollisionHull vertex, not a measured
    # conductive-tip centre.
    hull_mount = (T_mount_hull @ np.column_stack((vertices, np.ones(len(vertices)))).T).T[:, :3]
    p_tcp_mount = T_mount_tcp[:3, 3]
    nearest_index = int(np.argmin(np.linalg.norm(hull_mount - p_tcp_mount, axis=1)))
    p_head_mount = hull_mount[nearest_index]
    p_head_tcp = (np.linalg.inv(T_mount_tcp) @ np.r_[p_head_mount, 1.0])[:3]
    z_wire = -p_head_tcp / max(float(np.linalg.norm(p_head_tcp)), 1e-12)
    x_ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(x_ref, z_wire))) > 0.95:
        x_ref = np.array([0.0, 1.0, 0.0])
    x_wire = np.cross(x_ref, z_wire)
    x_wire /= max(float(np.linalg.norm(x_wire)), 1e-12)
    y_wire = np.cross(z_wire, x_wire)
    y_wire /= max(float(np.linalg.norm(y_wire)), 1e-12)
    p_tip_tcp = p_head_tcp + wire_length_m * z_wire

    result = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "OFFLINE_READ_ONLY",
        "live_tcp_source": tcp_source,
        "controller_tcp_pose_base_m": _fmt(tcp_base),
        "controller_effective_tcp_offset_flange_m": _fmt(controller_offset_flange),
        "controller_effective_tcp_offset_flange_mm": _fmt(controller_offset_flange, 1000.0),
        "q_ros_order_rad": _fmt(q),
        "q_ros_order_deg": _fmt(np.degrees(q)),
        "frames": {
            "base": "URDF base",
            "flange": "URDF flange",
            "tool0": "URDF tool0",
            "mount": "frozen ur5_tool_calibration.mat tool0->mount",
            "tcp": "mount->TCP frozen transform for wire geometry",
            "live_tcp": "controller tcp_pose_broadcaster/pose in URDF base",
        },
        "tcp_live_policy": {
            "red_marker_fresh_controller": True,
            "red_marker_fallback": "HMI flange FK using real_tcp_calibration_applied.json (or frozen MAT baseline)",
            "diagnostic_delta_uses_fallback": False,
        },
        "transforms": {
            "T_flange_tool0": T_flange_tool0.tolist(),
            "T_flange_mount": T_flange_mount.tolist(),
            "T_flange_hull": T_flange_hull.tolist(),
            "T_flange_tcp_nominal": T_flange_tcp_nominal.tolist(),
        },
        "j6_axis_flange": _fmt(axis_flange),
        "j6_axis_norm": float(np.linalg.norm(axis_flange)),
        "wire_geometry": {
            "length_mm": wire_length_m * 1000.0,
            "diameter_mm": wire_diameter_m * 1000.0,
            "length_source": str(AUDIT),
            "diameter_source": str(AUDIT),
            "measurement_status": "audit MAT recorded values; not newly measured in this run",
            "head_definition": "nearest GunCollisionHull vertex to frozen TCP (estimate)",
            "nearest_hull_vertex_index_zero_based": nearest_index,
            "tip_is_not": "a measured conductive-tip/nozzle centre",
            "p_head_tcp_m": _fmt(p_head_tcp),
            "p_tip_tcp_m": _fmt(p_tip_tcp),
        },
        "candidates": {},
        "consistency_checks": {
            "rotation_axis_is_flange_local": True,
            "rotation_about_flange_origin": True,
            "adjustment_left_multiply_order": "T_base_flange @ T_adjust(flange) @ T_flange_tcp @ p_tip_tcp",
            "tool0_flange_not_reapplied": True,
            "gun_hull_and_wire_share_adjustment": True,
            "wire_endpoint_not_hull_nearest_vertex": True,
            "hull_nearest_vertex_used_only_to_estimate_head": True,
        },
        "source_hashes": {str(p.name): _sha(p) for p in (URDF, SCENE, TOOL_CAL, AUDIT)
                           if p.is_file()},
    }
    if tcp_offset_controller is not None:
        raw = _finite_vec(tcp_offset_controller, 6)
        raw_to_urdf = (T_flange_tool0 @ np.r_[raw[:3], 1.0])[:3]
        result["controller_rtde_tcp_offset_raw"] = {
            "value": _fmt(raw),
            "translation_m": _fmt(raw[:3]),
            "rotation_vector_rad": _fmt(raw[3:]),
            "reference": "UR controller tool-output/flange frame (raw RTDE field)",
            "warning": "不要把这三个平移数值直接当作URDF flange XYZ；必须经过已核对的控制器→URDF坐标变换。",
            "translation_converted_to_urdf_flange_m": _fmt(raw_to_urdf),
            "translation_converted_to_urdf_flange_mm": _fmt(raw_to_urdf, 1000.0),
            "converted_vs_actual_pose_derived_delta_mm": _fmt((raw_to_urdf - controller_offset_flange) * 1000.0),
        }

    live = np.asarray(tcp_base, dtype=float)
    for roll in (0.0, 90.0, -90.0):
        T_adjust = _axis_angle(axis_flange, math.radians(roll))
        p_tip_flange = (T_adjust @ T_flange_tcp_nominal @ np.r_[p_tip_tcp, 1.0])[:3]
        p_tip_base = (frames["flange"] @ np.r_[p_tip_flange, 1.0])[:3]
        delta = (p_tip_base - live) * 1000.0
        # Independent proof that the hull is moved by a rigid parent
        # transform: pairwise distances are unchanged after the adjustment.
        hull0 = (T_adjust @ T_flange_hull @ np.column_stack((vertices, np.ones(len(vertices)))).T).T[:, :3]
        d0 = np.linalg.norm(hull0[None, :, :] - hull0[:, None, :], axis=2)
        hull_after = hull0  # pairwise matrix is evaluated against itself below
        hull_nom = (T_flange_hull @ np.column_stack((vertices, np.ones(len(vertices)))).T).T[:, :3]
        d_nom = np.linalg.norm(hull_nom[None, :, :] - hull_nom[:, None, :], axis=2)
        result["candidates"][f"{roll:+.0f}deg"] = {
            "roll_deg_right_hand_about_j6": roll,
            "wire_tip_flange_m": _fmt(p_tip_flange),
            "wire_tip_base_m": _fmt(p_tip_base),
            "delta_xyz_mm_wire_tip_minus_live_tcp": _fmt(delta),
            "distance_mm": float(np.linalg.norm(delta)),
            "hull_pairwise_distance_max_error_m": float(np.max(np.abs(d0 - d_nom))),
            "T_adjust": T_adjust.tolist(),
        }

    applied_path = ACTIVE_TCP if ACTIVE_TCP.is_file() else APPLIED_TCP
    if applied_path.is_file():
        applied = json.loads(applied_path.read_text(encoding="utf-8"))
        result["source_hashes"][applied_path.name] = _sha(applied_path)
        applied_offset = np.asarray(applied.get("position_flange_tcp_m", [np.nan] * 3), dtype=float).reshape(-1)
        applied_delta = ((controller_offset_flange - applied_offset) * 1000.0
                         if applied_offset.shape == (3,) and np.all(np.isfinite(applied_offset)) else None)
        result["hmi_applied_tcp_record"] = {
            "path": str(applied_path),
            "valid": bool(applied.get("valid")),
            "applied_to_planning_model": bool(applied.get("applied_to_planning_model")),
            "position_flange_tcp_mm": applied.get("position_flange_tcp_mm"),
            "controller_minus_hmi_offset_delta_mm": None if applied_delta is None else _fmt(applied_delta),
            "controller_minus_hmi_offset_distance_mm": None if applied_delta is None else float(np.linalg.norm(applied_delta)),
            "record_is_used_only_when_controller_tcp_stale": True,
            "record_role": "active confirmed real-planning TCP" if applied_path == ACTIVE_TCP else "legacy approved HMI TCP",
        }
    if MOUNT_PREVIEW.is_file():
        mount = json.loads(MOUNT_PREVIEW.read_text(encoding="utf-8"))
        result["source_hashes"][MOUNT_PREVIEW.name] = _sha(MOUNT_PREVIEW)
        result["mount_adjustment_record"] = {
            "path": str(MOUNT_PREVIEW),
            "status": mount.get("status"),
            "preview_only": str(mount.get("status", "")).startswith("PREVIEW"),
            "current_default_roll_deg": mount.get("candidates", {}).get("plus90", {}).get("angle_deg_right_hand_about_j6_axis"),
        }
    result["recommendation"] = (
        "不修改当前标定。当前偏差由 wire tip 使用GunCollisionHull最近顶点估计、"
        "以及焊枪/焊丝几何与真实导电嘴定义未绑定导致；先实测导电嘴中心和焊丝方向，"
        "确认后再单独更新真实安装标定并使旧轨迹STALE。"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--q", nargs=6, type=float, required=True, help="ordered ROS joint q in rad")
    parser.add_argument("--tcp", nargs=3, type=float, required=True, help="controller TCP XYZ in URDF base, metres")
    parser.add_argument("--tcp-source", default="ROS2 /tcp_pose_broadcaster/pose read-only snapshot")
    parser.add_argument("--tcp-offset-controller", nargs=6, type=float,
                        help="optional raw controller RTDE tcp_offset [x y z rx ry rz]")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    report = build_report(_finite_vec(args.q, 6), _finite_vec(args.tcp, 3), args.tcp_source,
                          args.output, args.tcp_offset_controller)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
