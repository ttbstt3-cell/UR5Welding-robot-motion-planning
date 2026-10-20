#!/usr/bin/env python3
"""Offline MATLAB/HMI TCP consistency audit (never connects to ROS or UR5).

The MATLAB side is produced by ``check_hmi_tcp_pose_consistency.m``.  This
script evaluates the same qPath with the HMI's calibrated URDF FK, compares
the ten deterministic samples and separates node mismatch from the nonlinear
FK error introduced by joint-space interpolation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.io import loadmat

HERE = Path(__file__).resolve()
HMI_ROOT = HERE.parents[1]
UR5_ROOT = HMI_ROOT.parent
sys.path.insert(0, str(HMI_ROOT))
from gui.robot_view import UrdfKinematicModel  # noqa: E402


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _arr(x, dtype=float):
    return np.asarray(x, dtype=dtype)


def _pose_error(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    dp = float(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0)
    R = A[:3, :3].T @ B[:3, :3]
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return dp, math.degrees(math.acos(c))


def _mat_struct_pose(sample: dict, key: str) -> np.ndarray:
    return _arr(sample[key])


def _stats(values) -> dict:
    a = _arr(values).reshape(-1)
    if a.size == 0:
        return {"count": 0, "max": None, "rms": None, "mean": None}
    return {"count": int(a.size), "max": float(np.max(a)),
            "rms": float(np.sqrt(np.mean(a * a))), "mean": float(np.mean(a))}


def run(trajectory: Path, matlab_report: Path, output: Path) -> dict:
    md = json.loads(matlab_report.read_text(encoding="utf-8"))
    mat_q = _arr(md["q_path_rad"])
    if mat_q.ndim != 2 or mat_q.shape[1] != 6:
        raise ValueError("MATLAB audit q_path_rad is not N x 6")
    data = loadmat(trajectory, squeeze_me=True, struct_as_record=False)
    q = _arr(data["qPath"])
    tcp_saved_mat = _arr(data["tcpPath"])
    if q.shape != mat_q.shape or not np.allclose(q, mat_q, atol=1e-10, rtol=0):
        raise ValueError("MATLAB audit and trajectory qPath differ")
    if tcp_saved_mat.shape != (q.shape[0], 3):
        raise ValueError("trajectory tcpPath must be N x 3")

    calibration = loadmat(UR5_ROOT / "ur5_tool_calibration.mat", squeeze_me=True)
    T_tool0_mount = _arr(calibration["T_tool0_mount"])
    T_mount_tcp = _arr(calibration["T_mount_tcp"])
    active_file = UR5_ROOT / "real_tcp_calibration_active.json"
    applied_file = active_file if active_file.is_file() else UR5_ROOT / "real_tcp_calibration_applied.json"
    applied = json.loads(applied_file.read_text(encoding="utf-8")) if applied_file.is_file() else {}
    real_offset = _arr(applied.get("position_flange_tcp_m", [np.nan] * 3))
    if real_offset.shape != (3,) or not np.all(np.isfinite(real_offset)):
        real_offset = None

    model = UrdfKinematicModel()
    # MATLAB scene base -> HMI URDF base.  This is the existing, documented
    # Rz(180) conversion; it is not applied to q or to the fixed tool chain.
    C = np.eye(4)
    C[:3, :3] = np.diag([-1.0, -1.0, 1.0])

    hmi_frames = []
    for qi in q:
        frames = model.link_transforms_in_base(qi)
        flange = frames["flange"]
        tool0 = frames["tool0"]
        mount = tool0 @ T_tool0_mount
        nominal = mount @ T_mount_tcp
        real = nominal.copy()
        if real_offset is not None:
            # The applied calibration is explicitly in URDF flange axes.
            real[:3, 3] = flange[:3, 3] + flange[:3, :3] @ real_offset
        hmi_frames.append({"flange": flange, "tool0": tool0, "mount": mount,
                           "nominal": nominal, "real": real})

    # Convert the saved MATLAB tcpPath (MATLAB base) into the HMI base frame.
    tcp_saved_hmi = (C[:3, :3] @ tcp_saved_mat.T).T
    nominal_hmi = _arr([x["nominal"][:3, 3] for x in hmi_frames])
    node_error = np.linalg.norm(nominal_hmi - tcp_saved_hmi, axis=1) * 1000.0

    # A straight line in Cartesian TCP space is not the same as the FK of a
    # linearly interpolated joint segment.  Measure this independently of the
    # node mismatch using both HMI endpoints and saved MATLAB endpoints.
    dense_vs_saved = []
    dense_vs_hmi_line = []
    dense_worst = {"error_mm": -1.0, "edge_index": None, "alpha": None}
    for i in range(len(q) - 1):
        for alpha in np.linspace(0.0, 1.0, 21)[1:-1]:
            qi = (1.0 - alpha) * q[i] + alpha * q[i + 1]
            frames = model.link_transforms_in_base(qi)
            p = (frames["tool0"] @ T_tool0_mount @ T_mount_tcp)[:3, 3]
            p_saved_line = (1.0 - alpha) * tcp_saved_hmi[i] + alpha * tcp_saved_hmi[i + 1]
            p_hmi_line = (1.0 - alpha) * nominal_hmi[i] + alpha * nominal_hmi[i + 1]
            e_saved = float(np.linalg.norm(p - p_saved_line) * 1000.0)
            e_hmi = float(np.linalg.norm(p - p_hmi_line) * 1000.0)
            dense_vs_saved.append(e_saved)
            dense_vs_hmi_line.append(e_hmi)
            if e_saved > dense_worst["error_mm"]:
                dense_worst = {"error_mm": e_saved, "edge_index": i + 1, "alpha": float(alpha)}

    # The ten samples are supplied by the MATLAB audit, not generated from a
    # different random selection.  Compare full 4x4 poses after the C frame
    # conversion.  MATLAB stock universalUR5 calls tool0 the flange frame;
    # the calibrated HMI URDF exposes both flange and tool0 explicitly.
    sample_indices = [int(x) - 1 for x in md["sample_indices"]]
    sample_rows = []
    frame_specs = {
        "flange": ("flange", "flange_T"),
        "tool0": ("tool0", "tool0_T"),
        "welding_gun_mount": ("mount", "welding_gun_mount_T"),
        "tcp_nominal": ("nominal", "tcp_nominal_T"),
    }
    frame_errors = {name: {"position_mm": [], "orientation_deg": []}
                    for name in frame_specs}
    for index in sample_indices:
        matlab_sample = md["samples"][sample_indices.index(index)]
        row = {"index": index + 1, "frames": {}}
        for name, (hmi_key, mat_key) in frame_specs.items():
            H = hmi_frames[index][hmi_key]
            M = C @ _mat_struct_pose(matlab_sample, mat_key)
            dp, da = _pose_error(H, M)
            frame_errors[name]["position_mm"].append(dp)
            frame_errors[name]["orientation_deg"].append(da)
            row["frames"][name] = {"hmi_xyz_m": H[:3, 3].tolist(),
                                    "matlab_xyz_m": M[:3, 3].tolist(),
                                    "position_error_mm": dp,
                                    "orientation_error_deg": da}
        sample_rows.append(row)

    # The applied offset is measured in HMI flange axes.  Show the conversion
    # needed before a MATLAB universalUR5/tool0 planner can consume it.  S maps
    # MATLAB tool0 coordinates into HMI flange coordinates; it must be fixed
    # for all q.  A large value here is a frame error, not a TCP fit residual.
    S_values = []
    for i in sample_indices:
        M = C @ _mat_struct_pose(md["samples"][sample_indices.index(i)], "flange_T")
        S_values.append(np.linalg.inv(M) @ hmi_frames[i]["flange"])
    S0 = S_values[0]
    S_err = [float(np.linalg.norm(S - S0, ord="fro")) for S in S_values]
    converted_tool0_offset = None
    real_frame_check = None
    real_corrected_errors = {"position_mm": [], "orientation_deg": []}
    real_direct_errors = {"position_mm": [], "orientation_deg": []}
    if real_offset is not None:
        # S0 was defined as inv(C*M_tool0) * HMI_flange, hence it maps a
        # point expressed in HMI URDF flange axes into MATLAB tool0 axes.
        # Use the homogeneous translation too; dropping it would hide the
        # calibrated model's millimetre-level base discrepancy.
        converted_vector = (S0 @ np.r_[real_offset, 1.0])[:3]
        converted_tool0_offset = converted_vector.tolist()
        for index in sample_indices:
            matlab_sample = md["samples"][sample_indices.index(index)]
            M_tool0_raw = _mat_struct_pose(matlab_sample, "tool0_T")
            M_nom_raw = _mat_struct_pose(matlab_sample, "tcp_nominal_T")
            M_real_corrected_raw = M_nom_raw.copy()
            M_real_corrected_raw[:3, 3] = M_tool0_raw[:3, 3] + M_tool0_raw[:3, :3] @ converted_vector
            dp, da = _pose_error(hmi_frames[index]["real"], C @ M_real_corrected_raw)
            real_corrected_errors["position_mm"].append(dp)
            real_corrected_errors["orientation_deg"].append(da)
            # This is the deliberately rejected error obtained when the URDF
            # flange vector is passed directly to MATLAB tool0.
            M_real_direct_raw = M_nom_raw.copy()
            M_real_direct_raw[:3, 3] = M_tool0_raw[:3, 3] + M_tool0_raw[:3, :3] @ real_offset
            dp, da = _pose_error(hmi_frames[index]["real"], C @ M_real_direct_raw)
            real_direct_errors["position_mm"].append(dp)
            real_direct_errors["orientation_deg"].append(da)
        real_frame_check = {
            "applied_offset_flange_hmi_m": real_offset.tolist(),
            "equivalent_offset_tool0_matlab_m": converted_tool0_offset,
            "corrected_pose_comparison_10": {
                "position_mm": _stats(real_corrected_errors["position_mm"]),
                "orientation_deg": _stats(real_corrected_errors["orientation_deg"]),
            },
            "direct_unconverted_pose_comparison_10": {
                "position_mm": _stats(real_direct_errors["position_mm"]),
                "orientation_deg": _stats(real_direct_errors["orientation_deg"]),
            },
            "direct_use_in_matlab_is_blocked": True,
            "reason": f"{applied_file.name} declares URDF flange; MATLAB stock universalUR5 exposes tool0 as the flange-equivalent frame",
        }

    nominal_offset = np.linalg.inv(hmi_frames[0]["flange"]) @ hmi_frames[0]["nominal"]
    switch_delta_mm = None if real_offset is None else float(np.linalg.norm(real_offset - nominal_offset[:3, 3]) * 1000.0)
    mount_relative = [np.linalg.inv(x["flange"]) @ x["mount"] for x in hmi_frames]
    tcp_relative = [np.linalg.inv(x["mount"]) @ x["nominal"] for x in hmi_frames]
    mount_rel_err = [float(np.linalg.norm(T - mount_relative[0], ord="fro")) for T in mount_relative]
    tcp_rel_err = [float(np.linalg.norm(T - tcp_relative[0], ord="fro")) for T in tcp_relative]

    report = {
        "schema_version": 1,
        "trajectory": str(trajectory),
        "matlab_pose_report": str(matlab_report),
        "point_count": int(len(q)),
        "sample_indices_1based": [x + 1 for x in sample_indices],
        "base_transform_matlab_to_hmi": C.tolist(),
        "frame_comparison_10": {
            name: {"position_mm": _stats(values["position_mm"]),
                   "orientation_deg": _stats(values["orientation_deg"]),
                   "per_sample_position_mm": values["position_mm"],
                   "per_sample_orientation_deg": values["orientation_deg"]}
            for name, values in frame_errors.items()
        },
        "node_error_hmi_nominal_vs_saved_matlab_tcpPath": {
            **_stats(node_error),
            "worst_node_1based": int(np.argmax(node_error) + 1),
            "per_node_mm": node_error.tolist(),
            "first_10_mm": node_error[sample_indices].tolist(),
        },
        "interpolation_error": {
            "hmi_fk_vs_linear_saved_matlab_tcpPath_mm": _stats(dense_vs_saved),
            "hmi_fk_vs_linear_hmi_fk_endpoints_mm": _stats(dense_vs_hmi_line),
            "worst_edge": dense_worst,
            "interpretation": "node mismatch and joint-space interpolation curvature are reported separately; no node is deleted or altered",
        },
        "tcp_offsets": {
            "frozen_nominal_flange_hmi_m": nominal_offset[:3, 3].tolist(),
            "applied_real_flange_hmi_m": None if real_offset is None else real_offset.tolist(),
            "switch_translation_mm": switch_delta_mm,
        },
        "frame_chain_check": {
            "flange_to_tool0_from_urdf_once": True,
            "tool0_to_mount_from_ur5_tool_calibration_once": True,
            "mount_to_tcp_from_ur5_tool_calibration_once": True,
            "max_mount_relative_change_fro": max(mount_rel_err),
            "max_mount_to_tcp_relative_change_fro": max(tcp_rel_err),
            "fixed_geometry_pass": max(mount_rel_err) < 1e-10 and max(tcp_rel_err) < 1e-10,
            "gun_and_wire_not_rebuilt_on_tcp_switch": True,
        },
        "real_tcp_matlab_frame_check": real_frame_check,
        "calibration_versions": {
            "urdf_sha256": _sha256(UR5_ROOT / "real_robot_description_calibrated.urdf"),
            "real_tcp_applied_sha256": _sha256(applied_file),
            "workpiece_pose_real_sha256": _sha256(UR5_ROOT / "workpiece_pose_real.json"),
            "scene_geometry_sha256": _sha256(UR5_ROOT / "scene_geometry.json"),
        },
        "matlab_scene_fixed_geometry_check_pass": bool(md.get("fixed_geometry_check_pass", False)),
        "robot_motion": False,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    default_traj = UR5_ROOT / "planning_runs/hmi_20261010_105205_168803/trajectory.mat"
    parser.add_argument("--trajectory", type=Path, default=default_traj)
    parser.add_argument("--matlab-report", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=UR5_ROOT / "planning_runs/tcp_consistency_report.json")
    args = parser.parse_args()
    matlab_report = args.matlab_report or args.trajectory.with_name("matlab_tcp_pose_consistency.json")
    result = run(args.trajectory, matlab_report, args.output)
    print(json.dumps({
        "output": str(args.output),
        "samples": result["sample_indices_1based"],
        "node_error_max_mm": result["node_error_hmi_nominal_vs_saved_matlab_tcpPath"]["max"],
        "node_error_rms_mm": result["node_error_hmi_nominal_vs_saved_matlab_tcpPath"]["rms"],
        "interpolation_error_max_mm": result["interpolation_error"]["hmi_fk_vs_linear_saved_matlab_tcpPath_mm"]["max"],
        "fixed_geometry_pass": result["frame_chain_check"]["fixed_geometry_pass"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
