#!/usr/bin/env python3
"""Offline preview of the real welding-gun mounting roll.

This tool deliberately does not modify ``scene_geometry.json`` or
``ur5_tool_calibration.mat``.  It applies a candidate +/-90 degree rotation
about the URDF J6/flange axis to the exported gun hull and wire only, while
keeping the calibrated TCP point fixed in the flange frame.  The output is a
JSON report (and, when matplotlib is available, a PNG preview) that can be
reviewed before committing a real-scene variant.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation


ROOT = Path("/home/ttbstt/jixiebi/UR5")
URDF = ROOT / "real_robot_description_calibrated.urdf"
SCENE = ROOT / "scene_geometry.json"
CALIB = ROOT / "ur5_tool_calibration.mat"
REAL_TCP = ROOT / "real_tcp_calibration_applied.json"
ACTIVE_TCP = ROOT / "real_tcp_calibration_active.json"
OUT_JSON = ROOT / "real_gun_mount_roll_preview.json"
OUT_PNG = ROOT / "real_gun_mount_roll_preview.png"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def pose_to_T(pose: dict) -> np.ndarray:
    x, y, z, w = [float(v) for v in pose.get("quaternion", [0, 0, 0, 1])]
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    T[:3, 3] = np.asarray(pose.get("position", [0, 0, 0]), dtype=float)
    return T


def homogeneous(R: np.ndarray, p: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float)
    if p is not None:
        T[:3, 3] = np.asarray(p, dtype=float)
    return T


def json_matrix(T: np.ndarray) -> list[list[float]]:
    return np.asarray(T, dtype=float).tolist()


def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    p = np.asarray(points, dtype=float)
    return (T @ np.c_[p, np.ones(len(p))].T).T[:, :3]


def extract_j6_axis(model, transforms: dict[str, np.ndarray]) -> np.ndarray:
    joint = model.joints["wrist_3_joint"]
    parent_T = transforms[joint["parent"]]
    axis_base = parent_T[:3, :3] @ joint["origin"][:3, :3] @ np.asarray(joint["axis"], dtype=float)
    axis_flange = transforms["flange"][:3, :3].T @ axis_base
    return axis_flange / max(float(np.linalg.norm(axis_flange)), 1e-12)


def build_wire_transform(T_mount_tcp: np.ndarray, head_mount: np.ndarray, length: float) -> np.ndarray:
    """Reproduce sync_ur5_joint_states.m's TCP-relative wire construction."""
    p_head_tcp = (np.linalg.inv(T_mount_tcp) @ np.r_[head_mount, 1.0])[:3]
    z_wire = -p_head_tcp / max(float(np.linalg.norm(p_head_tcp)), 1e-12)
    x_ref = np.array([1.0, 0.0, 0.0])
    if abs(float(x_ref @ z_wire)) > 0.95:
        x_ref = np.array([0.0, 1.0, 0.0])
    x_wire = np.cross(x_ref, z_wire)
    x_wire /= max(float(np.linalg.norm(x_wire)), 1e-12)
    y_wire = np.cross(z_wire, x_wire)
    y_wire /= max(float(np.linalg.norm(y_wire)), 1e-12)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.column_stack((x_wire, y_wire, z_wire))
    T[:3, 3] = p_head_tcp + 0.5 * length * z_wire
    return T


def wire_endpoints(T_flange_wire: np.ndarray, length: float) -> tuple[np.ndarray, np.ndarray]:
    # vtkCylinderSource's native axis is +Y; the HMI applies Rx(+90 deg)
    # before the parent transform, so its world axis is the second column.
    axis = T_flange_wire[:3, :3] @ np.array([0.0, 1.0, 0.0])
    center = T_flange_wire[:3, 3]
    return center - 0.5 * length * axis, center + 0.5 * length * axis


def make_candidate(
    deg: float,
    axis_flange: np.ndarray,
    T_flange_tool0: np.ndarray,
    T_tool0_mount: np.ndarray,
    T_mount_hull: np.ndarray,
    T_mount_tcp: np.ndarray,
    hull_vertices: np.ndarray,
    p_tcp_nominal: np.ndarray,
    p_tcp_real: np.ndarray,
    wire_length: float,
    T_flange_wire_current: np.ndarray,
    head_vertex_index_current: int,
    component_centers_mount: np.ndarray | None = None,
) -> dict:
    R = Rotation.from_rotvec(axis_flange * math.radians(deg)).as_matrix()
    T_corr_flange = homogeneous(R)

    # The correction is a rigid transform about the flange origin.  It is
    # conjugated into the tool0/mount chain for the MATLAB rigidBody model.
    T_tool0_mount_new = (
        np.linalg.inv(T_flange_tool0)
        @ T_corr_flange
        @ T_flange_tool0
        @ T_tool0_mount
    )
    T_mount_tcp_new = (
        np.linalg.inv(T_tool0_mount_new)
        @ T_tool0_mount
        @ T_mount_tcp
    )

    T_flange_hull_old = T_flange_tool0 @ T_tool0_mount @ T_mount_hull
    T_flange_hull_new = T_corr_flange @ T_flange_hull_old
    hull_world = transform_points(T_flange_hull_new, hull_vertices)
    T_flange_tcp_old = T_flange_tool0 @ T_tool0_mount @ T_mount_tcp
    T_flange_tcp_new = T_flange_tool0 @ T_tool0_mount_new @ T_mount_tcp_new
    tcp_fixed_error = float(np.linalg.norm(T_flange_tcp_new[:3, 3] - T_flange_tcp_old[:3, 3]))

    nominal_dist = np.linalg.norm(hull_world - p_tcp_nominal, axis=1)
    real_dist = np.linalg.norm(hull_world - p_tcp_real, axis=1)
    nominal_index = int(np.argmin(nominal_dist))
    real_index = int(np.argmin(real_dist))
    # The nozzle/head vertex is identified once in the unrotated, frozen
    # geometry.  Do not re-select a different vertex after rolling the hull:
    # that would silently change which physical end of the gun is treated as
    # the nozzle and would corrupt the wire comparison.
    head = hull_world[int(head_vertex_index_current)]

    # Rotating the existing wire as a rigid part keeps it attached to the gun
    # hull.  The TCP is *not* rotated; the distances below expose any physical
    # inconsistency that must be resolved before committing the variant.
    T_flange_wire_new = T_corr_flange @ T_flange_wire_current
    e0, e1 = wire_endpoints(T_flange_wire_new, wire_length)

    component_pairwise_error_mm = None
    if component_centers_mount is not None and len(component_centers_mount) > 1:
        before = np.asarray(component_centers_mount, dtype=float)
        after = (R @ before.T).T
        before_d = np.linalg.norm(before[:, None, :] - before[None, :, :], axis=2)
        after_d = np.linalg.norm(after[:, None, :] - after[None, :, :], axis=2)
        component_pairwise_error_mm = float(np.max(np.abs(before_d - after_d)) * 1000.0)

    return {
        "angle_deg_right_hand_about_j6_axis": float(deg),
        "axis_flange": axis_flange.tolist(),
        "correction_flange": json_matrix(T_corr_flange),
        "T_tool0_mount_corrected": json_matrix(T_tool0_mount_new),
        "T_mount_tcp_compensated_for_fixed_tcp": json_matrix(T_mount_tcp_new),
        "tcp_fixed_error_mm": tcp_fixed_error * 1000.0,
        "hull_bbox_m": {"min": hull_world.min(axis=0).tolist(), "max": hull_world.max(axis=0).tolist()},
        "head_vertex_current_frozen_selection": int(head_vertex_index_current) + 1,
        "head_vertex_nearest_nominal_tcp_after_roll": nominal_index + 1,
        "head_vertex_nearest_real_tcp": real_index + 1,
        "head_position_flange_m": head.tolist(),
        "head_to_nominal_tcp_mm": float(np.linalg.norm(head - p_tcp_nominal) * 1000.0),
        "head_to_real_tcp_mm": float(np.linalg.norm(head - p_tcp_real) * 1000.0),
        "wire_endpoints_flange_m": [e0.tolist(), e1.tolist()],
        "wire_endpoint_to_nominal_tcp_mm": [
            float(np.linalg.norm(e0 - p_tcp_nominal) * 1000.0),
            float(np.linalg.norm(e1 - p_tcp_nominal) * 1000.0),
        ],
        "wire_endpoint_to_real_tcp_mm": [
            float(np.linalg.norm(e0 - p_tcp_real) * 1000.0),
            float(np.linalg.norm(e1 - p_tcp_real) * 1000.0),
        ],
        "wire_length_mm": wire_length * 1000.0,
        "components_rigid_parent_transform": True,
        "component_pairwise_distance_error_mm": component_pairwise_error_mm,
        "wire_rigid_parent_transform": True,
    }


def make_preview_png(report: dict, hull_vertices: np.ndarray, faces: np.ndarray, p_tcp_real: np.ndarray, p_tcp_nominal: np.ndarray) -> None:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except Exception:
        return

    # Reconstruct each candidate's hull from the stored source data and the
    # flange correction.  The plot is in the flange frame and is diagnostic;
    # it does not alter any HMI or MATLAB geometry.
    fig = plt.figure(figsize=(13, 6), dpi=150)
    for col, (title, candidate) in enumerate((("+90°", report["candidates"]["plus90"]), ("-90°", report["candidates"]["minus90"])), 1):
        ax = fig.add_subplot(1, 2, col, projection="3d")
        T = np.asarray(candidate["correction_flange"], dtype=float)
        # The caller stores the current hull transform in the report.
        Tfh = np.asarray(report["T_flange_hull_current"], dtype=float)
        pts = transform_points(T @ Tfh, hull_vertices)
        polygons = [pts[np.asarray(face, dtype=int) - 1] for face in faces]
        ax.add_collection3d(Poly3DCollection(polygons, facecolor=(1.0, 0.55, 0.1), alpha=0.30, edgecolor=(0.9, 0.35, 0.0), linewidth=0.35))
        h = np.asarray(candidate["head_position_flange_m"], dtype=float)
        ax.scatter(*p_tcp_real, color="red", s=40, label="real TCP")
        ax.scatter(*p_tcp_nominal, color="gray", s=28, label="frozen TCP")
        ax.scatter(*h, color="black", s=24, label="gun head")
        e = np.asarray(candidate["wire_endpoints_flange_m"], dtype=float)
        ax.plot(e[:, 0], e[:, 1], e[:, 2], color=(0.95, 0.75, 0.1), linewidth=2.0, label="wire")
        ax.set_title(f"J6 axis roll {title}\nhead→real TCP {candidate['head_to_real_tcp_mm']:.1f} mm")
        ax.set_xlabel("flange X (m)")
        ax.set_ylabel("flange Y (m)")
        ax.set_zlabel("flange Z (m)")
        all_points = np.vstack((pts, p_tcp_real, p_tcp_nominal, h, e))
        pmin = all_points.min(axis=0)
        pmax = all_points.max(axis=0)
        centre = 0.5 * (pmin + pmax)
        half = max(float(np.max(pmax - pmin)) * 0.65, 0.06)
        ax.set_xlim(centre[0] - half, centre[0] + half)
        ax.set_ylim(centre[1] - half, centre[1] + half)
        ax.set_zlim(centre[2] - half, centre[2] + half)
        ax.legend(fontsize=7, loc="upper left")
        ax.set_box_aspect((1.3, 1.0, 1.0))
        ax.view_init(elev=24, azim=-62)
    fig.suptitle("Offline welding-gun mount roll preview (no scene write)")
    fig.tight_layout()
    fig.savefig(OUT_PNG)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=OUT_JSON)
    ap.add_argument("--png", type=Path, default=OUT_PNG)
    args = ap.parse_args()

    sys.path.insert(0, str(ROOT / "ur5_hmi"))
    from gui.robot_view import UrdfKinematicModel

    scene = json.loads(SCENE.read_text(encoding="utf-8"))
    calib = loadmat(str(CALIB), squeeze_me=True)
    real_tcp_path = ACTIVE_TCP if ACTIVE_TCP.is_file() else REAL_TCP
    real_tcp = json.loads(real_tcp_path.read_text(encoding="utf-8"))
    model = UrdfKinematicModel(urdf_path=URDF)
    transforms = model.link_transforms_in_base(np.zeros(6, dtype=float))
    T_flange_tool0 = np.linalg.inv(transforms["flange"]) @ transforms["tool0"]
    axis_flange = extract_j6_axis(model, transforms)
    T_tool0_mount = np.asarray(calib["T_tool0_mount"], dtype=float)
    T_mount_tcp = np.asarray(calib["T_mount_tcp"], dtype=float)
    hull = scene["welding_gun"]["collision_hull"]
    hull_vertices = np.asarray(hull["vertices_m"], dtype=float)
    hull_faces = np.asarray(hull["faces"], dtype=int)
    T_mount_hull = pose_to_T(hull.get("pose_mount", {}))
    T_flange_hull_current = T_flange_tool0 @ T_tool0_mount @ T_mount_hull
    T_flange_tcp_nominal = T_flange_tool0 @ T_tool0_mount @ T_mount_tcp
    p_tcp_nominal = T_flange_tcp_nominal[:3, 3]
    p_tcp_real = np.asarray(real_tcp["position_flange_tcp_m"], dtype=float)

    nominal_dist = np.linalg.norm(transform_points(T_flange_hull_current, hull_vertices) - p_tcp_nominal, axis=1)
    nominal_head = transform_points(T_flange_hull_current, hull_vertices)[int(np.argmin(nominal_dist))]
    head_mount = (np.linalg.inv(T_flange_tool0 @ T_tool0_mount) @ np.r_[nominal_head, 1.0])[:3]
    wire_length = float(calib.get("wireLengthM", np.array(0.018)).reshape(-1)[0])
    T_tcp_wire = build_wire_transform(T_mount_tcp, head_mount, wire_length)
    # Cylinder source uses +Y as its local axis, then the HMI/MATLAB applies
    # Rx(+90 deg) to map it to the wire +Z direction.
    T_source = homogeneous(Rotation.from_rotvec(np.array([1.0, 0.0, 0.0]) * (math.pi / 2.0)).as_matrix())
    T_flange_wire_current = T_flange_tool0 @ T_tool0_mount @ T_mount_tcp @ T_tcp_wire @ T_source
    component_centers_mount = np.asarray(
        [np.asarray(c["vertices_m"], dtype=float).mean(axis=0) for c in hull.get("components", [])],
        dtype=float,
    )

    report = {
        "schema_version": 1,
        "status": "PREVIEW_ONLY_NOT_COMMITTED",
        "rotation_reference": "URDF flange origin; URDF wrist_3_joint/J6 axis",
        "sign_convention": "+90/-90 are right-hand rotations about the flange-frame J6 axis",
        "inputs": {
            "urdf": str(URDF),
            "scene_geometry": str(SCENE),
            "frozen_tool_calibration": str(CALIB),
            "real_tcp_calibration": str(real_tcp_path),
            "sha256": {str(URDF.name): sha256(URDF), str(SCENE.name): sha256(SCENE), str(CALIB.name): sha256(CALIB), str(real_tcp_path.name): sha256(real_tcp_path)},
        },
        "axis_flange": axis_flange.tolist(),
        "T_flange_tool0": json_matrix(T_flange_tool0),
        "T_tool0_mount_current": json_matrix(T_tool0_mount),
        "T_mount_hull_current": json_matrix(T_mount_hull),
        "T_mount_tcp_current": json_matrix(T_mount_tcp),
        "T_flange_hull_current": json_matrix(T_flange_hull_current),
        "tcp_nominal_flange_m": p_tcp_nominal.tolist(),
        "tcp_real_flange_m": p_tcp_real.tolist(),
        "tcp_switch_translation_mm": float(np.linalg.norm(p_tcp_real - p_tcp_nominal) * 1000.0),
        "wire_length_mm": wire_length * 1000.0,
        "components": int(hull.get("component_count", len(hull.get("components", [])))),
    }
    current_head_index = int(np.argmin(nominal_dist))
    report["current_head_vertex_selection"] = current_head_index + 1
    report["candidates"] = {
        "plus90": make_candidate(90.0, axis_flange, T_flange_tool0, T_tool0_mount, T_mount_hull, T_mount_tcp, hull_vertices, p_tcp_nominal, p_tcp_real, wire_length, T_flange_wire_current, current_head_index, component_centers_mount),
        "minus90": make_candidate(-90.0, axis_flange, T_flange_tool0, T_tool0_mount, T_mount_hull, T_mount_tcp, hull_vertices, p_tcp_nominal, p_tcp_real, wire_length, T_flange_wire_current, current_head_index, component_centers_mount),
    }
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    make_preview_png(report, hull_vertices, hull_faces, p_tcp_real, p_tcp_nominal)
    if args.png != OUT_PNG and OUT_PNG.exists():
        args.png.write_bytes(OUT_PNG.read_bytes())
    print(f"REAL_GUN_MOUNT_ROLL_PREVIEW={args.output}")
    for name in ("plus90", "minus90"):
        c = report["candidates"][name]
        print(f"{name}: head_to_real_tcp_mm={c['head_to_real_tcp_mm']:.3f}, wire_real_tcp_min_mm={min(c['wire_endpoint_to_real_tcp_mm']):.3f}, tcp_fixed_error_mm={c['tcp_fixed_error_mm']:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
