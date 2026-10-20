#!/usr/bin/env python3
"""Review the draft STL mounting-face alignment without ROS or robot motion."""
from pathlib import Path
import hashlib
import json
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

ROOT = Path(__file__).resolve().parents[2]
GEOM = ROOT / "real_gun_geometry"
CAL = ROOT / "real_gun_mount_calibration.json"


def main():
    source = GEOM / "welding_gun_source.STL"
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source)); reader.Update()
    poly = reader.GetOutput()
    points = vtk_to_numpy(poly.GetPoints().GetData()).reshape(-1, 3)
    cells = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    triangles = points[cells]
    record = json.loads(CAL.read_text(encoding="utf-8"))
    alignment = json.loads((GEOM / "mount_hole_alignment.json").read_text(encoding="utf-8"))
    # Minimum source-X was a gun-body face, not the mounting flange. Use
    # the contact plane identified by its 63 mm disc and 50 mm bolt PCD.
    plane = alignment['gun_contact_plane_source_mm']
    mask = np.max(np.abs(triangles[:, :, 0] - plane), axis=1) < .001
    face_points = np.unique(np.round(triangles[mask].reshape(-1, 3), 6), axis=0)
    center = np.asarray(alignment['gun_plate_center_source_mm'], dtype=float)
    T = np.asarray(record["T_flange_source"], dtype=float)
    N = np.asarray(record["T_flange_source_nominal"], dtype=float)
    mapped = (T @ np.r_[center * 1e-3, 1.0])[:3]
    mapped_nominal = (N @ np.r_[center * 1e-3, 1.0])[:3]
    R = T[:3, :3]
    report = {
        "status": record.get("status"),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_face_candidate": {
            "plane_x_mm": float(center[0]),
            "center_source_mm": center.tolist(),
            "normal_source": [-1.0, 0.0, 0.0],
            "point_count": int(len(face_points)),
        },
        "T_flange_source": T.tolist(),
        "T_flange_source_nominal": N.tolist(),
        "face_center_in_flange_m": mapped.tolist(),
        "nominal_face_center_in_flange_m": mapped_nominal.tolist(),
        "face_center_in_flange_mm": (mapped * 1000).tolist(),
        "contact_plane_gap_mm": float(abs(mapped[0] * 1000 - alignment['robot_contact_plane_flange_mm'])),
        "hole_center_rms_mm": alignment['hole_center_rms_mm'],
        "hole_center_max_mm": alignment['hole_center_max_mm'],
        "four_hole_transform_match": bool(np.allclose(T, alignment['T_flange_source'], atol=1e-12)),
        "rotation_orthogonality_error": float(np.linalg.norm(R.T @ R - np.eye(3), "fro")),
        "rotation_determinant": float(np.linalg.det(R)),
        "physical_bolt_pattern_confirmed": False,
        "robot_motion": "NO",
        "application": "HMI preview only; MATLAB REAL scene remains inactive until confirmation",
    }
    (GEOM / "flange_face_alignment_review.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
