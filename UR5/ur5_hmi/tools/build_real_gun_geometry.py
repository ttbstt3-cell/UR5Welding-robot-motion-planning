#!/usr/bin/env python3
"""Build a reviewable real-gun geometry package without activating it.

V-HACD is preferred for the production collision mesh.  The current machine
does not provide a V-HACD executable/library, so this script creates a
deterministic slab-wise convex-hull *review proxy* only.  The proxy is marked
``PENDING_VHACD_AND_ALIGNMENT`` and is never consumed by HMI/MATLAB until a
human supplies a flange transform and explicitly changes the manifest status
to ``ACTIVE``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

try:
    import vtk
except ImportError as exc:  # pragma: no cover
    raise SystemExit(f"VTK is required to read STL: {exc}")


def read_points(path: Path) -> np.ndarray:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    poly = reader.GetOutput()
    if poly is None or poly.GetNumberOfPoints() < 4:
        raise RuntimeError("STL has no usable points")
    points = np.array([poly.GetPoint(i) for i in range(poly.GetNumberOfPoints())], dtype=float)
    # The audit has already established mm as the source unit.  This is a
    # scale conversion for a derived collision file, not a rewrite of STL.
    return points * 1e-3


def make_slab_hulls(points_m: np.ndarray, slab_count: int = 16) -> list[dict]:
    x0, x1 = float(points_m[:, 0].min()), float(points_m[:, 0].max())
    width = (x1 - x0) / slab_count
    components: list[dict] = []
    for i in range(slab_count):
        lo = x0 + i * width
        hi = x1 if i == slab_count - 1 else x0 + (i + 1) * width
        # A small overlap prevents a thin feature exactly on a slab boundary
        # from disappearing from both neighbouring proxies.
        pad = 0.002
        sample = points_m[(points_m[:, 0] >= lo - pad) & (points_m[:, 0] <= hi + pad)]
        if len(sample) < 4:
            continue
        # ConvexHull in 3-D needs non-coplanar points.  Keeping all source
        # points is deterministic; the resulting vertices are compact.
        try:
            hull = ConvexHull(sample)
        except Exception:
            continue
        vertices = sample[np.unique(hull.simplices)]
        components.append({
            "name": f"RealGunSlabHull_{i + 1:02d}",
            "type": "convex_hull_review_proxy",
            "vertices_m": vertices.tolist(),
            "source_x_bounds_m": [lo, hi],
        })
    if not components:
        raise RuntimeError("could not construct any convex review components")
    return components


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text(encoding="utf-8"))
    points_m = read_points(args.stl)
    components = make_slab_hulls(points_m)
    sha = hashlib.sha256(args.stl.read_bytes()).hexdigest()
    result = {
        "schema_version": "real_gun_geometry_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "STAGED_ALIGNMENT_REQUIRED",
        "activation_policy": "Only ACTIVE manifests with an explicitly confirmed flange transform may be loaded.",
        "source": {
            "stl": str(args.stl.resolve()),
            "sha256": sha,
            "units": "mm",
            "scale_to_m": 1e-3,
            "source_frame": "STL_SOURCE",
            "audit_file": str(args.audit.resolve()),
        },
        "visual": {
            "path": str(args.stl.resolve()),
            "units": "mm",
            "source_frame": "STL_SOURCE",
            "T_flange_source": None,
            "transform_status": "NOT_DEFINED",
        },
        "collision": {
            "status": "PENDING_VHACD_AND_ALIGNMENT",
            "method": "deterministic slab convex hull review proxy",
            "preferred_method": "V-HACD multi-convex decomposition",
            "source_frame": "STL_SOURCE",
            "T_mount_source": None,
            "T_tool0_source": None,
            "components": components,
            "component_count": len(components),
        },
        "mounting": {
            "status": "NOT_DEFINED",
            "frame": "URDF flange",
            "rotation_axis": None,
            "translation_m": None,
            "rotation_matrix": None,
            "reason": "The STL does not encode the flange origin or mounting transform; no centering/rotation was guessed.",
        },
        "wire": {
            "status": "PENDING_MEASUREMENT",
            "tip_source": None,
            "axis_source": None,
            "length_m": None,
            "diameter_m": None,
            "reason": "No reliable nozzle centre or feed-axis datum is encoded in the supplied mesh.",
        },
        "stl_audit_summary": {
            "triangle_count": audit.get("triangle_count"),
            "bounds": audit.get("raw_coordinate_bounds"),
            "unit_assessment": audit.get("unit_assessment"),
            "mesh_integrity": audit.get("mesh_integrity"),
            "contains_l_bracket": "visual inspection required; mesh includes the L-shaped bracket in the supplied STL preview",
        },
    }
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "component_count": len(components), "status": result["status"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
