#!/usr/bin/env python3
"""Read-only audit for the replacement welding-gun STL.

The audit deliberately does not translate, rotate, centre, or rewrite the
mesh.  The source coordinates are reported exactly as stored in the STL; the
only unit inference is a warning-level heuristic based on overall size.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from datetime import datetime, timezone
from pathlib import Path


def read_binary_stl(path: Path):
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError("file is shorter than a binary STL header")
    count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + 50 * count
    if expected != len(raw):
        raise ValueError(
            f"binary STL size mismatch: header says {count} triangles "
            f"({expected} bytes), file has {len(raw)} bytes"
        )
    vertices = []
    degenerate = 0
    nonfinite = 0
    normals = []
    for i in range(count):
        off = 84 + 50 * i
        values = struct.unpack_from("<12f", raw, off)
        normal = values[:3]
        tri = [values[3:6], values[6:9], values[9:12]]
        normals.append(normal)
        vertices.extend(tri)
        if not all(math.isfinite(v) for v in values):
            nonfinite += 1
        ax = [tri[1][j] - tri[0][j] for j in range(3)]
        bx = [tri[2][j] - tri[0][j] for j in range(3)]
        cross = [ax[1] * bx[2] - ax[2] * bx[1],
                 ax[2] * bx[0] - ax[0] * bx[2],
                 ax[0] * bx[1] - ax[1] * bx[0]]
        if sum(v * v for v in cross) <= 1e-18:
            degenerate += 1
    return raw, count, vertices, degenerate, nonfinite


def audit(path: Path) -> dict:
    raw, tri_count, vertices, degenerate, nonfinite = read_binary_stl(path)
    mins = [min(v[j] for v in vertices) for j in range(3)]
    maxs = [max(v[j] for v in vertices) for j in range(3)]
    size = [maxs[j] - mins[j] for j in range(3)]
    largest = max(size)
    # This is intentionally only a unit warning, never an automatic transform.
    unit_guess = "mm" if 10.0 < largest < 5000.0 else ("m" if largest < 10.0 else "unknown")
    result = {
        "schema_version": "real_gun_stl_audit_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_file": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "file_size_bytes": len(raw),
        "format": "binary_stl",
        "triangle_count": tri_count,
        "raw_coordinate_bounds": {"min": mins, "max": maxs, "size": size},
        "raw_coordinate_origin": [0.0, 0.0, 0.0],
        "unit_assessment": {
            "inferred_unit": unit_guess,
            "scale_to_m": 1e-3 if unit_guess == "mm" else (1.0 if unit_guess == "m" else None),
            "method": "size heuristic only; source mesh is not modified",
            "requires_human_confirmation": True,
        },
        "mesh_integrity": {
            "nonfinite_triangle_count": nonfinite,
            "degenerate_triangle_count": degenerate,
            "binary_size_matches_triangle_count": True,
        },
        "installation": {
            "status": "NOT_DEFINED",
            "reason": "STL contains no flange frame or mounting transform metadata",
            "automatic_center_rotate_scale_applied": False,
        },
        "tip_and_wire": {
            "status": "NOT_DEFINED",
            "reason": "No reliable nozzle centre or wire axis metadata was found in STL file",
            "automatic_tip_inference": False,
        },
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.stl)
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
