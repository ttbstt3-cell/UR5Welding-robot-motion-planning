#!/usr/bin/env python3
"""Audit available welding-gun visual/collision geometry without guessing.

The CoppeliaSim object path in ``scene_geometry.json`` is not a local CAD
file.  This report deliberately distinguishes that path, collision proxies,
and measured welding-tip data; it never promotes a collision-hull vertex to a
conductive-tip centre or changes a planning scene.
"""
from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "scene_geometry.json"
AUDIT = ROOT / "matlab仿真" / "current_weld_geometry_audit.mat"
OUT = ROOT / "diagnostics" / "gun_visual_geometry_audit.json"


def sha(path: Path):
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def scalar(obj, name):
    value = getattr(obj, name, None)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main():
    scene = json.loads(SCENE.read_text(encoding="utf-8"))
    gun = scene.get("welding_gun", {})
    hull = gun.get("collision_hull", {})
    visual_path = str(gun.get("visual_path", ""))
    # Only search files in this project; never search the user's home broadly.
    suffixes = {".stl", ".dae", ".obj", ".step", ".stp", ".iges", ".igs", ".ply"}
    local_assets = [str(p) for p in ROOT.rglob("*") if p.is_file()
                    and p.suffix.lower() in suffixes
                    and "direction_cascade" not in str(p)]
    audit = {}
    if AUDIT.is_file():
        obj = loadmat(str(AUDIT), squeeze_me=True, struct_as_record=False).get("audit")
        audit = {
            "wire_length_mm": None if obj is None else scalar(obj, "wireLengthM"),
            "wire_diameter_mm": None if obj is None else scalar(obj, "wireDiameterM"),
        }
        if audit["wire_length_mm"] is not None:
            audit["wire_length_mm"] *= 1000.0
        if audit["wire_diameter_mm"] is not None:
            audit["wire_diameter_mm"] *= 1000.0
    report = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "OFFLINE_READ_ONLY",
        "visual_geometry": {
            "coppeliasim_object_path": visual_path,
            "local_cad_assets_found": local_assets,
            "status": "NO_TRUSTED_LOCAL_CAD_FOUND" if not local_assets else "LOCAL_ASSET_REQUIRES_MANUAL_REVIEW",
            "note": "scene_geometry.visual_path is a CoppeliaSim object path, not proof of a local STL/CAD mesh.",
        },
        "collision_geometry": {
            "name": hull.get("name"),
            "type": hull.get("type"),
            "component_count": hull.get("component_count"),
            "vertex_count": hull.get("vertex_count"),
            "face_count": hull.get("face_count"),
            "frame": (hull.get("pose_mount") or {}).get("frame"),
            "status": "COLLISION_PROXY_ONLY",
            "note": "Four convex components are suitable for collision checking, not for identifying a conductive-tip centre.",
        },
        "tcp_geometry": {
            "weld_tcp_path": gun.get("weld_tcp", {}).get("path"),
            "weld_tcp_pose_mount": gun.get("weld_tcp", {}).get("pose_mount"),
            "status": "REFERENCE_POINT_PRESENT_BUT_TIP_IDENTITY_UNVERIFIED",
            "note": "WeldTCP and the nearest hull vertex are not accepted as a measured nozzle/contact-tip centre without physical confirmation.",
        },
        "wire_geometry": {
            **audit,
            "length_source": str(AUDIT) if AUDIT.is_file() else None,
            "diameter_source": str(AUDIT) if AUDIT.is_file() else None,
            "measurement_status": "RECORDED_AUDIT_VALUES_NOT_NEWLY_MEASURED",
            "axis_status": "MODEL_DEFINED_ACTIVE",
            "tip_status": "MODEL_DEFINED_ACTIVE",
            "derived_axis_policy": "Use the fixed CAD terminal-face point and source-frame axis from real_gun_geometry.json; no physical measurement is required for this model-defined record.",
        },
        "required_physical_measurements": [
            "flange->nozzle/conductive-tip centre XYZ in a documented flange frame",
            "two measured points along the actual wire/feed axis (axis direction and sign)",
            "wire stick-out length from the measured nozzle/contact point",
            "tool orientation/reference used by the pendant TCP",
        ],
        "source_hashes": {"scene_geometry.json": sha(SCENE), "current_weld_geometry_audit.mat": sha(AUDIT)},
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
