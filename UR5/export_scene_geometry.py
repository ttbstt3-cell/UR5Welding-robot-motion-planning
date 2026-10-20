#!/usr/bin/env python3
"""Export current CoppeliaSim tool and workpiece geometry.

This exporter is intentionally independent of the archived scene-sync
scripts.  CoppeliaSim is the source of truth for the current object names,
poses and collision-proxy dimensions.  The fixed UR5/TCP calibration is not
written here; it remains in ``ur5_tool_calibration.mat``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


DEFAULT_OUTPUT = Path(__file__).resolve().with_name("scene_geometry.json")
BASE_PATH = "/UR5"
CONNECTION_PATH = "/UR5/connection"
MOUNT_PATH = "/UR5/connection/WeldingGunMount"
TCP_PATH = f"{MOUNT_PATH}/WeldTCP"
HULL_PATH = f"{MOUNT_PATH}/GunCollisionHull"
WORKPIECE_PATH = "/Workpiece"

GUN_PROXY_NAMES = [
    "GunCollision_mount",
    "GunCollision_body",
    "GunCollision_neck1",
    "GunCollision_neck2",
    "GunCollision_nozzle",
]


def floats(values: Any) -> list[float]:
    return [float(value) for value in values]


def pose(sim: Any, obj: int, reference: int, frame: str) -> dict[str, Any]:
    values = sim.getObjectPose(obj, reference)
    return {
        "frame": frame,
        "position": floats(values[:3]),
        # CoppeliaSim ordering is [qx, qy, qz, qw].
        "quaternion": floats(values[3:7]),
    }


def shape_record(sim: Any, obj: int, name: str, mount: int, base: int) -> dict[str, Any]:
    _, primitive_type, primitive_params = sim.getShapeGeomInfo(obj)
    type_names = {
        int(sim.primitiveshape_cuboid): "box",
        int(sim.primitiveshape_cylinder): "cylinder",
        int(sim.primitiveshape_cone): "cone",
        int(sim.primitiveshape_spheroid): "spheroid",
        int(sim.primitiveshape_capsule): "capsule",
        int(sim.primitiveshape_disc): "disc",
    }
    geometry_type = type_names.get(int(primitive_type), "mesh")
    size = floats(primitive_params[:3])

    record: dict[str, Any] = {
        "name": name,
        "path": f"{MOUNT_PATH}/{name}",
        "type": geometry_type,
        "primitive_type_code": int(primitive_type),
        "size_m": size,
        "pose_mount": pose(sim, obj, mount, "WeldingGunMount"),
        "pose_base": pose(sim, obj, base, "UR5_base"),
        "special_property": int(sim.getObjectSpecialProperty(obj)),
        "visibility_layer": int(
            sim.getObjectInt32Param(obj, sim.objintparam_visibility_layer)
        ),
    }
    if geometry_type == "cylinder":
        record["radius_m"] = size[0] / 2.0
        record["length_m"] = size[2]
    return record


def mesh_bounds(sim: Any, obj: int) -> tuple[list[float], list[float]]:
    vertices, _, _ = sim.getShapeMesh(obj)
    points = np.asarray(vertices, dtype=float).reshape(-1, 3)
    return points.min(axis=0).tolist(), points.max(axis=0).tolist()


def mesh_record(sim: Any, obj: int, mount: int, base: int) -> dict[str, Any]:
    """Export a shape mesh in its own local frame.

    CoppeliaSim returns triangle indices starting at zero.  MATLAB's
    ``collisionMesh`` expects an N-by-3 vertex matrix and an M-by-3 face
    matrix with one-based indices, so the conversion is done here once.
    """
    vertices, indices, _ = sim.getShapeMesh(obj)
    vertex_array = np.asarray(vertices, dtype=float).reshape(-1, 3)
    face_array = np.asarray(indices, dtype=int).reshape(-1, 3) + 1

    # A convex-hull add-on applied to a compound visual can produce a
    # convex *compound* (one convex mesh per source component).  Preserve
    # those components instead of flattening them into a larger hull.
    try:
        component_handles = sim.getIntArrayProperty(obj, "meshes")
    except Exception:
        component_handles = []

    components: list[dict[str, Any]] = []
    for component_index, mesh_handle in enumerate(component_handles, start=1):
        component_vertices = np.asarray(
            sim.getFloatArrayProperty(mesh_handle, "vertices"),
            dtype=float,
        ).reshape(-1, 3)
        component_indices = np.asarray(
            sim.getIntArrayProperty(mesh_handle, "indices"),
            dtype=int,
        ).reshape(-1, 3) + 1
        components.append(
            {
                "index": component_index,
                "vertices_m": component_vertices.tolist(),
                "faces": component_indices.tolist(),
                "vertex_count": int(component_vertices.shape[0]),
                "face_count": int(component_indices.shape[0]),
            }
        )

    geom_result, _, _ = sim.getShapeGeomInfo(obj)
    is_compound = bool(int(geom_result) & 1)
    is_convex = bool(int(geom_result) & 4)
    if is_compound and is_convex:
        geometry_type = "convex_compound"
    elif is_convex:
        geometry_type = "convex_hull"
    else:
        geometry_type = "mesh"

    bbox_size, bbox_pose = sim.getShapeBB(obj)
    return {
        "name": "GunCollisionHull",
        "path": HULL_PATH,
        "type": geometry_type,
        "vertices_m": vertex_array.tolist(),
        "faces": face_array.tolist(),
        "vertex_count": int(vertex_array.shape[0]),
        "face_count": int(face_array.shape[0]),
        "component_count": len(components),
        "components": components,
        "pose_mount": pose(sim, obj, mount, "WeldingGunMount"),
        "pose_base": pose(sim, obj, base, "UR5_base"),
        "bbox_size_m": floats(bbox_size),
        "bbox_pose_object": floats(bbox_pose),
    }


def make_workpiece_proxies(mesh_min: list[float], mesh_max: list[float]) -> tuple[list[dict[str, Any]], float]:
    """Build four MATLAB collision boxes from the current U-mesh bounds.

    The simplified planning model is a bottom plate plus three walls.  The
    two x-walls and the y=ly wall are retained, leaving the y=0 side open.
    This is deliberately a four-box approximation; the source Workpiece
    mesh and its pose are still exported separately below.
    """
    lo = np.asarray(mesh_min, dtype=float)
    hi = np.asarray(mesh_max, dtype=float)
    extent = hi - lo

    # Current scene has horizontal plate levels z=0, 0.005 and z=0.235.
    # Derive the first nonzero z level rather than hard-coding the scene size.
    thickness = 0.005
    if lo[2] >= 0.0 and extent[2] > 0.0:
        thickness = min(0.005, float(extent[2]))

    lx, ly, height = [float(value) for value in extent]
    q_identity = [0.0, 0.0, 0.0, 1.0]

    def proxy(name: str, size: list[float], center: list[float]) -> dict[str, Any]:
        return {
            "name": name,
            "type": "box",
            "size_m": size,
            "pose_workpiece": {
                "frame": "Workpiece",
                "position": center,
                "quaternion": q_identity,
            },
            "source": "derived from current Workpiece mesh bounds",
        }

    proxies = [
        proxy(
            "WorkpieceCollision_bottom",
            [lx, ly, thickness],
            [lx / 2.0, ly / 2.0, thickness / 2.0],
        ),
        proxy(
            "WorkpieceCollision_wall_x0",
            [thickness, ly, height],
            [thickness / 2.0, ly / 2.0, height / 2.0],
        ),
        proxy(
            "WorkpieceCollision_wall_x1",
            [thickness, ly, height],
            [lx - thickness / 2.0, ly / 2.0, height / 2.0],
        ),
        proxy(
            "WorkpieceCollision_wall_y1",
            [lx, thickness, height],
            [lx / 2.0, ly - thickness / 2.0, height / 2.0],
        ),
    ]
    return proxies, thickness


def export_scene(output_path: Path, host: str, port: int) -> dict[str, Any]:
    client = RemoteAPIClient(host, port)
    sim = client.getObject("sim")

    base = sim.getObject(BASE_PATH)
    connection = sim.getObject(CONNECTION_PATH)
    mount = sim.getObject(MOUNT_PATH)
    tcp = sim.getObject(TCP_PATH)
    visual = sim.getObject(f"{MOUNT_PATH}/WeldingGun_visual")
    hull = sim.getObject(HULL_PATH)
    workpiece = sim.getObject(WORKPIECE_PATH)

    # The current scene may use the newer single/compound GunCollisionHull
    # and therefore no longer contain the legacy five proxy objects.  Export
    # legacy proxies when present, but do not make them a hard requirement.
    gun_proxies: list[dict[str, Any]] = []
    missing_gun_proxies: list[str] = []
    for name in GUN_PROXY_NAMES:
        try:
            proxy_obj = sim.getObject(f"{MOUNT_PATH}/{name}")
        except Exception:
            missing_gun_proxies.append(name)
            continue
        gun_proxies.append(shape_record(sim, proxy_obj, name, mount, base))
    if missing_gun_proxies:
        print(
            "Legacy gun proxies not present (skipped): "
            + ", ".join(missing_gun_proxies)
        )

    mesh_min, mesh_max = mesh_bounds(sim, workpiece)
    workpiece_proxies, plate_thickness = make_workpiece_proxies(mesh_min, mesh_max)
    bbox_size, bbox_pose = sim.getShapeBB(workpiece)

    data: dict[str, Any] = {
        "schema_version": 2,
        "source": {
            "scene": "/home/ttbstt/jixiebi/UR5/ur5.ttt",
            "base_path": BASE_PATH,
            "mount_path": MOUNT_PATH,
            "workpiece_path": WORKPIECE_PATH,
            "remote_api": {"host": host, "port": port},
        },
        "frames": {
            "reference_frame": "UR5_base",
            "base_path": BASE_PATH,
            "base_pose_world": pose(sim, base, -1, "world"),
            "connection_pose_base_current": pose(sim, connection, base, "UR5_base"),
            "mount_pose_connection": pose(sim, mount, connection, "connection"),
            "tcp_pose_mount": pose(sim, tcp, mount, "WeldingGunMount"),
            "calibration_source": "ur5_tool_calibration.mat",
        },
        "welding_gun": {
            "visual_path": f"{MOUNT_PATH}/WeldingGun_visual",
            "visual_pose_mount": pose(sim, visual, mount, "WeldingGunMount"),
            "mount_pose_base": pose(sim, mount, base, "UR5_base"),
            "collision_hull": mesh_record(sim, hull, mount, base),
            "weld_tcp": {
                "path": TCP_PATH,
                "pose_mount": pose(sim, tcp, mount, "WeldingGunMount"),
                "pose_base_current": pose(sim, tcp, base, "UR5_base"),
            },
            "collision_proxies": gun_proxies,
        },
        "workpiece": {
            "path": WORKPIECE_PATH,
            "pose_base": pose(sim, workpiece, base, "UR5_base"),
            "pose_world": pose(sim, workpiece, -1, "world"),
            "bbox_size_m": floats(bbox_size),
            "bbox_pose_object": floats(bbox_pose),
            "mesh_bounds_object": {"min": floats(mesh_min), "max": floats(mesh_max)},
            "geometry_description": (
                "U-shaped Workpiece approximation: bottom plate plus three walls "
                "(x=0, x=Lx, y=Ly); open side is y=0"
            ),
            "collision_proxy_count": len(workpiece_proxies),
            "collision_proxy_open_side": "y=0",
            "plate_thickness_m": plate_thickness,
            "collision_proxies": workpiece_proxies,
        },
    }

    output_path.write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    data = export_scene(args.output, args.host, args.port)
    print(f"Exported: {args.output}")
    print(f"Reference frame: {data['frames']['reference_frame']}")
    print(f"Gun proxies: {len(data['welding_gun']['collision_proxies'])}")
    hull_data = data["welding_gun"]["collision_hull"]
    print(
        f"Gun hull: {hull_data['vertex_count']} vertices, "
        f"{hull_data['face_count']} triangular faces"
    )
    print(f"Workpiece proxies: {len(data['workpiece']['collision_proxies'])}")
    print(f"WeldTCP pose_mount: {data['welding_gun']['weld_tcp']['pose_mount']}")
    print(f"Workpiece pose_base: {data['workpiece']['pose_base']}")


if __name__ == "__main__":
    main()
