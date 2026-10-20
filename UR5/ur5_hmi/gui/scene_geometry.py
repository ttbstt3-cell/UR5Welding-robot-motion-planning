"""Read-only workpiece / seam geometry for the UR5 HMI 3D view.

Two different source frames must never share one transform:

  * the workpiece boxes come from ``UR5/scene_geometry.json``, which is
    expressed in the CoppeliaSim ``UR5_base`` frame

        p_base = Rz(-90 deg) * p_UR5base + [0, 0, dz]

  * the planned TCP path comes from the MATLAB planner, which is expressed in
    the MATLAB ``base_link`` frame

        p_base = Rz(180 deg) * p_base_link

The HMI VTK widget works in the URDF ``base`` frame, which is also the frame of
``/tcp_pose_broadcaster/pose``, so the live TCP marker needs no conversion.

These two formulas are verified numerically by ``tools/verify_scene_frames.py``
against the validated ``hmi_weld_Weld_Seam`` trajectory: the planned weld TCP
deviates from the seam derived here by 0.0002 mm.
"""
from __future__ import annotations

import json
import math
import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# CoppeliaSim UR5_base -> URDF base vertical offset.  This is the magnitude of
# the Workpiece Z pose in scene_geometry.json; do not hardcode a different one.
Z_OFFSET_M = 0.014649842865765095

DEFAULT_SCENE_JSON = Path("/home/ttbstt/jixiebi/UR5/scene_geometry.json")
DEFAULT_ENVIRONMENT_JSON = DEFAULT_SCENE_JSON.with_name("environment_geometry_real.json")

WORKPIECE_PROXY_NAMES = (
    "WorkpieceCollision_bottom",
    "WorkpieceCollision_wall_x0",
    "WorkpieceCollision_wall_x1",
    "WorkpieceCollision_wall_y1",
)

# Seam end margin used by the ACCP planner (endMarginM default).
SEAM_END_MARGIN_M = 0.025


def rotation_z(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion_to_rotation(q) -> np.ndarray:
    """CoppeliaSim/JSON quaternion order is [x, y, z, w]."""
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def ur5base_to_base(points) -> np.ndarray:
    """CoppeliaSim ``UR5_base`` -> URDF ``base``."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    out = pts @ rotation_z(-90.0).T + np.array([0.0, 0.0, Z_OFFSET_M])
    return out if np.ndim(np.asarray(points)) > 1 else out[0]


def base_link_to_base(points) -> np.ndarray:
    """MATLAB ``base_link`` -> URDF ``base`` (Rz(180 deg), no offset)."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    out = pts @ rotation_z(180.0).T
    return out if np.ndim(np.asarray(points)) > 1 else out[0]


def environment_frame_to_base(frame_id: str) -> np.ndarray:
    """Fixed frame conversion shared with MATLAB; no visual alignment offset."""
    transform = np.eye(4)
    if frame_id == "base":
        return transform
    if frame_id == "base_link":
        transform[:3, :3] = rotation_z(180.0)
        return transform
    # Coppelia exports must first use their explicit calibrated
    # frames.T_base_UR5base in EnvironmentGeometry. No rounded-Z fallback.
    raise ValueError(f"unsupported environment coordinate frame: {frame_id!r}")


def _environment_rigid_transform(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("environment pose must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0)):
        raise ValueError("environment pose must be a proper rigid transform")
    return matrix.copy()


def _environment_pose(value) -> np.ndarray:
    if not isinstance(value, dict):
        return _environment_rigid_transform(value)
    position = np.asarray(value["position"], dtype=float)
    quaternion = np.asarray(value["quaternion"], dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))
            or not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-6, rtol=0)):
        raise ValueError("invalid exported environment pose")
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation(quaternion)
    matrix[:3, 3] = position
    return matrix


@dataclass
class EnvironmentComponent:
    """One exact exported collider; fences stay separate open mesh components."""
    name: str
    category: str
    geometry_type: str
    pose_base: np.ndarray
    size_m: np.ndarray | None = None
    vertices_m: np.ndarray | None = None
    faces: np.ndarray | None = None  # zero-based triangle indices
    radius_m: float | None = None
    length_m: float | None = None

    def mesh_vertices_base(self) -> np.ndarray:
        if self.vertices_m is None:
            raise ValueError("environment component is not a triangle mesh")
        return self.vertices_m @ self.pose_base[:3, :3].T + self.pose_base[:3, 3]


def environment_components(records, frame_id: str = "base") -> list[EnvironmentComponent]:
    """Validate exported/snapshot geometry without inventing missing obstacles."""
    if isinstance(records, dict):
        records = [records]
    if not isinstance(records, list):
        raise ValueError("environment records must be a list")
    result = []
    names = set()
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("environment record must be an object")
        # The existing workpiece layer already displays the same four plates.
        if str(record.get("category", "Environment")).lower() == "workpiece":
            continue
        name = str(record["name"])
        if not name or name in names:
            raise ValueError(f"environment collider name missing/duplicated: {name!r}")
        names.add(name)
        kind = str(record.get("type", record.get("geometry_type", ""))).lower()
        kind = {"cuboid": "box", "convex_mesh": "mesh", "convex_hull": "mesh", "spheroid": "sphere"}.get(kind, kind)
        matrix = record.get("pose_base", record.get("T_base_object"))
        if matrix is None:
            raise ValueError(f"environment object {name} has no explicit pose")
        pose = environment_frame_to_base(str(record.get("frame_id", frame_id))) @ _environment_rigid_transform(matrix)
        component = EnvironmentComponent(name, str(record.get("category", "Environment")), kind, pose)
        if kind == "box":
            size = np.asarray(record["size_m"], dtype=float).reshape(-1)
            if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
                raise ValueError(f"invalid environment box extents: {name}")
            component.size_m = size.copy()
        elif kind == "mesh":
            vertices = np.asarray(record["vertices_m"], dtype=float)
            faces = np.asarray(record["faces"], dtype=float)
            index_base = int(record.get("face_index_base", record.get("faces_index_base", 1)))
            if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
                    or not np.all(np.isfinite(vertices)) or faces.ndim != 2
                    or faces.shape[1] != 3 or not len(faces) or not np.all(np.isfinite(faces))
                    or not np.all(faces == np.floor(faces)) or index_base not in (0, 1)):
                raise ValueError(f"invalid environment triangular mesh: {name}")
            indices = faces.astype(int) - index_base
            if np.any(indices < 0) or np.any(indices >= len(vertices)):
                raise ValueError(f"environment mesh face index out of bounds: {name}")
            component.vertices_m = vertices.copy()
            component.faces = indices
        elif kind in ("cylinder", "sphere"):
            radius = float(record["radius_m"])
            if not np.isfinite(radius) or radius <= 0:
                raise ValueError(f"invalid environment radius: {name}")
            component.radius_m = radius
            if kind == "cylinder":
                length = float(record["length_m"])
                if not np.isfinite(length) or length <= 0:
                    raise ValueError(f"invalid environment cylinder length: {name}")
                component.length_m = length
        else:
            raise ValueError(f"unsupported environment geometry {kind!r}: {name}")
        result.append(component)
    return result


def environment_coverage_warning(coverage) -> str:
    """Report absent source obstacles without inventing geometry for them."""
    if not isinstance(coverage, dict):
        return ""
    missing = []
    for keys, text in ((('floor',), '地面：源场景未找到'),
                       (('safety_fence', 'fence'), '安全栅栏：源场景未找到')):
        values = [coverage[key] for key in keys if key in coverage]
        if values and not any(bool(value) for value in values):
            missing.append(text)
    return "；".join(missing)


class EnvironmentGeometry:
    """Independent static environment manifest, never moved by FK/workpiece pose."""
    def __init__(self, path: Path | str = DEFAULT_ENVIRONMENT_JSON):
        self.path = Path(path)
        content = self.path.read_bytes()
        self.sha256 = hashlib.sha256(content).hexdigest()
        self.data = json.loads(content)
        self.coverage_warning = environment_coverage_warning(self.data.get("scene_coverage", self.data.get("coverage")))
        if str(self.data.get("units", "m")) != "m":
            raise ValueError("environment geometry must use metres")
        frame = str(self.data["frame_id"])
        # Environment uses the exact frozen-adapter transform exported for
        # MATLAB, not the historical workpiece-display rounded Z constant.
        source_to_base = None
        if frame == "UR5_base":
            source_to_base = _environment_rigid_transform(self.data["frames"]["T_base_UR5base"])
        records = []
        for obj in self.data.get("objects", []):
            if not obj.get("collision_enabled", True):
                continue
            if "geometry" not in obj:
                record = dict(obj)
                if source_to_base is not None:
                    record["pose_base"] = source_to_base @ _environment_rigid_transform(obj["pose_base"])
                    record["frame_id"] = "base"
                records.append(record)
                continue
            geometry = obj["geometry"]
            pose = obj.get("pose_UR5_base")
            if pose is None:
                raise ValueError(f"environment object {obj.get('name')} has no pose_UR5_base")
            transform = _environment_pose(pose)
            # Coppelia also exports mesh data for native primitives. MATLAB
            # uses the native collider in this case, so use the same choice.
            native = str(geometry.get("type", "")).lower() in ("box", "cuboid", "cylinder", "sphere")
            compound = bool(geometry.get("components")) and not native
            parts = geometry["components"] if compound else [geometry]
            for i, part in enumerate(parts):
                record = dict(part)
                record.setdefault("type", "mesh" if "vertices_m" in part else geometry.get("type"))
                record["name"] = str(obj["name"]) + (f"/component_{i+1:02d}" if compound else "")
                record["category"] = obj.get("category", "Environment")
                if source_to_base is None:
                    raise ValueError("exported pose_UR5_base requires frames.T_base_UR5base")
                part_pose = _environment_pose(part["pose_object"]) if "pose_object" in part else np.eye(4)
                record["pose_base"] = source_to_base @ transform @ part_pose
                record["frame_id"] = "base"
                records.append(record)
        self.components = environment_components(records, frame)
        if not self.components:
            raise ValueError("environment export contains no enabled collision geometry")


@dataclass
class Box:
    """An oriented collision box expressed in the URDF base frame.

    ``axes_base`` is required: the Workpiece frame is rotated by Rz(-90 deg)
    relative to ``base``, so a Workpiece-x extent of 235 mm appears along
    base -Y.  Transforming only the centre and keeping the sizes axis-aligned
    would silently draw every plate with swapped x/y extents.
    """

    name: str
    size_m: np.ndarray          # full extents in the Workpiece axes, metres
    center_base: np.ndarray     # centre in the URDF base frame
    axes_base: np.ndarray       # 3x3, columns = Workpiece x/y/z axes in base

    def corners_base(self) -> np.ndarray:
        half = self.size_m / 2.0
        signs = np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
            dtype=float,
        )
        local = signs * half                    # (8,3) in Workpiece axes
        return self.center_base + local @ self.axes_base.T

    def faces(self) -> np.ndarray:
        """Indices of the 6 quads over ``corners_base`` (12 triangles)."""
        return np.array([
            [0, 1, 3, 2], [4, 5, 7, 6],
            [0, 1, 5, 4], [2, 3, 7, 6],
            [0, 2, 6, 4], [1, 3, 7, 5],
        ], dtype=int)


@dataclass
class Seam:
    name: str
    label: str
    polyline_base: np.ndarray    # N x 3, URDF base frame
    tangent_wp: np.ndarray
    gun_axis_wp: np.ndarray


class SceneGeometry:
    """Loads scene_geometry.json once and exposes base-frame geometry."""

    def __init__(self, path: Path | str = DEFAULT_SCENE_JSON):
        self.path = Path(path)
        with self.path.open(encoding="utf-8") as handle:
            self.data = json.load(handle)
        wp = self.data["workpiece"]
        self._wp = wp
        pose_base = wp["pose_base"]
        self._wp_position_ur5base = np.asarray(pose_base["position"], dtype=float)
        self._wp_rotation_ur5base = quaternion_to_rotation(pose_base["quaternion"])
        self._proxies = {p["name"]: p for p in wp["collision_proxies"]}
        self.plate_thickness_m = float(wp.get("plate_thickness_m", 0.005))
        self.description = str(wp.get("geometry_description", ""))

    # ---------------------------------------------------------------- boxes
    def workpiece_boxes(self) -> list[Box]:
        """The four real collision boxes, converted to the URDF base frame.

        Sizes and poses are read verbatim from scene_geometry.json; nothing is
        recomputed or re-created here.  Both the centre and the box axes are
        transformed, so the displayed plates keep their true orientation.
        """
        # Workpiece axes expressed in the URDF base frame.  UR5_base -> base is
        # a Rz(-90 deg) rotation, so Workpiece x -> base -y, y -> base +x.
        rot_base = rotation_z(-90.0) @ self._wp_rotation_ur5base
        boxes: list[Box] = []
        for name in WORKPIECE_PROXY_NAMES:
            proxy = self._proxies.get(name)
            if proxy is None:
                continue
            pp = proxy["pose_workpiece"]
            local_pos = np.asarray(pp["position"], dtype=float)
            local_rot = quaternion_to_rotation(pp["quaternion"])
            pos_ur5base = self._wp_rotation_ur5base @ local_pos + self._wp_position_ur5base
            axes_base = rot_base @ local_rot
            boxes.append(
                Box(
                    name,
                    np.asarray(proxy["size_m"], dtype=float),
                    ur5base_to_base(pos_ur5base),
                    axes_base,
                )
            )
        return boxes

    def workpiece_bbox_base(self) -> tuple[np.ndarray, np.ndarray]:
        corners = np.vstack([box.corners_base() for box in self.workpiece_boxes()])
        return corners.min(axis=0), corners.max(axis=0)

    def workpiece_pose_base(self) -> np.ndarray:
        """Return the nominal workpiece frame pose in the URDF ``base`` frame.

        The returned transform is derived from the same ``pose_base`` and
        UR5_base-to-base conversion used by :meth:`workpiece_boxes`.  It is
        only a display/calibration reference; it does not alter the source
        scene geometry or any robot coordinate frame.
        """
        rot_base = rotation_z(-90.0) @ self._wp_rotation_ur5base
        position_base = ur5base_to_base(self._wp_position_ur5base)
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rot_base
        transform[:3, 3] = position_base
        return transform

    # ---------------------------------------------------------------- seams
    def _proxy(self, name: str):
        proxy = self._proxies.get(name)
        if proxy is None:
            raise KeyError(f"workpiece proxy not found in {self.path}: {name}")
        pp = proxy["pose_workpiece"]
        return (
            np.asarray(pp["position"], dtype=float),
            quaternion_to_rotation(pp["quaternion"]),
            np.asarray(proxy["size_m"], dtype=float),
        )

    def seam(self, name: str = "A", end_margin_m: float = SEAM_END_MARGIN_M) -> Seam:
        """Build a seam centre line in the URDF base frame.

        Seam definitions mirror replan_weld_actual_axis_adaptive_core.localInferSeam:
          A = bottom x wall_y1, gun axis [0, +1, -1]/sqrt(2)
          B = bottom x wall_x0, gun axis [-1, 0, -1]/sqrt(2)
          C = bottom x wall_x1, gun axis [+1, 0, -1]/sqrt(2)
        """
        key = str(name).strip().upper()
        b_pos, _, b_size = self._proxy("WorkpieceCollision_bottom")
        x0_pos, _, x0_size = self._proxy("WorkpieceCollision_wall_x0")
        x1_pos, _, x1_size = self._proxy("WorkpieceCollision_wall_x1")
        y1_pos, _, y1_size = self._proxy("WorkpieceCollision_wall_y1")

        bottom_min = b_pos - b_size / 2.0
        bottom_max = b_pos + b_size / 2.0
        x_min = max(bottom_min[0], x0_pos[0] + x0_size[0] / 2.0)
        x_max = min(bottom_max[0], x1_pos[0] - x1_size[0] / 2.0)
        # Seam A runs along +X between the two x walls.  Seams B and C run along
        # +Y from the open side (y=0 per geometry_description) to the inner
        # face of wall_y1, matching the Seam B/C definitions in the handoff.
        y_open = bottom_min[1]
        y_seam = y1_pos[1] - y1_size[1] / 2.0
        z_seam = bottom_max[2]
        y_start = y_open + end_margin_m
        y_end = y_seam - end_margin_m

        if key == "A":
            p0 = np.array([x_min + end_margin_m, y_seam, z_seam])
            p1 = np.array([x_max - end_margin_m, y_seam, z_seam])
            axis = np.array([0.0, 1.0, -1.0]) / math.sqrt(2.0)
            label = "Seam A  (bottom x wall_y1)"
        elif key == "B":
            p0 = np.array([x_min, y_start, z_seam])
            p1 = np.array([x_min, y_end, z_seam])
            axis = np.array([-1.0, 0.0, -1.0]) / math.sqrt(2.0)
            label = "Seam B  (bottom x wall_x0)"
        elif key == "C":
            p0 = np.array([x_max, y_start, z_seam])
            p1 = np.array([x_max, y_end, z_seam])
            axis = np.array([1.0, 0.0, -1.0]) / math.sqrt(2.0)
            label = "Seam C  (bottom x wall_x1)"
        else:
            raise ValueError(f"unknown seam {name!r}; expected A, B or C")

        wp_points = np.vstack([p0, p1])
        ur5base = wp_points + self._wp_position_ur5base
        polyline = np.vstack([ur5base_to_base(p) for p in ur5base])
        tangent = p1 - p0
        tangent = tangent / max(float(np.linalg.norm(tangent)), 1e-9)
        return Seam(key, label, polyline, tangent, axis)


_CACHED: SceneGeometry | None = None


def load_scene_geometry(path: Path | str = DEFAULT_SCENE_JSON, refresh: bool = False) -> SceneGeometry:
    global _CACHED
    if _CACHED is None or refresh:
        _CACHED = SceneGeometry(path)
    return _CACHED
