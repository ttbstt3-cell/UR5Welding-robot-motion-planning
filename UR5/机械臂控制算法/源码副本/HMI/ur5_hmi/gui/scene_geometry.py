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
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# CoppeliaSim UR5_base -> URDF base vertical offset.  This is the magnitude of
# the Workpiece Z pose in scene_geometry.json; do not hardcode a different one.
Z_OFFSET_M = 0.014649842865765095

DEFAULT_SCENE_JSON = Path("/home/ttbstt/jixiebi/UR5/scene_geometry.json")

WORKPIECE_PROXY_NAMES = (
    "WorkpieceCollision_bottom",
    "WorkpieceCollision_wall_x0",
    "WorkpieceCollision_wall_x1",
    "WorkpieceCollision_wall_y1",
)

# Seam end margin used by the A-ARC-IK+ planner (endMarginM default).
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
