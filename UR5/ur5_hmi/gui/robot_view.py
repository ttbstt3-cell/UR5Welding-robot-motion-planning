from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget

from safety.joint_limits import JOINT_NAMES
from gui.preview_display import PreviewDisplayMixin
from gui.scene_geometry import EnvironmentGeometry, DEFAULT_ENVIRONMENT_JSON


URDF_PATH = Path(__file__).resolve().parents[2] / "real_robot_description_calibrated.urdf"
LIVE_TCP_MAX_AGE_SEC = 1.5


def _validated_tcp_transform(value) -> np.ndarray:
    """Validate a flange-local TCP transform without silently repairing it."""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("planning TCP transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8, rtol=0.0):
        raise ValueError("planning TCP transform must be homogeneous")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0.0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0.0)):
        raise ValueError("planning TCP transform must be a proper rigid transform")
    return matrix.copy()


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _axis_rotation(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0 or abs(angle) < 1e-14:
        return np.eye(4)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    out = np.eye(4)
    out[:3, :3] = np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])
    return out


def _origin(element) -> np.ndarray:
    origin = element.find("origin")
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
    out = np.eye(4)
    out[:3, :3] = _rpy_matrix(*rpy)
    out[:3, 3] = xyz
    return out


class UrdfKinematicModel:
    """Small URDF FK reader for a read-only visualization.

    It uses the actual calibrated URDF joint origins and axes.  It is not used
    for control, IK, collision checking, or trajectory generation.
    """

    def __init__(self, urdf_path: Path = URDF_PATH, urdf_text: str | None = None):
        root = ET.fromstring(urdf_text) if urdf_text else ET.parse(str(urdf_path)).getroot()
        self.children = {}
        self.joints = {}
        self.link_meshes = {}
        for link in root.findall("link"):
            collision = link.find("collision")
            visual = link.find("visual")
            mesh_element = None
            origin_element = None
            if collision is not None:
                geometry = collision.find("geometry")
                mesh_element = None if geometry is None else geometry.find("mesh")
                origin_element = collision
            if mesh_element is None and visual is not None:
                geometry = visual.find("geometry")
                mesh_element = None if geometry is None else geometry.find("mesh")
                origin_element = visual
            if mesh_element is not None:
                filename = mesh_element.attrib.get("filename", "")
                self.link_meshes[link.attrib["name"]] = {
                    "filename": filename,
                    "origin": _origin(origin_element) if origin_element is not None else np.eye(4),
                }
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            axis_element = joint.find("axis")
            axis_text = "0 0 1" if axis_element is None else axis_element.attrib.get("xyz", "0 0 1")
            record = {
                "name": joint.attrib["name"],
                "type": joint.attrib.get("type", "fixed"),
                "parent": parent.attrib["link"],
                "child": child.attrib["link"],
                "origin": _origin(joint),
                "axis": [float(v) for v in axis_text.split()],
            }
            self.children.setdefault(record["parent"], []).append(record)
            self.joints[record["name"]] = record
        self.revolute_names = [name for name in JOINT_NAMES if name in self.joints]
        self.link_order = [
            "base", "base_link", "base_link_inertia", "shoulder_link",
            "upper_arm_link", "forearm_link", "wrist_1_link",
            "wrist_2_link", "wrist_3_link", "flange", "tool0",
        ]

    def _walk(self, parent: str, transform: np.ndarray, q_by_name: dict, frames: dict) -> None:
        for joint in self.children.get(parent, []):
            T = transform @ joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                T = T @ _axis_rotation(joint["axis"], q_by_name.get(joint["name"], 0.0))
            frames[joint["child"]] = T
            self._walk(joint["child"], T, q_by_name, frames)

    def link_points_in_base(self, q) -> tuple[np.ndarray, list[str]]:
        transforms = self.link_transforms_in_base(q)
        points = []
        labels = []
        for link in self.link_order:
            if link in transforms:
                points.append(transforms[link][:3, 3])
                labels.append(link)
        return np.asarray(points), labels

    def link_transforms_in_base(self, q) -> dict[str, np.ndarray]:
        q_by_name = {name: float(value) for name, value in zip(self.revolute_names, q)}
        frames = {"world": np.eye(4)}
        self._walk("world", np.eye(4), q_by_name, frames)
        if "base" not in frames:
            raise RuntimeError("URDF does not contain base frame")
        T_world_base = frames["base"]
        T_base_world = np.linalg.inv(T_world_base)
        return {link: T_base_world @ transform for link, transform in frames.items()}


class Robot3DView(QWidget, PreviewDisplayMixin):
    """Read-only 10 Hz UR5 skeleton view embedded in the HMI."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model = UrdfKinematicModel()
        # Keep a complete reference pose visible before/without live feedback.
        self.q = [0.0] * 6
        self._live_q = None
        self._preview_q = None
        self.preview_active = False
        self.live_feedback = False
        self._live_tcp_topic = None
        self._live_tcp_topic_stamp = 0.0
        self._live_tcp_rtde = None
        self._live_tcp_rtde_stamp = 0.0
        self._live_tcp_source = "unavailable"
        self._live_tcp_fk = None
        self._preview_tcp_fk = None
        self._preview_tcp_index = None
        self._preview_tcp_path_errors_mm = []
        self._preview_relative_errors_mm = []
        self._preview_relative_orientation_errors_deg = []
        self.tcp = None
        self.tcp_frame_id = ""
        self._calibrated_tcp_offset = None
        self._tcp_offset_flange = None
        self._tcp_rotation_flange = np.eye(3, dtype=float)
        self._planning_tcp_transform = None
        self._gun_mount_roll_deg = 90.0
        self._gun_mount_translation_mm = np.zeros(3, dtype=float)
        self._gun_mount_adjustment_status = "DEFAULT"
        self.canvas = FigureCanvas(Figure(figsize=(5.2, 4.2)))
        self.ax = self.canvas.figure.add_subplot(111, projection="3d")
        # Read-only scene layers mirroring the VTK view.
        self._scene = None
        self._scene_status = "not loaded"
        self._workpiece_calibration = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._planned_tcp_points = None
        self._environment_path = DEFAULT_ENVIRONMENT_JSON
        self._environment_geometry = None
        self._environment_status = "未加载"
        self._environment_file_binding = None
        self._environment_last_check = 0.0
        self._visibility = {
            "robot": True,
            "workpiece": True,
            "seam": True,
            "tcp_path": False,
            "current_tcp": True,
            "preview_tcp": True,
            "gun": True,
            "collision": True,
            "environment": True,
        }
        try:
            from gui.scene_geometry import SceneGeometry
            self._scene = SceneGeometry()
            self._scene_status = "loaded"
        except Exception as exc:  # missing file / malformed json
            self._scene = None
            self._scene_status = f"unavailable: {exc}"
        self.reload_environment_geometry()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def _fk_tcp_from_flange(self, q):
        if self._tcp_offset_flange is None or q is None or len(q) != 6:
            return None, None
        transforms = self.model.link_transforms_in_base(q)
        flange = transforms.get("flange")
        if flange is None:
            return None, None
        return flange, (flange @ np.r_[self._tcp_offset_flange, 1.0])[:3]

    def _select_live_tcp(self) -> None:
        if self._live_q is None:
            self._live_tcp_fk = None
            self._live_tcp_source = "unavailable"
            self.tcp = None
            return
        topic = None if self._live_tcp_topic is None else np.asarray(self._live_tcp_topic, dtype=float)
        age = (time.monotonic() - float(self._live_tcp_topic_stamp)
               if self._live_tcp_topic_stamp > 0 else float("inf"))
        rtde = None if self._live_tcp_rtde is None else np.asarray(self._live_tcp_rtde, dtype=float)
        rtde_age = (time.monotonic() - float(self._live_tcp_rtde_stamp)
                    if self._live_tcp_rtde_stamp > 0 else float("inf"))
        frame = str(self.tcp_frame_id or "base").strip().lower()
        if (topic is not None and topic.shape == (3,) and np.all(np.isfinite(topic))
                and age <= LIVE_TCP_MAX_AGE_SEC and frame in ("", "base")):
            self._live_tcp_fk = topic.copy()
            self._live_tcp_source = "controller tcp_pose_broadcaster/pose"
        elif (topic is not None and topic.shape == (3,) and np.all(np.isfinite(topic))
              and age <= LIVE_TCP_MAX_AGE_SEC and frame == "base_link"):
            from gui.scene_geometry import base_link_to_base
            self._live_tcp_fk = base_link_to_base(topic)
            self._live_tcp_source = "controller tcp_pose_broadcaster/pose (base_link→base)"
        elif (rtde is not None and rtde.shape == (3,) and np.all(np.isfinite(rtde))
              and rtde_age <= LIVE_TCP_MAX_AGE_SEC):
            self._live_tcp_fk = rtde.copy()
            self._live_tcp_source = "controller RTDE actual_TCP_pose"
        else:
            self._live_tcp_fk = self._fk_tcp_from_flange(self._live_q)[1]
            self._live_tcp_source = "HMI flange FK fallback (controller TCP stale/unavailable)"
        self.tcp = None if self._live_tcp_fk is None else self._live_tcp_fk.tolist()

    def set_controller_tcp_pose(self, tcp_position, q=None, tcp_frame_id="base") -> None:
        """Store a fresh read-only controller TCP without touching preview."""
        pose = np.asarray(tcp_position, dtype=float).reshape(-1)
        if pose.size != 3 or not np.all(np.isfinite(pose)):
            return
        if q is not None and len(q) == 6:
            self._live_q = list(q)
            self.live_feedback = True
            if not self.preview_active:
                self.q = list(self._live_q)
        self._live_tcp_rtde = pose.copy()
        self._live_tcp_rtde_stamp = time.monotonic()
        self.tcp_frame_id = str(tcp_frame_id or "base")
        self._select_live_tcp()
        self._refresh()

    def set_live_state(self, q, tcp_position=None, tcp_frame_id="", tcp_stamp=0.0) -> None:
        if q is not None and len(q) == 6:
            self._live_q = list(q)
            self.live_feedback = True
            self._live_tcp_topic_stamp = float(tcp_stamp or 0.0)
        else:
            self.live_feedback = False
            self._live_q = None
            self._live_tcp_fk = None
            self._live_tcp_topic_stamp = 0.0
            self._live_tcp_source = "unavailable"
            self.tcp = None
        self._live_tcp_topic = None if tcp_position is None else list(tcp_position)
        self.tcp_frame_id = str(tcp_frame_id or "")
        self._select_live_tcp()
        if not self.preview_active and self._live_q is not None:
            self.q = list(self._live_q)

    def set_state(self, q, tcp_position=None, tcp_frame_id="", tcp_stamp=0.0) -> None:
        self.set_live_state(q, tcp_position, tcp_frame_id, tcp_stamp)

    def set_calibrated_tcp_offset(self, offset_flange_m) -> None:
        """Show a separate calibrated flange-FK TCP annotation in fallback view."""
        offset = np.asarray(offset_flange_m, dtype=float).reshape(-1)
        if offset.size != 3 or not np.all(np.isfinite(offset)):
            raise ValueError("calibrated TCP offset must be a finite 3-vector")
        self._calibrated_tcp_offset = offset.copy()
        self._tcp_offset_flange = offset.copy()
        self._select_live_tcp()
        self._refresh()

    set_display_tcp_offset = set_calibrated_tcp_offset

    def set_planning_tcp_transform(self, transform) -> None:
        """Mirror the confirmed controller TCP; never alter live feedback."""
        matrix = _validated_tcp_transform(transform)
        self._planning_tcp_transform = matrix
        self._tcp_rotation_flange = matrix[:3, :3].copy()
        self._calibrated_tcp_offset = matrix[:3, 3].copy()
        self._tcp_offset_flange = matrix[:3, 3].copy()
        if self._preview_q is not None:
            self._preview_tcp_fk = self._preview_tcp_from_snapshot()
        self._refresh()

    def clear_calibrated_tcp_offset(self) -> None:
        self._calibrated_tcp_offset = None
        self._tcp_offset_flange = None
        self._planning_tcp_transform = None
        self._tcp_rotation_flange = np.eye(3, dtype=float)
        self._select_live_tcp()
        self._preview_tcp_fk = None
        self._refresh()

    def set_preview_state(self, q, tcp_index: int | None = None) -> None:
        """Display a local preview pose without presenting it as live TCP data."""
        if q is not None and len(q) == 6:
            self._preview_q = list(q)
            self.preview_active = True
            self._preview_tcp_index = tcp_index
            self._preview_tcp_fk = self._preview_tcp_from_snapshot()
            if tcp_index is not None and self._preview_tcp_fk is not None:
                point = self._planned_tcp_raw_point_base(tcp_index)
                if point is not None:
                    self._preview_tcp_path_errors_mm.append(float(np.linalg.norm(self._preview_tcp_fk - point) * 1000.0))
            self._check_preview_relative_transform()
            self.q = list(self._preview_q)
            self._refresh()

    def clear_preview_state(self) -> None:
        self.preview_active = False
        self._preview_q = None
        self._preview_tcp_fk = None
        self._preview_tcp_index = None
        self._preview_tcp_path_errors_mm = []
        self._preview_relative_errors_mm = []
        self._preview_relative_orientation_errors_deg = []
        if self._live_q is not None:
            self.q = list(self._live_q)

    def _planned_tcp_raw_point_base(self, index: int):
        if self._planned_tcp_points is None or len(self._planned_tcp_points) == 0:
            return None
        idx = int(np.clip(index, 0, len(self._planned_tcp_points) - 1))
        return np.asarray(self._planned_tcp_points[idx], dtype=float)

    def _check_preview_relative_transform(self) -> None:
        if self._preview_q is None or self._tcp_offset_flange is None or self._preview_tcp_fk is None:
            return
        try:
            flange = self.model.link_transforms_in_base(self._preview_q)["flange"]
            expected = self._tcp_offset_flange
            expected_rotation = self._tcp_rotation_flange
            snapshot = getattr(self, "_preview_scene_snapshot", None)
            if snapshot is not None:
                expected = snapshot["T_flange_to_tcp"][:3,3]
                expected_rotation = snapshot["T_flange_to_tcp"][:3,:3]
            actual = (np.linalg.inv(flange) @ np.r_[self._preview_tcp_fk, 1.0])[:3]
            self._preview_relative_errors_mm.append(float(np.linalg.norm(actual - expected) * 1000.0))
            actual_rotation = flange[:3, :3].T @ (flange[:3, :3] @ expected_rotation)
            error_rotation = actual_rotation @ expected_rotation.T
            cosine = np.clip((np.trace(error_rotation) - 1.0) * 0.5, -1.0, 1.0)
            self._preview_relative_orientation_errors_deg.append(float(np.degrees(np.arccos(cosine))))
        except Exception:
            pass

    def tcp_preview_diagnostics(self) -> dict:
        path = np.asarray(self._preview_tcp_path_errors_mm, dtype=float)
        rel = np.asarray(self._preview_relative_errors_mm, dtype=float)
        rel_rot = np.asarray(self._preview_relative_orientation_errors_deg, dtype=float)
        return {
            "frame_count": int(max(path.size, rel.size)),
            "max_tcp_path_error_mm": float(np.max(path)) if path.size else None,
            "max_flange_tcp_relative_error_mm": float(np.max(rel)) if rel.size else None,
            "max_flange_tcp_relative_orientation_error_deg": float(np.max(rel_rot)) if rel_rot.size else 0.0,
            "tcp_offset_flange_m": None if self._tcp_offset_flange is None else self._tcp_offset_flange.tolist(),
            "offset_source": "fixed display calibration" if self._tcp_offset_flange is not None else "unavailable",
        }

    # --- read-only scene layers (mirror of the VTK widget) -----------------
    def set_planned_tcp_path(self, points, frame: str = "base_link", follow_workpiece: bool = True) -> None:
        self._planned_path_follow_workpiece = follow_workpiece
        if points is None:
            self._planned_tcp_points = None
            self._preview_tcp_path_errors_mm = []
            self._preview_relative_errors_mm = []
            return
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            self._planned_tcp_points = None
            self._preview_tcp_path_errors_mm = []
            self._preview_relative_errors_mm = []
            return
        if frame == "base_link":
            from gui.scene_geometry import base_link_to_base
            pts = base_link_to_base(pts)
        self._planned_tcp_points = pts
        self._preview_tcp_path_errors_mm = []
        self._preview_relative_errors_mm = []

    def clear_planned_tcp_path(self) -> None:
        self._planned_tcp_points = None

    def scene_status(self) -> str:
        environment = self.environment_display_diagnostics()
        return (f"{self._scene_status} | 工件位姿: {self._workpiece_pose_source}"
                f" | 环境: {environment['object_count']} 组件 {environment['status']}")

    def reload_environment_geometry(self, path=None):
        if path is not None:
            self._environment_path = Path(path)
        try:
            stat = self._environment_path.stat()
            self._environment_file_binding = (stat.st_mtime_ns, stat.st_size)
            self._environment_geometry = EnvironmentGeometry(self._environment_path)
            self._environment_status = "已加载"
            if self._environment_geometry.coverage_warning:
                self._environment_status += " WARNING: " + self._environment_geometry.coverage_warning
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._environment_geometry = None
            self._environment_status = "WARNING: " + str(exc)
        return self.environment_display_diagnostics()

    def environment_display_diagnostics(self):
        preview = self.preview_active
        components = (getattr(self, "_preview_environment_components", []) if preview else
                      (self._environment_geometry.components if self._environment_geometry else []))
        return {"frame_id": "base", "source": "preview_snapshot" if preview else str(self._environment_path),
                "status": (getattr(self, "_preview_environment_error", "") +
                           getattr(self, "_preview_environment_coverage_warning", "")) if preview else self._environment_status,
                "object_count": len(components),
                "objects": [{"name": c.name, "pose_base": c.pose_base.tolist(), "type": c.geometry_type} for c in components]}

    def _check_environment_binding(self):
        now = time.monotonic()
        if now - self._environment_last_check < 1.0:
            return
        self._environment_last_check = now
        try:
            stat = self._environment_path.stat()
            binding = (stat.st_mtime_ns, stat.st_size)
        except OSError:
            binding = None
        if binding != self._environment_file_binding:
            self.reload_environment_geometry()
            self._environment_file_binding = binding

    def set_gun_mount_adjustment(self, roll_deg: float, translation_mm=(0.0, 0.0, 0.0), render: bool = True) -> dict:
        roll = float(roll_deg)
        translation = np.asarray(translation_mm, dtype=float).reshape(-1)
        if not np.isfinite(roll) or roll < -180 or roll > 180 or translation.size != 3 or not np.all(np.isfinite(translation)):
            raise ValueError("invalid flange-local gun adjustment")
        self._gun_mount_roll_deg = roll
        self._gun_mount_translation_mm = translation.copy()
        self._gun_mount_adjustment_status = "PREVIEW"
        if render:
            self._refresh()
        return self.gun_mount_adjustment()

    def gun_mount_adjustment(self) -> dict:
        return {
            "roll_deg": float(self._gun_mount_roll_deg),
            "translation_mm": self._gun_mount_translation_mm.tolist(),
            "rotation_axis_flange": [1.0, 0.0, 0.0],
            "coordinate_frame": "URDF flange",
            "status": self._gun_mount_adjustment_status,
        }

    def set_gun_mount_calibration_status(self, status: str) -> None:
        self._gun_mount_adjustment_status = str(status).upper()

    def gun_mount_status(self) -> str:
        return f"J6轴 [+1.000, +0.000, +0.000]；角度 {self._gun_mount_roll_deg:+.1f}°；{self._gun_mount_adjustment_status}"

    def gun_mount_wire_diagnostics(self) -> dict:
        return {"available": False, "reason": "Matplotlib fallback does not render gun/wire geometry"}

    def set_workpiece_calibration(self, transform) -> None:
        """Store the display calibration for the non-VTK fallback view.

        The normal HMI uses VTK for the scene.  Keeping the same method on the
        Matplotlib fallback makes calibration smoke tests and headless startup
        safe without changing robot or source scene coordinates.
        """
        matrix = np.asarray(transform, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("workpiece calibration transform must be finite 4x4")
        self._workpiece_calibration = matrix.copy()
        self._workpiece_pose_source = "REAL CALIBRATION"
        self._refresh()

    def set_workpiece_calibration_preview(self, transform) -> None:
        """Show a saved real pose for diagnostics without enabling planning.

        A draft/poor calibration may be useful to inspect visually, but it
        must not be confused with an approved REAL CALIBRATION scene source.
        """
        matrix = np.asarray(transform, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("workpiece calibration preview must be finite 4x4")
        self._workpiece_calibration = matrix.copy()
        self._workpiece_pose_source = "REAL CALIBRATION PREVIEW"
        self._refresh()

    def clear_workpiece_calibration(self) -> None:
        self._workpiece_calibration = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._refresh()

    def _transform_workpiece_points(self, points):
        pts = np.asarray(points, dtype=float)
        shape = pts.shape
        flat = pts.reshape(-1, 3)
        hom = np.column_stack((flat, np.ones(len(flat))))
        return (hom @ self._workpiece_calibration.T)[:, :3].reshape(shape)

    def set_visibility(self, **flags) -> None:
        for key, value in flags.items():
            if key in self._visibility:
                self._visibility[key] = bool(value)

    def visibility(self) -> dict:
        return dict(self._visibility)

    def set_view(self, view_name: str) -> None:
        presets = {"top": (89, -90), "front": (0, -90), "side": (0, 0), "reset": (25, 135)}
        elev, azim = presets.get(view_name, (25, 135))
        self._view = (elev, azim)
        self._refresh()

    def set_view_with_scene(self, view_name: str) -> None:
        self.set_view(view_name)

    def fit_robot_view(self, view_name: str = "iso") -> None:
        self.set_view(view_name)

    def fit_scene_view(self, view_name: str = "iso") -> None:
        # Matplotlib fallback has no separate camera bounds API; preserve the
        # same public interface as the VTK view and keep the scene read-only.
        self.set_view(view_name)

    def fit_gun_view(self, view_name: str = "iso") -> None:
        # Keep the same public API as the VTK view for headless/offscreen UI
        # tests; the fallback has no separate gun mesh bounds.
        self.set_view(view_name)

    def preview_point_base(self, index: int):
        if self._planned_tcp_points is None or len(self._planned_tcp_points) == 0:
            return None
        idx = int(np.clip(index, 0, len(self._planned_tcp_points) - 1))
        return self._transform_workpiece_points(self._planned_tcp_points[idx])

    def set_urdf_text(self, urdf_text: str) -> None:
        try:
            self.model = UrdfKinematicModel(urdf_text=urdf_text)
        except Exception:
            # Keep the known calibrated local URDF as a safe display fallback.
            pass

    def _refresh(self) -> None:
        self._check_environment_binding()
        display_q = self._preview_q if self.preview_active else self._live_q
        if display_q is None or len(display_q) != 6:
            return
        # During headless/startup layout the canvas can temporarily be 0x0;
        # avoid Matplotlib divide-by-zero warnings until the widget is visible.
        if self.width() < 10 or self.height() < 10:
            return
        try:
            points, labels = self.model.link_points_in_base(display_q)
        except Exception:
            return
        ax = self.ax
        ax.clear()
        if self._visibility.get("environment", True):
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            components = (getattr(self, "_preview_environment_components", []) if self.preview_active else
                          (self._environment_geometry.components if self._environment_geometry else []))
            for component in components:
                if component.geometry_type == "mesh":
                    vertices = component.mesh_vertices_base()
                    ax.add_collection3d(Poly3DCollection(vertices[component.faces], alpha=.2, facecolor="#789faf"))
                elif component.geometry_type == "box":
                    from gui.scene_geometry import Box
                    box = Box(component.name, component.size_m, component.pose_base[:3,3], component.pose_base[:3,:3])
                    vertices = box.corners_base()
                    ax.add_collection3d(Poly3DCollection(vertices[box.faces()], alpha=.2, facecolor="#789faf"))
                else:
                    theta = np.linspace(0, 2*np.pi, 24)
                    if component.geometry_type == "cylinder":
                        theta, z = np.meshgrid(theta, [-component.length_m/2, component.length_m/2])
                        local = np.stack([component.radius_m*np.cos(theta), component.radius_m*np.sin(theta), z], axis=-1)
                    else:
                        theta, phi = np.meshgrid(theta, np.linspace(0, np.pi, 16))
                        local = component.radius_m*np.stack([np.cos(theta)*np.sin(phi), np.sin(theta)*np.sin(phi), np.cos(phi)], axis=-1)
                    world = local @ component.pose_base[:3,:3].T + component.pose_base[:3,3]
                    ax.plot_surface(world[:,:,0], world[:,:,1], world[:,:,2], color="#789faf", alpha=.2)
        # ax.clear() wipes every previous artist, so all scene overlays are
        # rebuilt here on every refresh instead of being added once.
        if self.preview_active and getattr(self, "_preview_scene_snapshot", None) is not None:
            d = self.preview_frame_diagnostics()
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            from scipy.spatial import ConvexHull
            if self._visibility.get("gun", True):
                for component in self._preview_scene_snapshot.get("components", []):
                    local = np.asarray(component["vertices_m"])
                    world = (d["gun_pose"] @ np.c_[local, np.ones(len(local))].T).T[:,:3]
                    ax.add_collection3d(Poly3DCollection(world[ConvexHull(local).simplices], alpha=.2, facecolor="orange"))
            if self._visibility.get("wire", True):
                a,b = d["gun_tip_world"],d["wire_tip_world"]
                ax.plot([a[0],b[0]],[a[1],b[1]],[a[2],b[2]],color="yellow",linewidth=2)
        if self._scene is not None and self._visibility.get("workpiece", True):
            opacity = 0.32 if self._visibility.get("collision", True) else 0.12
            for box in self._scene.workpiece_boxes():
                corners = self._transform_workpiece_points(box.corners_base())
                for face in box.faces():
                    quad = corners[list(face)]
                    ax.plot(
                        quad[:, 0], quad[:, 1], quad[:, 2],
                        color="#9ea3a8", linewidth=1.0, alpha=opacity,
                    )
            corners = self._transform_workpiece_points(
                np.vstack([box.corners_base() for box in self._scene.workpiece_boxes()])
            )
            lo, hi = corners.min(axis=0), corners.max(axis=0)
            ax.plot(
                [lo[0], hi[0], hi[0], lo[0], lo[0]],
                [lo[1], lo[1], hi[1], hi[1], lo[1]],
                [lo[2], lo[2], lo[2], lo[2], lo[2]],
                color="#9ea3a8", linewidth=1.0, alpha=opacity,
            )
        if self._scene is not None and self._visibility.get("seam", True):
            try:
                seam = self._scene.seam("A")
                poly = self._transform_workpiece_points(seam.polyline_base)
                ax.plot(poly[:, 0], poly[:, 1], poly[:, 2], "-", color="#ffd91a", linewidth=4)
            except Exception:
                pass
        if self._scene is not None and self._visibility.get("workpiece", True):
            try:
                nominal = self._scene.workpiece_pose_base()
                origin = self._transform_workpiece_points(nominal[:3, 3])
                ax.scatter([origin[0]], [origin[1]], [origin[2]], color="#ff26bf", s=65)
                ax.text(origin[0], origin[1], origin[2], "工件原点", color="#ff8bdd", fontsize=8)
            except Exception:
                pass
        if self._planned_tcp_points is not None and self._visibility.get("tcp_path", True):
            path = (self._transform_workpiece_points(self._planned_tcp_points)
                    if getattr(self, "_planned_path_follow_workpiece", True) else self._planned_tcp_points)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], "--", color="#26f269", linewidth=2)
            ax.scatter(*path[0], color="#26f269", s=25)
            ax.scatter(*path[-1], color="#66aaff", s=25)
            ax.text(*path[0], "Start", color="#26f269", fontsize=8)
            ax.text(*path[-1], "End", color="#66aaff", fontsize=8)
        if self._visibility.get("robot", True):
            ax.plot(points[:, 0], points[:, 1], points[:, 2], "-o", color="#2f6db0", linewidth=4, markersize=5)
        # Independent live (red) and preview (blue) TCP markers.  Both are
        # calculated from their own q snapshot and the same fixed flange
        # offset; the ROS TCP topic is not reused for preview.
        try:
            live_frames = self.model.link_transforms_in_base(self._live_q) if self._live_q is not None else None
            if live_frames is not None and self._live_tcp_fk is not None and self._visibility.get("current_tcp", True):
                p = np.asarray(self._live_tcp_fk)
                f = live_frames["flange"][:3, 3]
                ax.plot([f[0], p[0]], [f[1], p[1]], [f[2], p[2]], color="#d62728", linewidth=1.5)
                ax.scatter([p[0]], [p[1]], [p[2]], color="#d62728", s=65)
                ax.text(p[0], p[1], p[2], "TCP LIVE", color="#d62728", fontsize=8)
            if self.preview_active and self._preview_tcp_fk is not None and self._visibility.get("preview_tcp", True):
                preview_frames = self.model.link_transforms_in_base(self._preview_q)
                p = np.asarray(self._preview_tcp_fk)
                f = preview_frames["flange"][:3, 3]
                ax.plot([f[0], p[0]], [f[1], p[1]], [f[2], p[2]], color="#3366ff", linewidth=1.5)
                ax.scatter([p[0]], [p[1]], [p[2]], color="#3366ff", s=55)
                ax.text(p[0], p[1], p[2], "TCP PREVIEW", color="#3366ff", fontsize=8)
        except Exception:
            pass
        joint_links = {
            "shoulder_link": "J1",
            "upper_arm_link": "J2",
            "forearm_link": "J3",
            "wrist_1_link": "J4",
            "wrist_2_link": "J5",
            "wrist_3_link": "J6",
        }
        for p, label in zip(points, labels):
            if label in joint_links:
                ax.text(p[0], p[1], p[2], joint_links[label], fontsize=7)
        # Base-frame axes, so TCP and skeleton use the same reference frame.
        origin = np.zeros(3)
        L = 0.15
        ax.quiver(*origin, L, 0, 0, color="r", linewidth=1.5)
        ax.quiver(*origin, 0, L, 0, color="g", linewidth=1.5)
        ax.quiver(*origin, 0, 0, L, color="b", linewidth=1.5)
        # Match the compact VTK axis captions; these are labels only and do
        # not alter the reference-frame vectors or their positions.
        ax.text(L, 0, 0, "X", fontsize=8, fontweight="normal")
        ax.text(0, L, 0, "Y", fontsize=8, fontweight="normal")
        ax.text(0, 0, L, "Z", fontsize=8, fontweight="normal")
        ax.set_xlabel("base X (m)")
        ax.set_ylabel("base Y (m)")
        ax.set_zlabel("base Z (m)")
        # Keep the fallback camera consistent with VTK: fit only the robot
        # skeleton and optional workpiece bounds.  Text, axes, TCP marker and
        # planned path do not expand the framing, so the model remains large
        # and readable in offscreen/CI sessions as well.
        fit_sets = []
        if self._visibility.get("robot", True):
            fit_sets.append(np.asarray(points, dtype=float))
        if self._scene is not None and self._visibility.get("workpiece", True):
            try:
                fit_sets.append(self._transform_workpiece_points(
                    np.vstack([box.corners_base() for box in self._scene.workpiece_boxes()])
                ))
            except Exception:
                pass
        if fit_sets:
            fit = np.vstack(fit_sets)
            lo = fit.min(axis=0)
            hi = fit.max(axis=0)
            center = 0.5 * (lo + hi)
            half = max(float(np.max(hi - lo)) * 0.68, 0.20)
            ax.set_xlim(center[0] - half, center[0] + half)
            ax.set_ylim(center[1] - half, center[1] + half)
            ax.set_zlim(center[2] - half, center[2] + half)
        else:
            ax.set_xlim(-1.0, 1.0)
            ax.set_ylim(-1.0, 1.0)
            ax.set_zlim(-0.15, 1.35)
        try:
            ax.set_box_aspect((1.0, 1.0, 0.9))
        except AttributeError:
            pass

        ax.grid(True)
        ax.set_title("UR5 real-time skeleton (base frame)")
        view = getattr(self, "_view", (25, 135))
        ax.view_init(elev=view[0], azim=view[1])
        self.canvas.draw_idle()
