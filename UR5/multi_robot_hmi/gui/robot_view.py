from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget



URDF_PATH = Path(__file__).resolve().parents[2] / "real_robot_description_calibrated.urdf"


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

    def __init__(self, urdf_path: Path = URDF_PATH, urdf_text: str | None = None,
                 joint_names=None):
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
                    "scale": [float(v) for v in mesh_element.attrib.get("scale", "1 1 1").split()],
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
        revolute_order = [j.attrib["name"] for j in root.findall("joint")
                          if j.attrib.get("type") in ("revolute", "continuous")]
        self.revolute_names = [n for n in (joint_names or revolute_order) if n in self.joints]
        children_of_joints = {
            joint["child"] for joints in self.children.values() for joint in joints
        }
        self.base_frame = next(
            (joint["child"] for joint in self.children.get("world", [])
             if joint["type"] == "fixed"),
            next((link.attrib["name"] for link in root.findall("link")
                  if link.attrib["name"] not in children_of_joints), None),
        )
        self.link_order = []
        if self.base_frame:
            self._collect_link_order(self.base_frame)

    def _collect_link_order(self, link_name: str) -> None:
        if link_name not in self.link_order:
            self.link_order.append(link_name)
        for joint in self.children.get(link_name, []):
            self._collect_link_order(joint["child"])

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
        if not self.base_frame or self.base_frame not in frames:
            raise RuntimeError("URDF does not contain a discoverable robot base frame")
        T_world_base = frames[self.base_frame]
        T_base_world = np.linalg.inv(T_world_base)
        return {link: T_base_world @ transform for link, transform in frames.items()}


class Robot3DView(QWidget):
    """Read-only 10 Hz URDF kinematic view embedded in the HMI."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model = UrdfKinematicModel()
        # Keep a complete reference pose visible before/without live feedback.
        self.q = [0.0] * 6
        self.live_feedback = False
        self.tcp = None
        self.tcp_frame_id = ""
        self.canvas = FigureCanvas(Figure(figsize=(5.2, 4.2)))
        self.ax = self.canvas.figure.add_subplot(111, projection="3d")
        # Read-only scene layers mirroring the VTK view.
        self._scene = None
        self._scene_status = "not loaded"
        self._workpiece_calibration = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._planned_tcp_points = None
        self._visibility = {
            "robot": True,
            "workpiece": True,
            "seam": True,
            "tcp_path": False,
            "current_tcp": True,
            "collision": True,
        }
        try:
            from gui.scene_geometry import SceneGeometry
            self._scene = SceneGeometry()
            self._scene_status = "loaded"
        except Exception as exc:  # missing file / malformed json
            self._scene = None
            self._scene_status = f"unavailable: {exc}"
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    def set_state(self, q, tcp_position=None, tcp_frame_id="") -> None:
        if q is not None and len(q) == 6:
            self.q = list(q)
            self.live_feedback = True
        elif self.q is None:
            self.q = [0.0] * 6
            self.live_feedback = False
        else:
            self.live_feedback = False
        self.tcp = None if tcp_position is None else list(tcp_position)
        self.tcp_frame_id = str(tcp_frame_id or "")

    def set_preview_state(self, q, tcp_index: int | None = None) -> None:
        """Display a local preview pose without presenting it as live TCP data."""
        if q is not None and len(q) == 6:
            self.q = list(q)
        self.live_feedback = False
        if tcp_index is None or self._planned_tcp_points is None:
            self.tcp = None
            self.tcp_frame_id = ""
        else:
            idx = int(np.clip(tcp_index, 0, len(self._planned_tcp_points) - 1))
            self.tcp = list(self._planned_tcp_points[idx])
            self.tcp_frame_id = "base"

    # --- read-only scene layers (mirror of the VTK widget) -----------------
    def set_planned_tcp_path(self, points, frame: str = "base_link") -> None:
        if points is None:
            self._planned_tcp_points = None
            return
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            self._planned_tcp_points = None
            return
        if frame == "base_link":
            from gui.scene_geometry import base_link_to_base
            pts = base_link_to_base(pts)
        self._planned_tcp_points = pts

    def clear_planned_tcp_path(self) -> None:
        self._planned_tcp_points = None

    def set_robot_profile(self, robot_description: Path, joint_names, scene_geometry_path=None) -> None:
        self.model = UrdfKinematicModel(Path(robot_description), joint_names=joint_names)
        self.q = [0.0] * len(joint_names)
        self.live_feedback = False
        self.tcp = None
        self.tcp_frame_id = ""
        self.clear_planned_tcp_path()
        self._workpiece_calibration = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        if scene_geometry_path:
            from gui.scene_geometry import SceneGeometry
            self._scene = SceneGeometry(scene_geometry_path)
            self._scene_status = "loaded"
        else:
            self._scene = None
            self._scene_status = "not configured for this robot"
        self._refresh()

    def scene_status(self) -> str:
        return f"{self._scene_status} | 工件位姿: {self._workpiece_pose_source}"

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

    def preview_point_base(self, index: int):
        if self._planned_tcp_points is None or len(self._planned_tcp_points) == 0:
            return None
        idx = int(np.clip(index, 0, len(self._planned_tcp_points) - 1))
        return self._transform_workpiece_points(self._planned_tcp_points[idx])

    def set_urdf_text(self, urdf_text: str) -> None:
        try:
            self.model = UrdfKinematicModel(urdf_text=urdf_text,
                                            joint_names=self.model.revolute_names)
        except Exception:
            # Keep the known calibrated local URDF as a safe display fallback.
            pass

    def _refresh(self) -> None:
        if self.q is None or len(self.q) != 6:
            return
        # During headless/startup layout the canvas can temporarily be 0x0;
        # avoid Matplotlib divide-by-zero warnings until the widget is visible.
        if self.width() < 10 or self.height() < 10:
            return
        try:
            points, labels = self.model.link_points_in_base(self.q)
        except Exception:
            return
        ax = self.ax
        ax.clear()
        # ax.clear() wipes every previous artist, so all scene overlays are
        # rebuilt here on every refresh instead of being added once.
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
            path = self._transform_workpiece_points(self._planned_tcp_points)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], "--", color="#26f269", linewidth=2)
        if self._visibility.get("robot", True):
            ax.plot(points[:, 0], points[:, 1], points[:, 2], "-o", color="#2f6db0", linewidth=4, markersize=5)
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
        if (
            self.tcp is not None
            and len(self.tcp) == 3
            and self.tcp_frame_id in ("base", "base_link")
            and self._visibility.get("current_tcp", True)
        ):
            ax.scatter([self.tcp[0]], [self.tcp[1]], [self.tcp[2]], color="#d62728", s=65, label="TCP")
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
