from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QPushButton, QPlainTextEdit
from scipy.spatial import ConvexHull

from ros.state_monitor import RobotState
from gui.robot_view import URDF_PATH, UrdfKinematicModel, _validated_tcp_transform
from gui.scene_geometry import SceneGeometry, EnvironmentGeometry, DEFAULT_ENVIRONMENT_JSON, base_link_to_base
from safety.joint_limits import JOINT_NAMES
from gui.preview_display import PreviewDisplayMixin

try:
    import vtk
    from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
except ImportError as exc:  # pragma: no cover - fallback handled by main_window
    raise ImportError("VTK Qt5 integration is unavailable") from exc


COLLISION_DIR = Path("/opt/ros/humble/share/ur_description/meshes/ur5/collision")

# Read-only scene display.  The workpiece boxes come from scene_geometry.json
# (CoppeliaSim UR5_base) and the planned TCP path from the MATLAB planner
# (base_link); each uses its own verified transform, see gui/scene_geometry.py.
WORKPIECE_COLOR = (0.62, 0.64, 0.67)
WORKPIECE_OPACITY = 0.32
SEAM_COLOR = (1.0, 0.85, 0.10)
PLANNED_TCP_COLOR = (0.15, 0.95, 0.35)
LIVE_TCP_COLOR = (0.95, 0.10, 0.10)
PREVIEW_TCP_COLOR = (0.20, 0.45, 1.00)
GUN_COLLISION_COLOR = (1.00, 0.55, 0.10)
# Display-only radius. Never used for wire or collision geometry.
TCP_MARKER_RADIUS_M = 0.003  # 3 mm radius: pinpoint TCP without hiding the wire tip.
REAL_GUN_MANIFEST = Path(__file__).resolve().parents[2] / "real_gun_geometry" / "real_gun_geometry.json"
# Prefer the controller's active TCP pose while the ROS topic is fresh.  FK
# from the fixed HMI offset is only a clearly labelled fallback.
LIVE_TCP_MAX_AGE_SEC = 1.5
# Field-confirmed display correction: the physical gun is rotated +90 deg
# (right-hand convention) about the URDF flange/J6 axis.  This is a display
# variant only; the MATLAB collision scene is not silently changed here.
REAL_GUN_MOUNT_ROLL_DEG = 90.0

JOINT_CN = {
    "shoulder_pan_joint": "肩部旋转",
    "shoulder_lift_joint": "肩部俯仰",
    "elbow_joint": "肘部",
    "wrist_1_joint": "腕部1",
    "wrist_2_joint": "腕部2",
    "wrist_3_joint": "腕部3",
}


def _resolve_mesh(filename: str, link_name: str) -> Path | None:
    """Resolve a URDF mesh to a local STL suitable for VTK.

    The ROS visual meshes are COLLADA .dae files, while the matching collision
    meshes are STL and are already installed with ur_description.  They use
    the same URDF link-local origin and provide a lightweight, accurate display
    model without adding another Python mesh dependency.
    """
    basename = Path(filename.split("/")[-1]).stem
    candidate = COLLISION_DIR / f"{basename}.stl"
    if candidate.exists():
        return candidate
    candidate = COLLISION_DIR / f"{link_name.replace('_link', '')}.stl"
    return candidate if candidate.exists() else None


def _vtk_matrix(T: np.ndarray):
    matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            matrix.SetElement(r, c, float(T[r, c]))
    return matrix


def _axis_angle_rotation(axis, angle_rad: float) -> np.ndarray:
    """Return a homogeneous right-hand rotation about an arbitrary axis."""
    axis = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12 or abs(float(angle_rad)) <= 1e-15:
        return np.eye(4, dtype=float)
    x, y, z = axis / norm
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    C = 1.0 - c
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ], dtype=float)
    return T


def _oriented_box_actor(box) -> "vtk.vtkActor":
    """Build an actor for a possibly rotated collision box.

    vtkCubeSource can only produce axis-aligned boxes, which would draw the
    Workpiece plates with swapped x/y extents.  The eight already-transformed
    corners are therefore triangulated directly.
    """
    corners = np.asarray(box.corners_base(), dtype=float)
    points = vtk.vtkPoints()
    for corner in corners:
        points.InsertNextPoint(float(corner[0]), float(corner[1]), float(corner[2]))
    triangles = vtk.vtkCellArray()
    for quad in np.asarray(box.faces(), dtype=int):
        for triple in ((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])):
            cell = vtk.vtkTriangle()
            for k, index in enumerate(triple):
                cell.GetPointIds().SetId(k, int(index))
            triangles.InsertNextCell(cell)
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(triangles)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


def _polyline_actor(points) -> "vtk.vtkActor":
    """Build a single polyline actor from an (N,3) array of points.

    VTK 9.1 requires a vtkCellArray for SetLines, so the polyline cell is
    inserted into one instead of being assigned directly.
    """
    pts = np.asarray(points, dtype=float)
    vtk_points = vtk.vtkPoints()
    for point in pts:
        vtk_points.InsertNextPoint(float(point[0]), float(point[1]), float(point[2]))
    polyline = vtk.vtkPolyLine()
    polyline.GetPointIds().SetNumberOfIds(len(pts))
    for index in range(len(pts)):
        polyline.GetPointIds().SetId(index, index)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(polyline)
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(vtk_points)
    polydata.SetLines(cells)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


def _triangle_mesh_actor(vertices, faces) -> "vtk.vtkActor":
    """Create a display actor from exported collision-hull mesh data.

    The vertices/faces are copied verbatim from ``scene_geometry.json``.  No
    dimensions or poses are inferred here; the actor is later attached to the
    dynamic flange->mount transform.
    """
    points = vtk.vtkPoints()
    for point in np.asarray(vertices, dtype=float):
        points.InsertNextPoint(float(point[0]), float(point[1]), float(point[2]))
    triangles = vtk.vtkCellArray()
    for face in np.asarray(faces, dtype=int):
        cell = vtk.vtkTriangle()
        for index, vertex in enumerate(face):
            cell.GetPointIds().SetId(index, int(vertex) - 1)
        triangles.InsertNextCell(cell)
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(triangles)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


class VtkRobotView(QWidget, PreviewDisplayMixin):
    """Embedded URDF-derived UR5 mesh view.

    It is read-only.  No ROS publishers, actions, services, or robot command
    interfaces are referenced by this widget.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model = UrdfKinematicModel()
        # Keep a valid display configuration even before the first ROS sample.
        # When feedback becomes stale we freeze this pose instead of leaving
        # every mesh at its untransformed link-local origin.
        # Live feedback and preview state are deliberately separate.  The
        # displayed robot pose may follow ``_preview_q`` while ROS continues
        # updating ``_live_q`` in the background; preview must never overwrite
        # the real joint/TCP snapshot.
        self.q = [0.0] * 6  # current display pose (live or preview)
        self._live_q = None
        self._preview_q = None
        self.preview_active = False
        self.live_feedback = False
        self._live_tcp_topic = None  # diagnostic only; never used for preview
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
        self.tcp = None  # legacy live marker accessor, kept for compatibility
        self.tcp_frame_id = ""
        self._actors = {}
        self._joint_labels = {}
        self._tcp_label = None
        self._calibrated_tcp_offset = None
        self._tcp_offset_flange = None
        self._tcp_rotation_flange = np.eye(3, dtype=float)
        self._planning_tcp_transform = None
        self._calibrated_tcp_actor = None
        self._calibrated_tcp_line_actor = None
        self._calibrated_tcp_line_source = None
        self._calibrated_tcp_label = None
        self._camera_initialized = False
        # Read-only scene layers (workpiece / seam / planned TCP path).
        self._workpiece_actors = []
        self._workpiece_outline_actors = []
        self._seam_actors = {}
        self._planned_tcp_actor = None
        self._planned_tcp_points = None
        self._gun_actor = None
        self._gun_collision_actors = []
        self._gun_collision_mount_transforms = []
        self._gun_actor_mount_transform = None
        self._gun_mount_roll_deg = REAL_GUN_MOUNT_ROLL_DEG
        # User adjustment is expressed in the URDF flange frame.  It is a
        # display-only rigid transform: the flange origin and the active TCP
        # marker are never modified by it.
        self._gun_mount_translation_m = np.zeros(3, dtype=float)
        self._gun_nominal_mount_transform = None
        self._gun_mount_roll_axis_flange = None
        self._gun_mount_adjustment_status = "DEFAULT"
        self._gun_display_status = "unavailable"
        self._wire_actor = None
        self._wire_collision_actor = None
        self._wire_live_planning_error_mm = None
        self._wire_tcp_transform = None
        self._wire_nominal_tcp_transform = None
        self._flange_tcp_nominal_transform = None
        self._wire_tip_tcp = None
        self._wire_length_m = None
        self._wire_display_status = "unavailable"
        # Legacy diagnostic handles stay empty: no yellow WIRE TIP sphere
        # or deviation connector is created by the model-defined wire.
        self._wire_tip_actor = None
        self._wire_tip_label = None
        self._wire_tcp_error_line_actor = None
        self._wire_tcp_error_line_source = None
        self._wire_tcp_error_label = None
        self._workpiece_origin_actor = None
        self._workpiece_origin_label = None
        self._workpiece_origin_axes = []
        self._scene = None
        self._scene_status = "not loaded"
        self._environment_actors = {}
        self._preview_environment_actors = {}
        self._environment_geometry = None
        self._environment_status = "未加载"
        self._environment_path = DEFAULT_ENVIRONMENT_JSON
        self._environment_file_binding = None
        self._environment_last_check = 0.0
        # Display-only transform from the nominal exported workpiece frame to
        # a three-point real calibration.  Robot/TCP coordinates are never
        # changed by this transform.
        self._workpiece_calibration_delta = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._visibility = {
            "robot": True,
            "workpiece": True,
            "seam": True,
            "tcp_path": False,
            "current_tcp": True,
            "preview_tcp": True,
            "gun": True,
            "wire": True,
            "collision": True,
            "environment": True,
        }

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.09, 0.11, 0.15)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self._default_view_angle = float(self.renderer.GetActiveCamera().GetViewAngle())
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.vtk_widget)
        self._wire_nozzle_source = None
        self._wire_nozzle_axis_source = None
        start_sphere = vtk.vtkSphereSource()
        start_sphere.SetRadius(0.002)
        start_sphere.SetThetaResolution(20)
        start_sphere.SetPhiResolution(16)
        start_mapper = vtk.vtkPolyDataMapper()
        start_mapper.SetInputConnection(start_sphere.GetOutputPort())
        self._wire_start_marker = vtk.vtkActor()
        self._wire_start_marker.SetMapper(start_mapper)
        self._wire_start_marker.GetProperty().SetColor(0.1, 1.0, 0.2)
        self._wire_start_marker.SetVisibility(False)
        self.renderer.AddActor(self._wire_start_marker)
        self._wire_diagnostic_button = QPushButton("焊丝几何诊断（只读，world = URDF base）")
        self._wire_diagnostic_button.setCheckable(True)
        self._wire_diagnostic_text = QPlainTextEdit()
        self._wire_diagnostic_text.setReadOnly(True)
        self._wire_diagnostic_text.setMaximumHeight(190)
        self._wire_diagnostic_text.setMinimumHeight(120)
        self._wire_diagnostic_text.hide()
        self._wire_diagnostic_button.toggled.connect(self._wire_diagnostic_text.setVisible)
        layout.addWidget(self._wire_diagnostic_button)
        layout.addWidget(self._wire_diagnostic_text)
        self._add_axes()
        self._load_meshes()
        self._load_scene_geometry()
        self.reload_environment_geometry()
        self.vtk_widget.GetRenderWindow().Render()
        self.vtk_widget.Initialize()
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()

    def _add_axes(self):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.125, 0.125, 0.125)
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.010)
        axes.SetConeRadius(0.018)
        axes.SetSphereRadius(0.014)
        axes.SetXAxisLabelText("X")
        axes.SetYAxisLabelText("Y")
        axes.SetZAxisLabelText("Z")
        # VTK's built-in caption actors scale very aggressively with the
        # camera, which made the XYZ letters dominate the enlarged scene.
        # Keep the actual axes unchanged, hide only those captions, and use
        # small billboard labels at the same axis tips.
        axes.AxisLabelsOff()
        self.renderer.AddActor(axes)
        self._axes = axes
        self._axis_label_actors = []
        for letter, position in (
            ("X", (0.145, 0.0, 0.0)),
            ("Y", (0.0, 0.145, 0.0)),
            # Lift Z slightly above the base so the compact label is not
            # occluded by the robot pedestal in the default isometric view.
            ("Z", (0.0, 0.0, 0.165)),
        ):
            label = vtk.vtkBillboardTextActor3D()
            label.SetInput(letter)
            label.SetPosition(*position)
            text = label.GetTextProperty()
            text.SetFontSize(14)
            text.SetBold(False)
            text.SetColor(0.95, 0.95, 0.95)
            text.SetBackgroundOpacity(0.0)
            self.renderer.AddActor(label)
            self._axis_label_actors.append(label)

    def _load_meshes(self):
        if getattr(self, "_gun_actor", None) is not None:
            self.renderer.RemoveActor(self._gun_actor)
            self._gun_actor = None
            self._gun_actor_mount_transform = None
        for actor in getattr(self, "_gun_collision_actors", []):
            self.renderer.RemoveActor(actor)
        self._gun_collision_actors = []
        self._gun_collision_mount_transforms = []
        if getattr(self, "_wire_actor", None) is not None:
            self.renderer.RemoveActor(self._wire_actor)
            self._wire_actor = None
            self._wire_tcp_transform = None
        if getattr(self, "_wire_collision_actor", None) is not None:
            self.renderer.RemoveActor(self._wire_collision_actor)
            self._wire_collision_actor = None
        for actor, _ in self._actors.values():
            self.renderer.RemoveActor(actor)
        for label, _ in self._joint_labels.values():
            self.renderer.RemoveActor(label)
        if hasattr(self, "_tcp_actor"):
            self.renderer.RemoveActor(self._tcp_actor)
        if hasattr(self, "_tcp_line_actor"):
            self.renderer.RemoveActor(self._tcp_line_actor)
        if self._tcp_label is not None:
            self.renderer.RemoveActor(self._tcp_label)
        for actor in (
            getattr(self, "_preview_tcp_actor", None),
            getattr(self, "_preview_tcp_line_actor", None),
            getattr(self, "_preview_tcp_label", None),
        ):
            if actor is not None:
                self.renderer.RemoveActor(actor)
        for actor in (
            getattr(self, "_wire_tip_actor", None),
            getattr(self, "_wire_tcp_error_line_actor", None),
            getattr(self, "_wire_tip_label", None),
            getattr(self, "_wire_tcp_error_label", None),
        ):
            if actor is not None:
                self.renderer.RemoveActor(actor)
        for actor in (self._calibrated_tcp_actor, self._calibrated_tcp_line_actor, self._calibrated_tcp_label):
            if actor is not None:
                self.renderer.RemoveActor(actor)
        self._actors = {}
        self._joint_labels = {}
        self._tcp_label = None
        self._calibrated_tcp_actor = None
        self._calibrated_tcp_line_actor = None
        self._calibrated_tcp_line_source = None
        self._calibrated_tcp_label = None
        self._wire_tip_actor = None
        self._wire_tip_label = None
        self._wire_tcp_error_line_actor = None
        self._wire_tcp_error_line_source = None
        self._wire_tcp_error_label = None
        self._camera_initialized = False
        for link_name, info in self.model.link_meshes.items():
            path = _resolve_mesh(info["filename"], link_name)
            if path is None:
                continue
            reader = vtk.vtkSTLReader()
            reader.SetFileName(str(path))
            reader.Update()
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            # Original first-version HMI appearance: one neutral mesh color.
            # No per-link palette or newly invented material colors are used.
            actor.GetProperty().SetColor(0.75, 0.78, 0.82)
            actor.GetProperty().SetAmbient(0.20)
            actor.GetProperty().SetDiffuse(0.80)
            actor.GetProperty().SetSpecular(0.28)
            actor.GetProperty().SetSpecularPower(24)
            self.renderer.AddActor(actor)
            self._actors[link_name] = (actor, info["origin"])

        # Put a Chinese J1--J6 label at each corresponding URDF joint frame.
        # These are read-only annotations and follow the live robot pose.
        for index, joint_name in enumerate(JOINT_NAMES, start=1):
            joint = self.model.joints.get(joint_name)
            if joint is None:
                continue
            label = vtk.vtkBillboardTextActor3D()
            label.SetInput(f"J{index}")
            text = label.GetTextProperty()
            text.SetFontSize(14)
            text.SetBold(True)
            text.SetColor(1.0, 1.0, 1.0)
            text.SetBackgroundColor(0.05, 0.05, 0.05)
            text.SetBackgroundOpacity(0.65)
            self.renderer.AddActor(label)
            self._joint_labels[joint_name] = (label, joint["child"])

        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(TCP_MARKER_RADIUS_M)
        sphere.SetThetaResolution(20)
        sphere.SetPhiResolution(12)
        sphere.Update()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        self._tcp_actor = vtk.vtkActor()
        self._tcp_actor.SetMapper(mapper)
        self._tcp_actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        self.renderer.AddActor(self._tcp_actor)
        line = vtk.vtkLineSource()
        line.SetPoint1(0.0, 0.0, 0.0)
        line.SetPoint2(0.0, 0.0, 0.0)
        line.Update()
        line_mapper = vtk.vtkPolyDataMapper()
        line_mapper.SetInputConnection(line.GetOutputPort())
        self._tcp_line_actor = vtk.vtkActor()
        self._tcp_line_actor.SetMapper(line_mapper)
        self._tcp_line_actor.GetProperty().SetColor(0.95, 0.25, 0.15)
        self._tcp_line_actor.GetProperty().SetLineWidth(2.0)
        self.renderer.AddActor(self._tcp_line_actor)
        self._tcp_line_source = line
        self._tcp_label = vtk.vtkBillboardTextActor3D()
        self._tcp_label.SetInput("TCP LIVE")
        tcp_text = self._tcp_label.GetTextProperty()
        tcp_text.SetFontSize(18)
        tcp_text.SetBold(True)
        tcp_text.SetColor(1.0, 1.0, 1.0)
        tcp_text.SetBackgroundColor(0.05, 0.05, 0.05)
        tcp_text.SetBackgroundOpacity(0.65)
        self.renderer.AddActor(self._tcp_label)
        preview_sphere = vtk.vtkSphereSource()
        preview_sphere.SetRadius(0.021)
        preview_sphere.SetThetaResolution(20)
        preview_sphere.SetPhiResolution(12)
        preview_mapper = vtk.vtkPolyDataMapper()
        preview_mapper.SetInputConnection(preview_sphere.GetOutputPort())
        self._preview_tcp_actor = vtk.vtkActor()
        self._preview_tcp_actor.SetMapper(preview_mapper)
        self._preview_tcp_actor.GetProperty().SetColor(*PREVIEW_TCP_COLOR)
        self.renderer.AddActor(self._preview_tcp_actor)
        preview_line = vtk.vtkLineSource()
        preview_line.SetPoint1(0.0, 0.0, 0.0)
        preview_line.SetPoint2(0.0, 0.0, 0.0)
        preview_line_mapper = vtk.vtkPolyDataMapper()
        preview_line_mapper.SetInputConnection(preview_line.GetOutputPort())
        self._preview_tcp_line_actor = vtk.vtkActor()
        self._preview_tcp_line_actor.SetMapper(preview_line_mapper)
        self._preview_tcp_line_actor.GetProperty().SetColor(*PREVIEW_TCP_COLOR)
        self._preview_tcp_line_actor.GetProperty().SetLineWidth(2.0)
        self.renderer.AddActor(self._preview_tcp_line_actor)
        self._preview_tcp_line_source = preview_line
        self._preview_tcp_label = vtk.vtkBillboardTextActor3D()
        self._preview_tcp_label.SetInput("TCP PREVIEW")
        preview_text = self._preview_tcp_label.GetTextProperty()
        preview_text.SetFontSize(16)
        preview_text.SetBold(True)
        preview_text.SetColor(*PREVIEW_TCP_COLOR)
        preview_text.SetBackgroundColor(0.05, 0.05, 0.05)
        preview_text.SetBackgroundOpacity(0.65)
        self.renderer.AddActor(self._preview_tcp_label)
        # A calibrated FK TCP is deliberately a separate cyan annotation.  It
        # never replaces the live red TCP topic marker.
        cal_sphere = vtk.vtkSphereSource()
        cal_sphere.SetRadius(0.018)
        cal_sphere.SetThetaResolution(18)
        cal_sphere.SetPhiResolution(10)
        cal_mapper = vtk.vtkPolyDataMapper()
        cal_mapper.SetInputConnection(cal_sphere.GetOutputPort())
        self._calibrated_tcp_actor = vtk.vtkActor()
        self._calibrated_tcp_actor.SetMapper(cal_mapper)
        self._calibrated_tcp_actor.GetProperty().SetColor(0.15, 0.95, 0.95)
        self.renderer.AddActor(self._calibrated_tcp_actor)
        cal_line = vtk.vtkLineSource()
        cal_line.SetPoint1(0.0, 0.0, 0.0)
        cal_line.SetPoint2(0.0, 0.0, 0.0)
        cal_line_mapper = vtk.vtkPolyDataMapper()
        cal_line_mapper.SetInputConnection(cal_line.GetOutputPort())
        self._calibrated_tcp_line_source = cal_line
        self._calibrated_tcp_line_actor = vtk.vtkActor()
        self._calibrated_tcp_line_actor.SetMapper(cal_line_mapper)
        self._calibrated_tcp_line_actor.GetProperty().SetColor(0.15, 0.95, 0.95)
        self._calibrated_tcp_line_actor.GetProperty().SetLineWidth(1.5)
        self.renderer.AddActor(self._calibrated_tcp_line_actor)
        self._calibrated_tcp_label = vtk.vtkBillboardTextActor3D()
        self._calibrated_tcp_label.SetInput("TCP calibrated (FK)")
        cal_text = self._calibrated_tcp_label.GetTextProperty()
        cal_text.SetFontSize(13)
        cal_text.SetBold(False)
        cal_text.SetColor(0.25, 1.0, 1.0)
        cal_text.SetBackgroundOpacity(0.45)
        self.renderer.AddActor(self._calibrated_tcp_label)
        # Camera is initialized after the first real joint transform is
        # applied.  Resetting it here would frame all links at their zero-pose
        # origins and lose the robot once the live transforms arrive.

    # ------------------------------------------------------------------
    # Read-only scene geometry: workpiece boxes, seam lines, planned TCP path
    # ------------------------------------------------------------------
    def _load_scene_geometry(self) -> None:
        """Build the workpiece/seam actors from scene_geometry.json.

        Nothing here creates geometry of its own: box sizes and poses are the
        values exported by the CoppeliaSim geometry exporter, converted into the
        URDF ``base`` frame with the verified transform.
        """
        self._clear_scene_actors()
        try:
            scene = SceneGeometry()
        except Exception as exc:  # missing file / malformed json
            self._scene = None
            self._scene_status = f"unavailable: {exc}"
            return
        self._scene = scene
        self._create_gun_collision_actor()

        # A visible reference marker for the Workpiece frame origin.  It is a
        # display annotation only and is deliberately excluded from camera
        # fitting; the workpiece geometry remains the fit reference.
        self._create_workpiece_origin_marker()

        for box in scene.workpiece_boxes():
            actor = _oriented_box_actor(box)
            actor.GetProperty().SetColor(*WORKPIECE_COLOR)
            actor.GetProperty().SetOpacity(WORKPIECE_OPACITY)
            actor.GetProperty().SetEdgeVisibility(True)
            actor.GetProperty().SetEdgeColor(0.85, 0.86, 0.88)
            actor.GetProperty().SetLineWidth(1.0)
            actor.GetProperty().SetAmbient(0.25)
            actor.GetProperty().SetDiffuse(0.70)
            self.renderer.AddActor(actor)
            self._workpiece_actors.append(actor)

        # Seam centre lines for Seam A, plus B/C for later use.
        for name in ("A", "B", "C"):
            try:
                seam = scene.seam(name)
            except Exception:
                continue
            actor = _polyline_actor(seam.polyline_base)
            actor.GetProperty().SetColor(*SEAM_COLOR)
            actor.GetProperty().SetLineWidth(4.0)
            self.renderer.AddActor(actor)
            self._seam_actors[name] = actor

        self._scene_status = (
            f"{len(self._workpiece_actors)} workpiece boxes, "
            f"{len(self._seam_actors)} seam lines"
        )
        self._apply_workpiece_transform()

    @staticmethod
    def _pose_to_transform(pose) -> np.ndarray:
        """Convert an exported Coppelia pose ([x,y,z],[qx,qy,qz,qw])."""
        position = np.asarray(pose.get("position", [0.0, 0.0, 0.0]), dtype=float)
        qx, qy, qz, qw = [float(v) for v in pose.get("quaternion", [0, 0, 0, 1])]
        norm = max(float(np.linalg.norm([qx, qy, qz, qw])), 1e-12)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ], dtype=float)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = position[:3]
        return T

    @staticmethod
    def _read_real_gun_manifest() -> dict | None:
        """Read the independent real-gun package, never the frozen scene."""
        if not REAL_GUN_MANIFEST.is_file():
            return None
        try:
            with REAL_GUN_MANIFEST.open(encoding="utf-8") as handle:
                data = json.load(handle)
            if not isinstance(data, dict):
                raise ValueError("manifest root is not an object")
            return data
        except Exception:
            return {"status": "INVALID_MANIFEST"}

    @staticmethod
    def _manifest_matrix(value, name: str) -> np.ndarray:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError(f"{name} must be a finite 4x4 matrix")
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
            raise ValueError(f"{name} is not homogeneous")
        if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-6):
            raise ValueError(f"{name} is not a rigid transform")
        return matrix

    def _create_active_real_gun(self, manifest: dict, preview: bool = False) -> None:
        """Load active geometry or an explicitly labeled offline source preview.

        Visual and collision components share one source-to-flange matrix.
        A staged preview is never a measured mounting pose or planning approval.
        """
        visual = manifest.get("visual", {})
        collision = manifest.get("collision", {})
        transform = visual.get("T_flange_source") or collision.get("T_flange_source")
        if preview:
            # Explicit source-at-flange preview, NOT a guessed physical datum.
            # Raw STL origin and orientation remain intact; user aligns it.
            transform = manifest.get("preview", {}).get("T_flange_source", np.eye(4).tolist())
        if transform is None:
            raise ValueError("ACTIVE real-gun manifest has no T_flange_source")
        T_flange_source = self._manifest_matrix(transform, "T_flange_source")
        source_path = Path(str(visual.get("path", "")))
        if not source_path.is_file():
            raise FileNotFoundError(source_path)

        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(source_path))
        reader.Update()
        if reader.GetOutput() is None or reader.GetOutput().GetNumberOfPoints() == 0:
            raise ValueError("real-gun STL has no points")
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        if str(visual.get("units", "mm")).lower() == "mm":
            actor.SetScale(0.001, 0.001, 0.001)
        actor.GetProperty().SetColor(0.35, 0.37, 0.39)
        actor.GetProperty().SetOpacity(0.96)
        actor.GetProperty().SetEdgeVisibility(False)
        self.renderer.AddActor(actor)
        self._gun_actor = actor
        self._gun_nominal_mount_transform = T_flange_source
        self._gun_actor_mount_transform = T_flange_source.copy()
        # The manifest already contains the complete confirmed installation
        # transform.  Do not apply the legacy display-only +90° adjustment a
        # second time; any further adjustment must be an explicit user action.
        self._gun_mount_roll_deg = 0.0
        self._gun_mount_translation_m = np.zeros(3, dtype=float)
        self._gun_mount_adjustment_status = "UNCONFIRMED_SOURCE_PREVIEW" if preview else "ACTIVE_MANIFEST"
        self._gun_source_sha256 = manifest.get("source", {}).get("sha256")
        self._gun_geometry_preview = preview

        # Collision is a distinct translucent layer.  Components are already
        # convex vertices in the same STL source frame; triangulate only for
        # display, while MATLAB consumes the exact component vertices.
        components = collision.get("components", [])
        for component in components:
            vertices = np.asarray(component.get("vertices_m", []), dtype=float)
            if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4:
                continue
            try:
                faces = ConvexHull(vertices).simplices
            except Exception:
                continue
            hull_actor = _triangle_mesh_actor(vertices, faces + 1)
            hull_actor.GetProperty().SetColor(0.96, 0.70, 0.10)
            hull_actor.GetProperty().SetOpacity(0.20)
            hull_actor.GetProperty().SetEdgeVisibility(True)
            hull_actor.GetProperty().SetEdgeColor(1.0, 0.75, 0.1)
            self.renderer.AddActor(hull_actor)
            self._gun_collision_actors.append(hull_actor)
            self._gun_collision_mount_transforms.append(T_flange_source.copy())

        self._gun_display_status = ("新 STL + V-HACD：安装未确认，仅预览" if preview
                                    else "真实焊枪 STL + 独立 V-HACD 碰撞层")
        self._wire_display_status = "REAL 焊丝不可用：缺少有效模型定义"
        self._wire_actor = None
        self._wire_tip_tcp = None
        self._wire_nominal_tcp_transform = None

        # Keep the existing flange/TCP calibration chain and J6 axis logic;
        # loading a new gun never modifies the controller TCP.
        from scipy.io import loadmat
        calibration = loadmat(str(Path(__file__).resolve().parents[2] / "ur5_tool_calibration.mat"))
        T_tool0_mount = np.asarray(calibration["T_tool0_mount"], dtype=float)
        T0 = self.model.link_transforms_in_base(np.zeros(6, dtype=float))
        T_flange_tool0 = np.linalg.inv(T0["flange"]) @ T0["tool0"]
        if "T_mount_tcp" in calibration:
            T_mount_tcp = np.asarray(calibration["T_mount_tcp"], dtype=float)
            self._flange_tcp_nominal_transform = T_flange_tool0 @ T_tool0_mount @ T_mount_tcp
            if self._planning_tcp_transform is None:
                self._tcp_rotation_flange = self._flange_tcp_nominal_transform[:3, :3].copy()
        j6 = self.model.joints.get("wrist_3_joint")
        if j6 is None or j6.get("parent") not in T0:
            raise RuntimeError("URDF wrist_3_joint/J6 frame is unavailable")
        axis_base = (T0[j6["parent"]][:3, :3]
                     @ np.asarray(j6["origin"], dtype=float)[:3, :3]
                     @ np.asarray(j6["axis"], dtype=float).reshape(3))
        axis_flange = T0["flange"][:3, :3].T @ axis_base
        norm = float(np.linalg.norm(axis_flange))
        if norm <= 1e-12:
            raise RuntimeError("J6/flange axis is degenerate")
        self._gun_mount_roll_axis_flange = axis_flange / norm

        self._refresh_model_axis_wire(manifest)
        self._update_gun_mount_adjustment_transform(render=False)

    def _refresh_model_axis_wire(self, manifest: dict | None = None) -> None:
        """Create/update wire layers after the ACTIVE TCP first becomes available.

        Both layers consume the same saved planning definition.  Updating
        this display reads records only and never changes gun installation,
        controller TCP, joint feedback, or collision/calibration files.
        """
        from calibration.model_axis_wire import build_model_axis_wire
        try:
            manifest = self._read_real_gun_manifest() if manifest is None else manifest
            self._wire_nozzle_source = None
            self._wire_nozzle_axis_source = None
            # Independent CAD datum, not inferred from the displayed wire.
            evidence = (manifest or {}).get("wire", {}).get("evidence_file")
            if evidence and Path(evidence).is_file():
                datum = json.loads(Path(evidence).read_text(encoding="utf-8"))
                if datum.get("source_sha256") == (manifest or {}).get("source", {}).get("sha256"):
                    self._wire_nozzle_source = np.asarray(datum["nozzle_center_source_mm"], dtype=float) * 0.001
                    self._wire_nozzle_axis_source = np.asarray(datum["nozzle_axis_source"], dtype=float)
            if self._gun_actor is None or self._gun_actor_mount_transform is None:
                raise ValueError("active real-gun visual/flange transform is unavailable")
            wire = build_model_axis_wire(Path(__file__).resolve().parents[2], manifest or {},
                                         preview_only=bool(getattr(self, "_gun_geometry_preview", False)))
            start = np.asarray(wire["tip_source"], dtype=float)
            end = np.asarray(wire["wire_end_source"], dtype=float)
            if (start.shape != (3,) or end.shape != (3,)
                    or not np.all(np.isfinite(np.r_[start, end]))):
                raise ValueError("wire endpoints must be finite source-frame 3-vectors")
            # The first controller sync may happen after startup, when no
            # wire actor existed.  Do not condition creation on its existence.
            if self._wire_actor is None or self._wire_collision_actor is None:
                for existing in (self._wire_actor, self._wire_collision_actor):
                    if existing is not None:
                        self.renderer.RemoveActor(existing)
                for prefix, color, opacity in (
                    ("_wire", (1.0, 0.85, 0.1), 1.0),
                    ("_wire_collision", GUN_COLLISION_COLOR, 0.35),
                ):
                    line = vtk.vtkLineSource()
                    tube = vtk.vtkTubeFilter()
                    tube.SetInputConnection(line.GetOutputPort())
                    tube.SetRadius(wire["diameter_m"] / 2)
                    tube.SetNumberOfSides(20)
                    tube.CappingOn()
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(tube.GetOutputPort())
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    actor.GetProperty().SetColor(*color)
                    actor.GetProperty().SetOpacity(opacity)
                    self.renderer.AddActor(actor)
                    setattr(self, prefix + "_line_source", line)
                    setattr(self, prefix + "_tube", tube)
                    setattr(self, prefix + "_actor", actor)
            else:
                self._wire_tube.SetRadius(wire["diameter_m"] / 2)
                self._wire_collision_tube.SetRadius(wire["diameter_m"] / 2)
            self._wire_start_source = start.copy()
            self._wire_end_source = end.copy()
            self._wire_display_status = (
                f"模型枪口→示教器同步 TCP 焊丝 Ø1.2 mm / {wire['length_m'] * 1000:.3f} mm"
                + ("（安装待确认，仅预览）" if wire["preview_only"] else "")
            )
        except (OSError, ValueError, TypeError) as exc:
            self._wire_display_status = f"REAL 焊丝不可用：{exc}"
            self._wire_end_source = None
            self._wire_live_planning_error_mm = None
            for actor in (self._wire_actor, self._wire_collision_actor):
                if actor is not None:
                    actor.SetVisibility(False)

    def _create_gun_collision_actor(self) -> None:
        """Display the exported GunCollisionHull with the live flange pose.

        MATLAB uses the same exported hull as the gun collision geometry.  The
        HMI only visualizes it; it is not added to ROS2 or to any collision
        checker.  The flange->mount transform comes from the frozen
        ``ur5_tool_calibration.mat`` and is read-only.
        """
        self._gun_display_status = "unavailable"
        self._gun_actor = None
        self._gun_collision_actors = []
        self._gun_collision_mount_transforms = []
        self._gun_actor_mount_transform = None
        self._gun_nominal_mount_transform = None
        self._wire_nominal_tcp_transform = None
        self._flange_tcp_nominal_transform = None
        self._wire_tip_tcp = None
        self._wire_length_m = None
        try:
            real_manifest = None
            real_manifest = self._read_real_gun_manifest()
            if real_manifest is not None:
                if str(real_manifest.get("status", "")).upper() == "ACTIVE":
                    self._create_active_real_gun(real_manifest)
                    return
                if (real_manifest.get("collision", {}).get("method") == "CoppeliaSim simConvex.vhacd"
                        and real_manifest.get("collision", {}).get("components")):
                    self._create_active_real_gun(real_manifest, preview=True)
                    return
                self._gun_display_status = (
                    "新焊枪 STL 已暂存（旧冻结 GunCollisionHull 仍显示；"
                    f"状态 {real_manifest.get('status', 'UNKNOWN')}）"
                )
            spec = self._scene.data["welding_gun"]["collision_hull"]
            vertices = np.asarray(spec["vertices_m"], dtype=float)
            faces = np.asarray(spec["faces"], dtype=int)
            if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
                raise ValueError("GunCollisionHull vertices/faces are not N×3")
            actor = _triangle_mesh_actor(vertices, faces)
            actor.GetProperty().SetColor(*GUN_COLLISION_COLOR)
            actor.GetProperty().SetOpacity(0.34)
            actor.GetProperty().SetEdgeVisibility(True)
            actor.GetProperty().SetEdgeColor(1.0, 0.75, 0.25)
            actor.GetProperty().SetLineWidth(1.0)
            self.renderer.AddActor(actor)

            # T_tool0_mount is the frozen tool calibration transform.  The
            # URDF model supplies flange->tool0, so no tool0/TCP transform is
            # duplicated in the HMI display chain.
            from scipy.io import loadmat
            calibration = loadmat(str(Path(__file__).resolve().parents[2] / "ur5_tool_calibration.mat"))
            T_tool0_mount = np.asarray(calibration["T_tool0_mount"], dtype=float)
            T0 = self.model.link_transforms_in_base(np.zeros(6, dtype=float))
            T_flange_tool0 = np.linalg.inv(T0["flange"]) @ T0["tool0"]
            T_mount_hull = self._pose_to_transform(spec.get("pose_mount", {}))
            # The field-confirmed physical installation differs from the
            # nominal exported HMI gun by +90 deg about the J6/flange axis.
            # Compute that axis from the calibrated URDF instead of assuming
            # a world axis.  The rotation is left-multiplied in flange
            # coordinates, so the flange origin remains the rotation centre.
            j6 = self.model.joints.get("wrist_3_joint")
            if j6 is None or j6.get("parent") not in T0:
                raise RuntimeError("URDF wrist_3_joint/J6 frame is unavailable")
            axis_base = (
                T0[j6["parent"]][:3, :3]
                @ np.asarray(j6["origin"], dtype=float)[:3, :3]
                @ np.asarray(j6["axis"], dtype=float).reshape(3)
            )
            axis_flange = T0["flange"][:3, :3].T @ axis_base
            axis_norm = float(np.linalg.norm(axis_flange))
            if axis_norm <= 1e-12:
                raise RuntimeError("J6/flange axis is degenerate")
            axis_flange = axis_flange / axis_norm
            self._gun_mount_roll_axis_flange = axis_flange.copy()
            # Keep the nominal exported transforms separate from the user
            # adjustment.  This makes it impossible for a preview change to
            # accidentally alter the TCP offset or the MATLAB scene.
            self._gun_nominal_mount_transform = (
                T_flange_tool0 @ T_tool0_mount @ T_mount_hull
            )
            if "T_mount_tcp" in calibration:
                T_mount_tcp = np.asarray(calibration["T_mount_tcp"], dtype=float)
                self._flange_tcp_nominal_transform = T_flange_tool0 @ T_tool0_mount @ T_mount_tcp
                if self._planning_tcp_transform is None:
                    self._tcp_rotation_flange = self._flange_tcp_nominal_transform[:3, :3].copy()
            else:
                T_mount_tcp = None
            self._gun_actor = actor
            if real_manifest is None:
                self._gun_display_status = (
                    "GunCollisionHull（HMI flange局部调整；MATLAB场景未改写）"
                )
            else:
                self._gun_display_status = (
                    "新焊枪 STL 已暂存；旧冻结 GunCollisionHull 仍显示（"
                    f"{real_manifest.get('status', 'UNKNOWN')}）"
                )

            # No nearest-hull nozzle inference or artificial wire generation.
            self._wire_display_status = "无已确认原始焊丝网格；禁止自动生成"
            self._update_gun_mount_adjustment_transform(render=False)
        except Exception as exc:
            self._gun_display_status = f"unavailable: {exc}"
            self._wire_display_status = f"unavailable: {exc}"

    def _update_gun_mount_adjustment_transform(self, render: bool = True) -> None:
        """Rebuild the display transforms in the local flange frame.

        The same left-multiplied rigid transform is applied to the gun hull
        and the wire.  Consequently all four convex hull components (stored
        in the combined actor), the visual gun, and the wire retain their
        relative geometry while rotating about the physical flange origin.
        """
        axis = self._gun_mount_roll_axis_flange
        if axis is None:
            return
        T_adjust = _axis_angle_rotation(axis, np.deg2rad(float(self._gun_mount_roll_deg)))
        T_adjust[:3, 3] = np.asarray(self._gun_mount_translation_m, dtype=float).reshape(3)
        if self._gun_nominal_mount_transform is not None:
            self._gun_actor_mount_transform = T_adjust @ self._gun_nominal_mount_transform
        for index, actor in enumerate(getattr(self, "_gun_collision_actors", [])):
            # Collision components use the same source-frame transform as the
            # visual STL and are deliberately kept as separate display actors.
            if index < len(self._gun_collision_mount_transforms):
                self._gun_collision_mount_transforms[index] = T_adjust @ self._gun_nominal_mount_transform
        if self._wire_nominal_tcp_transform is not None:
            self._wire_tcp_transform = T_adjust @ self._wire_nominal_tcp_transform
        if render and hasattr(self, "vtk_widget"):
            self.vtk_widget.GetRenderWindow().Render()

    def set_gun_mount_adjustment(self, roll_deg: float, translation_mm=(0.0, 0.0, 0.0), render: bool = True) -> dict:
        """Set a flange-local, display-only gun/wire adjustment.

        ``roll_deg`` is the right-hand rotation about the URDF J6/flange axis,
        and ``translation_mm`` is expressed in the same local flange frame.
        No TCP, joint, ROS, or MATLAB planning state is changed.
        """
        roll = float(roll_deg)
        translation = np.asarray(translation_mm, dtype=float).reshape(-1)
        if not np.isfinite(roll) or roll < -180.0 or roll > 180.0:
            raise ValueError("gun mount roll must be finite and within [-180, 180] deg")
        if translation.size != 3 or not np.all(np.isfinite(translation)):
            raise ValueError("gun mount translation must be a finite XYZ 3-vector in mm")
        self._gun_mount_roll_deg = roll
        self._gun_mount_translation_m = translation * 1e-3
        self._gun_mount_adjustment_status = "PREVIEW"
        self._update_gun_mount_adjustment_transform(render=render)
        return self.gun_mount_adjustment()

    def set_gun_source_basis(self, matrix) -> None:
        """Explicit manual source-frame orientation; preview only, no TCP edits."""
        if not getattr(self, "_gun_source_sha256", None):
            raise ValueError("Source-basis adjustment requires the replacement STL")
        self._gun_nominal_mount_transform = self._manifest_matrix(matrix, "source basis").copy()
        self._update_gun_mount_adjustment_transform(render=False)

    def gun_mount_adjustment(self) -> dict:
        axis = None if self._gun_mount_roll_axis_flange is None else np.asarray(self._gun_mount_roll_axis_flange, dtype=float)
        return {
            "roll_deg": float(self._gun_mount_roll_deg),
            "translation_mm": (np.asarray(self._gun_mount_translation_m, dtype=float) * 1000.0).tolist(),
            "rotation_axis_flange": None if axis is None else axis.tolist(),
            "coordinate_frame": "URDF flange",
            "status": self._gun_mount_adjustment_status,
            "source_sha256": getattr(self, "_gun_source_sha256", None),
            "geometry_preview": getattr(self, "_gun_geometry_preview", False),
            "T_flange_source": None if self._gun_actor_mount_transform is None else self._gun_actor_mount_transform.tolist(),
            "T_flange_source_nominal": None if self._gun_nominal_mount_transform is None else self._gun_nominal_mount_transform.tolist(),
        }

    def set_gun_mount_calibration_status(self, status: str) -> None:
        self._gun_mount_adjustment_status = str(status).upper()

    def gun_mount_wire_diagnostics(self) -> dict:
        """Compatibility accessor; synthetic endpoint diagnostics are removed."""
        return {"available": False, "reason": self._wire_display_status,
                "status": "DISABLED_NO_ORIGINAL_WIRE"}

    def real_wire_geometry_diagnostics(self) -> dict:
        """Inspect LIVE geometry only; never move endpoints or trust FK fallback as LIVE."""
        report = {"frame_id": "URDF base", "units": "m", "mode": "LIVE",
                  "tcp_source": self._live_tcp_source, "available": False,
                  "display_mode": "PREVIEW" if self.preview_active else "LIVE"}
        if self._live_q is None or self._wire_end_source is None:
            return {**report, "reason": "等待真实关节反馈及焊丝定义"}
        if not str(self._live_tcp_source).startswith("controller") or self._live_tcp_fk is None:
            return {**report, "reason": "TCP LIVE UNKNOWN；不能将本地FK替代为示教器实测TCP"}
        stamp = (self._live_tcp_topic_stamp if "tcp_pose_broadcaster" in self._live_tcp_source
                 else self._live_tcp_rtde_stamp)
        if stamp <= 0 or time.monotonic() - stamp > LIVE_TCP_MAX_AGE_SEC:
            return {**report, "reason": "控制器TCP数据已过期，实时几何诊断为UNKNOWN"}
        frames = self.model.link_transforms_in_base(self._live_q)
        T = frames["flange"] @ self._gun_actor_mount_transform
        start = (T @ np.r_[self._wire_start_source, 1.0])[:3]
        end = np.asarray(self._live_tcp_fk, dtype=float)
        delta = end - start
        length = float(np.linalg.norm(delta))
        axis = delta / length if length > 1e-12 else None
        nozzle = (None if self._wire_nozzle_source is None else
                  (T @ np.r_[self._wire_nozzle_source, 1.0])[:3])
        nozzle_axis = (None if self._wire_nozzle_axis_source is None else
                       T[:3, :3] @ self._wire_nozzle_axis_source)
        angle = None
        if axis is not None and nozzle_axis is not None and np.linalg.norm(nozzle_axis) > 1e-12:
            angle = float(np.rad2deg(np.arccos(np.clip(
                np.dot(axis, nozzle_axis / np.linalg.norm(nozzle_axis)), -1, 1))))
        visual_start = visual_end = None
        if not self.preview_active:
            visual_start = np.asarray(self._wire_line_source.GetPoint1())
            visual_end = np.asarray(self._wire_line_source.GetPoint2())
        return {**report, "available": True,
                "nozzle_tip_world": None if nozzle is None else nozzle.tolist(),
                "tcp_live_world": end.tolist(), "wire_start": start.tolist(),
                "wire_end": end.tolist(), "wire_length": length,
                "wire_axis": None if axis is None else axis.tolist(),
                "nozzle_axis_world": None if nozzle_axis is None else nozzle_axis.tolist(),
                "wire_vs_nozzle_axis_deg": angle,
                "start_to_nozzle_error_mm": None if nozzle is None else float(np.linalg.norm(start-nozzle)*1000),
                "display_end_to_live_tcp_error_mm": None if visual_end is None else float(np.linalg.norm(visual_end-end)*1000),
                "display_start_error_mm": None if visual_start is None else float(np.linalg.norm(visual_start-start)*1000),
                "fixed_collision_end_world": (T @ np.r_[self._wire_end_source, 1.0])[:3].tolist(),
                "cad_datum_is_physical_measurement": False}

    def _update_wire_diagnostic_display(self) -> None:
        report = self.real_wire_geometry_diagnostics()
        if not report["available"]:
            text = "TCP LIVE / 焊丝诊断：UNKNOWN\n" + report["reason"]
        else:
            def point(key):
                value = report[key]
                return "UNKNOWN" if value is None else "[" + ", ".join(f"{v*1000:.3f}" for v in value) + "] mm"
            text = (f"world = URDF base；显示模式 {report['display_mode']}；{report['tcp_source']}\n"
                    f"Nozzle tip: {point('nozzle_tip_world')}\nTCP LIVE: {point('tcp_live_world')}\n"
                    f"wire_start: {point('wire_start')}\nwire_end: {point('wire_end')}\n"
                    f"wire_length: {report['wire_length']*1000:.3f} mm；wire_axis: {report['wire_axis']}\n"
                    f"枪口起点偏差: {report['start_to_nozzle_error_mm']} mm；显示端点/TCP偏差: {report['display_end_to_live_tcp_error_mm']} mm\n"
                    f"焊丝/CAD枪嘴轴夹角: {report['wire_vs_nozzle_axis_deg']}°；CAD参考非实测。仅报告，不修正。")
        if self._wire_diagnostic_text.toPlainText() != text:
            self._wire_diagnostic_text.setPlainText(text)

    def gun_display_status(self) -> str:
        return (
            f"{self._gun_display_status}（J6 {self._gun_mount_roll_deg:+.1f}°，"
            f"flange Δxyz {np.asarray(self._gun_mount_translation_m) * 1000.0} mm）；"
            f"{self._wire_display_status}"
        )

    def gun_mount_status(self) -> str:
        """Short status string for the offline gun-adjustment page."""
        axis = self._gun_mount_roll_axis_flange
        axis_text = "—" if axis is None else "[" + ", ".join(f"{float(v):+.3f}" for v in axis) + "]"
        return f"J6轴 {axis_text}；角度 {self._gun_mount_roll_deg:+.1f}°；{self._gun_mount_adjustment_status}"

    def _clear_scene_actors(self) -> None:
        for actor in [getattr(self, "_gun_actor", None), getattr(self, "_wire_actor", None),
                      getattr(self, "_wire_collision_actor", None),
                      *getattr(self, "_gun_collision_actors", [])]:
            if actor is not None:
                self.renderer.RemoveActor(actor)
        self._gun_actor = None
        self._wire_actor = None
        self._gun_collision_actors = []
        self._wire_collision_actor = None
        self._wire_live_planning_error_mm = None
        self._gun_collision_mount_transforms = []
        for actor in getattr(self, "_workpiece_actors", []):
            self.renderer.RemoveActor(actor)
        for actor in getattr(self, "_workpiece_outline_actors", []):
            self.renderer.RemoveActor(actor)
        for actor in getattr(self, "_seam_actors", {}).values():
            self.renderer.RemoveActor(actor)
        if getattr(self, "_planned_tcp_actor", None) is not None:
            self.renderer.RemoveActor(self._planned_tcp_actor)
        for actor in getattr(self, "_planned_endpoint_actors", []):
            self.renderer.RemoveActor(actor)
        self._planned_endpoint_actors = []
        if getattr(self, "_workpiece_origin_actor", None) is not None:
            self.renderer.RemoveActor(self._workpiece_origin_actor)
        if getattr(self, "_workpiece_origin_label", None) is not None:
            self.renderer.RemoveActor(self._workpiece_origin_label)
        for actor in getattr(self, "_workpiece_origin_axes", []):
            self.renderer.RemoveActor(actor)
        self._workpiece_actors = []
        self._workpiece_outline_actors = []
        self._seam_actors = {}
        self._planned_tcp_actor = None
        self._planned_tcp_points = None
        self._workpiece_origin_actor = None
        self._workpiece_origin_label = None
        self._workpiece_origin_axes = []

    def _create_workpiece_origin_marker(self) -> None:
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(0.012)
        sphere.SetThetaResolution(20)
        sphere.SetPhiResolution(12)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        marker = vtk.vtkActor()
        marker.SetMapper(mapper)
        marker.GetProperty().SetColor(1.0, 0.15, 0.75)
        marker.GetProperty().SetAmbient(0.35)
        marker.GetProperty().SetDiffuse(0.70)
        self.renderer.AddActor(marker)
        self._workpiece_origin_actor = marker

        label = vtk.vtkBillboardTextActor3D()
        label.SetInput("工件原点")
        label.SetPosition(0.018, 0.0, 0.018)
        text = label.GetTextProperty()
        text.SetFontSize(15)
        text.SetBold(True)
        text.SetColor(1.0, 0.55, 0.90)
        text.SetBackgroundColor(0.05, 0.05, 0.05)
        text.SetBackgroundOpacity(0.70)
        self.renderer.AddActor(label)
        self._workpiece_origin_label = label

        # Small local frame arms make the point unambiguous when the origin is
        # near a plate edge.  They are annotations, not collision geometry.
        for color, endpoint in (
            ((1.0, 0.2, 0.2), (0.045, 0.0, 0.0)),
            ((0.2, 1.0, 0.2), (0.0, 0.045, 0.0)),
            ((0.2, 0.45, 1.0), (0.0, 0.0, 0.045)),
        ):
            line = vtk.vtkLineSource()
            line.SetPoint1(0.0, 0.0, 0.0)
            line.SetPoint2(*endpoint)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(line.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetLineWidth(3.0)
            self.renderer.AddActor(actor)
            self._workpiece_origin_axes.append(actor)

    def set_planned_tcp_path(self, points, frame: str = "base_link", follow_workpiece: bool = True) -> None:
        """Show a planned TCP path.

        ``frame='base_link'`` applies the verified Rz(180 deg) conversion from
        the MATLAB planner output; ``frame='base'`` uses the points unchanged.
        """
        self._planned_path_follow_workpiece = follow_workpiece
        for marker in getattr(self, "_planned_endpoint_actors", []):
            self.renderer.RemoveActor(marker)
        self._planned_endpoint_actors = []
        if getattr(self, "_planned_tcp_actor", None) is not None:
            self.renderer.RemoveActor(self._planned_tcp_actor)
            self._planned_tcp_actor = None
            self._planned_tcp_points = None
        if points is None:
            return
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 2:
            return
        if frame == "base_link":
            pts = base_link_to_base(pts)
        self._planned_tcp_points = pts
        self._preview_tcp_path_errors_mm = []
        self._preview_relative_errors_mm = []
        self._preview_relative_orientation_errors_deg = []
        actor = _polyline_actor(pts)
        actor.GetProperty().SetColor(*PLANNED_TCP_COLOR)
        actor.GetProperty().SetLineWidth(3.0)
        self.renderer.AddActor(actor)
        self._planned_tcp_actor = actor
        if follow_workpiece:
            self._apply_actor_workpiece_transform(actor)
        for p, title, color in ((pts[0], "Start", (0.1, 1.0, 0.3)), (pts[-1], "End", (0.3, 0.6, 1.0))):
            sphere = vtk.vtkSphereSource()
            sphere.SetCenter(*p)
            sphere.SetRadius(0.003)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())
            marker = vtk.vtkActor()
            marker.SetMapper(mapper)
            marker.GetProperty().SetColor(*color)
            label = vtk.vtkBillboardTextActor3D()
            label.SetInput(title)
            label.SetPosition(*p)
            label.GetTextProperty().SetFontSize(12)
            label.GetTextProperty().SetColor(*color)
            for item in (marker, label):
                self.renderer.AddActor(item)
                if follow_workpiece:
                    self._apply_actor_workpiece_transform(item)
                self._planned_endpoint_actors.append(item)
        self._apply_visibility()

    def clear_planned_tcp_path(self) -> None:
        self.set_planned_tcp_path(None)

    def scene_status(self) -> str:
        status = f"{self._scene_status} | {self._gun_display_status} | 工件位姿: {self._workpiece_pose_source}"
        if self.preview_active:
            status += f" | 预览环境: {len(self._preview_environment_actors)} 组件（固定快照）"
            if getattr(self, "_preview_environment_error", ""):
                status += " WARNING: " + self._preview_environment_error
            if getattr(self, "_preview_environment_coverage_warning", ""):
                status += " WARNING: " + self._preview_environment_coverage_warning
        else:
            status += f" | 环境: {self._environment_status}"
        error = self._wire_live_planning_error_mm
        if error is not None and error > 0.5:
            status += f" | LIVE/规划焊丝端点差 {error:.3f} mm：需确认同步规划 TCP"
        return status

    def _make_environment_actor(self, component):
        """Display the exact collider in its fixed base pose, never its AABB."""
        if component.geometry_type == "mesh":
            actor = _triangle_mesh_actor(component.vertices_m, component.faces + 1)
        else:
            if component.geometry_type == "box":
                source = vtk.vtkCubeSource()
                source.SetXLength(float(component.size_m[0]))
                source.SetYLength(float(component.size_m[1]))
                source.SetZLength(float(component.size_m[2]))
            elif component.geometry_type == "sphere":
                source = vtk.vtkSphereSource()
                source.SetRadius(component.radius_m)
                source.SetThetaResolution(32); source.SetPhiResolution(24)
            elif component.geometry_type == "cylinder":
                cylinder = vtk.vtkCylinderSource()
                cylinder.SetRadius(component.radius_m); cylinder.SetHeight(component.length_m)
                cylinder.SetResolution(48)
                # MATLAB collisionCylinder local axis is +Z; VTK uses +Y.
                rotation = vtk.vtkTransform(); rotation.RotateX(90)
                source = vtk.vtkTransformPolyDataFilter()
                source.SetTransform(rotation); source.SetInputConnection(cylinder.GetOutputPort())
            else:
                raise ValueError(f"unsupported environment collider: {component.geometry_type}")
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(source.GetOutputPort())
            actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.SetUserMatrix(_vtk_matrix(component.pose_base))
        actor.GetProperty().SetColor(0.47, 0.63, 0.72)
        actor.GetProperty().SetOpacity(0.20)
        actor.GetProperty().SetEdgeVisibility(component.geometry_type == "box")
        actor.GetProperty().SetEdgeColor(0.70, 0.84, 0.92)
        return actor

    def _replace_environment_actors(self, components, preview=False):
        attr = "_preview_environment_actors" if preview else "_environment_actors"
        for actor in getattr(self, attr, {}).values():
            self.renderer.RemoveActor(actor)
        actors = {}
        for component in components:
            actor = self._make_environment_actor(component)
            self.renderer.AddActor(actor)
            actors[component.name] = actor
        setattr(self, attr, actors)
        self._sync_environment_visibility()

    def reload_environment_geometry(self, path=None):
        """Read a new manifest binding; snapshot actors remain independent."""
        if path is not None:
            self._environment_path = Path(path)
        try:
            stat = self._environment_path.stat()
            self._environment_file_binding = (str(self._environment_path), stat.st_mtime_ns, stat.st_size)
            scene = EnvironmentGeometry(self._environment_path)
            self._environment_geometry = scene
            self._replace_environment_actors(scene.components)
            self._environment_status = f"{len(scene.components)} 组件"
            if scene.coverage_warning:
                self._environment_status += " WARNING: " + scene.coverage_warning
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._environment_geometry = None
            self._replace_environment_actors([])
            self._environment_status = "WARNING: " + str(exc)
        self.vtk_widget.GetRenderWindow().Render()
        return self.environment_display_diagnostics()

    def _check_environment_binding(self):
        now = time.monotonic()
        if now - self._environment_last_check < 1.0:
            return
        self._environment_last_check = now
        try:
            stat = self._environment_path.stat()
            binding = (str(self._environment_path), stat.st_mtime_ns, stat.st_size)
        except OSError:
            binding = (str(self._environment_path), None, None)
        if binding != self._environment_file_binding:
            self.reload_environment_geometry()
            self._environment_file_binding = binding

    def _sync_environment_visibility(self):
        visible = bool(self._visibility.get("environment", True))
        for actor in self._environment_actors.values():
            actor.SetVisibility(visible and not self.preview_active)
        for actor in self._preview_environment_actors.values():
            actor.SetVisibility(visible and self.preview_active)

    def environment_display_diagnostics(self):
        """Expose actual render transforms for offline MATLAB/HMI verification."""
        preview = self.preview_active
        components = (getattr(self, "_preview_environment_components", []) if preview else
                      (self._environment_geometry.components if self._environment_geometry else []))
        return {"frame_id": "base", "source": "preview_snapshot" if preview else str(self._environment_path),
                "status": (getattr(self, "_preview_environment_error", "") +
                           getattr(self, "_preview_environment_coverage_warning", "")) if preview else self._environment_status,
                "object_count": len(components),
                "objects": [{"name": c.name, "pose_base": c.pose_base.tolist(), "type": c.geometry_type} for c in components]}

    def _apply_actor_workpiece_transform(self, actor) -> None:
        """Apply the current display-only workpiece calibration to one actor."""
        if actor is None:
            return
        delta = np.asarray(self._workpiece_calibration_delta, dtype=float)
        self._set_actor_transform(actor, delta)

    @staticmethod
    def _set_actor_transform(actor, matrix: np.ndarray) -> None:
        if actor is None:
            return
        matrix = np.asarray(matrix, dtype=float)
        if np.allclose(matrix, np.eye(4), atol=1e-12):
            actor.SetUserTransform(None)
            return
        transform = vtk.vtkTransform()
        transform.SetMatrix(_vtk_matrix(matrix))
        actor.SetUserTransform(transform)

    def _apply_workpiece_transform(self) -> None:
        """Update all workpiece/seam/path display actors without moving UR5."""
        actors = list(self._workpiece_actors)
        actors.extend(self._workpiece_outline_actors)
        actors.extend(self._seam_actors.values())
        if self._workpiece_origin_actor is not None:
            actors.append(self._workpiece_origin_actor)
        if self._workpiece_origin_label is not None:
            actors.append(self._workpiece_origin_label)
        actors.extend(self._workpiece_origin_axes)
        if self._planned_tcp_actor is not None and getattr(self, "_planned_path_follow_workpiece", True):
            actors.append(self._planned_tcp_actor)
            actors.extend(getattr(self, "_planned_endpoint_actors", []))
        for actor in actors:
            self._apply_actor_workpiece_transform(actor)
        if self._scene is not None:
            # Origin marker/axes are authored in the local Workpiece frame,
            # whereas boxes/seams are already authored in base coordinates.
            # Therefore the marker uses the full current base<-workpiece pose.
            origin_pose = self._workpiece_calibration_delta @ self._scene.workpiece_pose_base()
            for actor in [self._workpiece_origin_actor, self._workpiece_origin_label, *self._workpiece_origin_axes]:
                self._set_actor_transform(actor, origin_pose)
        if hasattr(self, "vtk_widget"):
            self.vtk_widget.GetRenderWindow().Render()

    def set_workpiece_calibration(self, T_base_workpiece_real) -> None:
        """Show a real three-point workpiece pose relative to the nominal scene.

        ``T_base_workpiece_real`` is the measured workpiece frame in the URDF
        ``base`` frame.  The nominal exported scene remains untouched; VTK
        receives only the relative display transform so seams and planned
        paths move together with the workpiece.
        """
        if self._scene is None:
            raise RuntimeError("scene geometry is unavailable")
        measured = np.asarray(T_base_workpiece_real, dtype=float)
        if measured.shape != (4, 4) or not np.all(np.isfinite(measured)):
            raise ValueError("workpiece calibration transform must be finite 4x4")
        nominal = self._scene.workpiece_pose_base()
        self._workpiece_calibration_delta = measured @ np.linalg.inv(nominal)
        self._workpiece_pose_source = "REAL CALIBRATION"
        self._apply_workpiece_transform()

    def set_workpiece_calibration_preview(self, T_base_workpiece_real) -> None:
        """Show a saved real pose for diagnostics without enabling planning.

        This path is intentionally separate from the approved REAL
        CALIBRATION source: a draft or poor-quality result can be inspected in
        3D, while the planning/application gate remains blocked.
        """
        if self._scene is None:
            raise RuntimeError("scene geometry is unavailable")
        measured = np.asarray(T_base_workpiece_real, dtype=float)
        if measured.shape != (4, 4) or not np.all(np.isfinite(measured)):
            raise ValueError("workpiece calibration preview must be finite 4x4")
        nominal = self._scene.workpiece_pose_base()
        self._workpiece_calibration_delta = measured @ np.linalg.inv(nominal)
        self._workpiece_pose_source = "REAL CALIBRATION PREVIEW"
        self._apply_workpiece_transform()

    def clear_workpiece_calibration(self) -> None:
        """Restore the exported simulation workpiece pose in the display."""
        self._workpiece_calibration_delta = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._apply_workpiece_transform()

    def set_visibility(self, **flags) -> None:
        for key, value in flags.items():
            if key in self._visibility:
                self._visibility[key] = bool(value)
        self._apply_visibility()

    def visibility(self) -> dict:
        return dict(self._visibility)

    def _apply_visibility(self) -> None:
        self._sync_environment_visibility()
        for actor, _origin in self._actors.values():
            actor.SetVisibility(bool(self._visibility["robot"]))
        for label in self._joint_labels.values():
            label[0].SetVisibility(bool(self._visibility["robot"]))
        if hasattr(self, "_axes"):
            self._axes.SetVisibility(bool(self._visibility["robot"]))
        for actor in self._workpiece_actors:
            actor.SetVisibility(bool(self._visibility["workpiece"]))
        if self._workpiece_origin_actor is not None:
            self._workpiece_origin_actor.SetVisibility(bool(self._visibility["workpiece"]))
        if self._workpiece_origin_label is not None:
            self._workpiece_origin_label.SetVisibility(bool(self._visibility["workpiece"]))
        for actor in self._workpiece_origin_axes:
            actor.SetVisibility(bool(self._visibility["workpiece"]))
        # Collision geometry is currently the same four workpiece boxes that
        # define the collision model, so both switches drive the same actors.
        collision_on = bool(self._visibility["workpiece"]) and bool(self._visibility["collision"])
        for actor in self._workpiece_actors:
            actor.GetProperty().SetOpacity(WORKPIECE_OPACITY if collision_on else 0.0)
        seam_on = bool(self._visibility["seam"])
        for name, actor in self._seam_actors.items():
            actor.SetVisibility(seam_on and (name == "A"))
        if self._planned_tcp_actor is not None:
            self._planned_tcp_actor.SetVisibility(bool(self._visibility["tcp_path"]))
        for marker in getattr(self, "_planned_endpoint_actors", []):
            marker.SetVisibility(bool(self._visibility["tcp_path"]))
        if self._gun_actor is not None:
            self._gun_actor.SetVisibility(bool(self._visibility.get("gun", True)))
        for actor in getattr(self, "_gun_collision_actors", []):
            actor.SetVisibility(bool(self._visibility.get("gun", True)) and bool(self._visibility.get("collision", True)))
        if self._wire_actor is not None:
            self._wire_actor.SetVisibility(self._wire_end_source is not None
                                           and bool(self._visibility.get("wire", True)))
        if self._wire_collision_actor is not None:
            self._wire_collision_actor.SetVisibility(self._wire_end_source is not None
                                                     and bool(self._visibility.get("wire", True))
                                                     and bool(self._visibility.get("collision", True)))
        if self._wire_tip_actor is not None:
            self._wire_tip_actor.SetVisibility(bool(self._visibility.get("wire", True)))
        if self._wire_tip_label is not None:
            self._wire_tip_label.SetVisibility(bool(self._visibility.get("wire", True)))
        if self._wire_tcp_error_line_actor is not None:
            self._wire_tcp_error_line_actor.SetVisibility(False)
        if self._wire_tcp_error_label is not None:
            self._wire_tcp_error_label.SetVisibility(False)
        self.vtk_widget.GetRenderWindow().Render()

    def preview_point_base(self, index: int):
        """Return the planned TCP sample at ``index`` in the URDF base frame."""
        if self._planned_tcp_points is None or len(self._planned_tcp_points) == 0:
            return None
        idx = int(np.clip(index, 0, len(self._planned_tcp_points) - 1))
        point = np.r_[self._planned_tcp_points[idx], 1.0]
        return (self._workpiece_calibration_delta @ point)[:3]

    def scene_bbox(self):
        """Bounds of the robot meshes plus the workpiece, for framing."""
        bounds = []
        for actor, _origin in self._actors.values():
            current = actor.GetBounds()
            if current and len(current) == 6 and all(np.isfinite(current)):
                bounds.append(np.asarray(current, dtype=float))
        for actor in self._workpiece_actors:
            current = actor.GetBounds()
            if current and len(current) == 6 and all(np.isfinite(current)):
                bounds.append(np.asarray(current, dtype=float))
        if not bounds:
            return None
        stacked = np.vstack(bounds)
        return stacked.min(axis=0), stacked.max(axis=1)

    def fit_scene_view(self, view_name: str = "iso") -> None:
        """Frame UR5 + workpiece using mesh bounds only.

        Labels, axes, TCP helpers, seams and planned paths are deliberately not
        part of the fit bounds.  The explicit zoom is applied after the
        bounds-based framing so the physical model fills the viewport without
        changing any geometry or coordinate transform.
        """
        bounds = self._mesh_bounds(include_workpiece=True)
        center, radius = self._mesh_center_radius(include_workpiece=True)
        camera = self.renderer.GetActiveCamera()
        if bounds is not None:
            # Explicit bounds exclude labels, axes and helper actors.  The
            # custom orientation below then preserves the requested view.
            self.renderer.ResetCamera([float(value) for value in bounds])
        camera.SetViewAngle(self._default_view_angle)
        view_angle = max(self._default_view_angle, 20.0)
        half_angle = np.deg2rad(view_angle * 0.5)
        # The explicit post-fit zoom is intentionally stronger than VTK's
        # default ResetCamera framing.  The combined robot+workpiece view is
        # wide, so 3.5x fills roughly 70--80% of the viewport height while
        # retaining a small margin for labels and interaction.  This changes
        # only the camera, never the scene geometry or coordinate transforms.
        target_fraction = 0.50
        distance = radius / max(target_fraction * np.tan(half_angle), 1e-3)
        distance = max(distance * 1.02, 0.9)
        directions = {
            "iso": (np.array([0.92, -1.12, 0.78]), (0.0, 0.0, 1.0)),
            "top": (np.array([0.0, 0.0, 1.0]), (0.0, 1.0, 0.0)),
            "front": (np.array([1.0, 0.0, 0.0]), (0.0, 0.0, 1.0)),
            "side": (np.array([0.0, 1.0, 0.0]), (0.0, 0.0, 1.0)),
        }
        direction, view_up = directions.get(view_name, directions["iso"])
        direction = direction / max(float(np.linalg.norm(direction)), 1e-9)
        camera.SetFocalPoint(*[float(v) for v in center])
        camera.SetPosition(*[float(v) for v in center + direction * distance])
        camera.SetViewUp(*view_up)
        camera.Zoom(3.5)
        self.renderer.ResetCameraClippingRange()
        self.vtk_widget.GetRenderWindow().Render()
        self._camera_initialized = True

    def _fk_tcp_from_flange(self, q):
        """Return (base->flange, base TCP) using one fixed flange offset."""
        if self._tcp_offset_flange is None or q is None or len(q) != 6:
            return None, None
        transforms = self.model.link_transforms_in_base(q)
        flange = transforms.get("flange")
        if flange is None:
            return None, None
        tcp = (flange @ np.r_[self._tcp_offset_flange, 1.0])[:3]
        return flange, tcp

    def _select_live_tcp(self) -> None:
        """Select the red LIVE marker source without touching preview state."""
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
        self._refresh()  # Red marker and wire update together from this snapshot.

    def set_live_state(self, q, tcp_position=None, tcp_frame_id="", tcp_stamp=0.0) -> None:
        """Update only the live ROS snapshot; preview state is untouched."""
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
        """Backward-compatible alias for callers outside the HMI."""
        self.set_live_state(q, tcp_position, tcp_frame_id, tcp_stamp)

    def set_calibrated_tcp_offset(self, offset_flange_m) -> None:
        """Set the fixed display-only flange->TCP translation.

        This is shared by live and preview FK, but is never changed by a
        preview frame.  The calibration file itself is not written here.
        """
        offset = np.asarray(offset_flange_m, dtype=float).reshape(-1)
        if offset.size != 3 or not np.all(np.isfinite(offset)):
            raise ValueError("calibrated TCP offset must be a finite 3-vector")
        self._calibrated_tcp_offset = offset.copy()
        self._tcp_offset_flange = offset.copy()
        # Re-read the applied record after an explicit calibration adoption.
        # The offset argument may be a draft display preview: never use it
        # to silently change the physical wire/planning definition.
        self._refresh_model_axis_wire()
        self._select_live_tcp()

    set_display_tcp_offset = set_calibrated_tcp_offset

    def set_planning_tcp_transform(self, transform) -> None:
        """Mirror the complete controller-derived flange TCP for local preview.

        LIVE TCP remains the separately stored ROS/RTDE reference.  This
        setter never changes its pose, the live/preview joint snapshots, or
        the gun mounting transform; only the fixed preview TCP is updated.
        """
        matrix = _validated_tcp_transform(transform)
        self._planning_tcp_transform = matrix
        self._tcp_rotation_flange = matrix[:3, :3].copy()
        self._calibrated_tcp_offset = matrix[:3, 3].copy()
        self._tcp_offset_flange = matrix[:3, 3].copy()
        if self._preview_q is not None:
            self._preview_tcp_fk = self._preview_tcp_from_snapshot()
        self._refresh_model_axis_wire()
        self._refresh()

    def clear_calibrated_tcp_offset(self) -> None:
        self._calibrated_tcp_offset = None
        self._tcp_offset_flange = None
        self._planning_tcp_transform = None
        self._tcp_rotation_flange = (np.eye(3, dtype=float)
                                     if self._flange_tcp_nominal_transform is None
                                     else self._flange_tcp_nominal_transform[:3, :3].copy())
        self._wire_end_source = None
        self._wire_live_planning_error_mm = None
        for actor in (self._wire_actor, self._wire_collision_actor):
            if actor is not None:
                actor.SetVisibility(False)
        self._select_live_tcp()
        self._preview_tcp_fk = None
        for actor in (self._calibrated_tcp_actor, self._calibrated_tcp_line_actor, self._calibrated_tcp_label):
            if actor is not None:
                actor.SetVisibility(False)

    def set_preview_state(self, q, tcp_index: int | None = None) -> None:
        """Set the preview pose without mutating live q/TCP state."""
        if q is not None and len(q) == 6:
            self._preview_q = list(q)
            self.preview_active = True
            self._preview_tcp_index = tcp_index
            self._preview_tcp_fk = self._preview_tcp_from_snapshot()
            if tcp_index is not None and self._preview_tcp_fk is not None:
                point = self._planned_tcp_raw_point_base(tcp_index)
                if point is not None:
                    self._preview_tcp_path_errors_mm.append(
                        float(np.linalg.norm(self._preview_tcp_fk - point) * 1000.0)
                    )
            self._check_preview_relative_transform()
            self.q = list(self._preview_q)
            # Atomic robot + flange + tool refresh, not a later independent tick.
            self._refresh()

    def clear_preview_state(self) -> None:
        self.preview_active = False
        self._preview_q = None
        self._preview_tcp_fk = None
        self._preview_tcp_index = None
        self._preview_tcp_path_errors_mm = []
        self._preview_relative_errors_mm = []
        self._preview_relative_orientation_errors_deg = []
        for actor in getattr(self, "_preview_tool_actors", []):
            actor.SetVisibility(False)
        self._sync_environment_visibility()
        if self._live_q is not None:
            self.q = list(self._live_q)

    def _planned_tcp_raw_point_base(self, index: int):
        if self._planned_tcp_points is None or len(self._planned_tcp_points) == 0:
            return None
        idx = int(np.clip(index, 0, len(self._planned_tcp_points) - 1))
        return np.asarray(self._planned_tcp_points[idx], dtype=float)

    def _check_preview_relative_transform(self) -> None:
        if self._preview_q is None or self._tcp_offset_flange is None:
            return
        try:
            transforms = self.model.link_transforms_in_base(self._preview_q)
            flange = transforms["flange"]
            T_expected = np.eye(4, dtype=float)
            T_expected[:3, :3] = self._tcp_rotation_flange
            T_expected[:3, 3] = self._tcp_offset_flange
            snapshot = getattr(self, "_preview_scene_snapshot", None)
            if snapshot is not None:
                T_expected = snapshot["T_flange_to_tcp"]
            T_tcp = np.eye(4, dtype=float)
            T_tcp[:3, :3] = flange[:3, :3] @ T_expected[:3, :3]
            T_tcp[:3, 3] = self._preview_tcp_fk
            T_actual = np.linalg.inv(flange) @ np.block([
                [T_tcp[:3, :3], T_tcp[:3, 3].reshape(3, 1)],
                [np.zeros((1, 3)), np.ones((1, 1))],
            ])
            self._preview_relative_errors_mm.append(
                float(np.linalg.norm(T_actual[:3, 3] - T_expected[:3, 3]) * 1000.0)
            )
            R_error = T_actual[:3, :3] @ T_expected[:3, :3].T
            angle = np.clip((np.trace(R_error) - 1.0) * 0.5, -1.0, 1.0)
            self._preview_relative_orientation_errors_deg.append(float(np.degrees(np.arccos(angle))))
        except Exception:
            pass

    def tcp_preview_diagnostics(self) -> dict:
        path_errors = np.asarray(self._preview_tcp_path_errors_mm, dtype=float)
        rel_errors = np.asarray(self._preview_relative_errors_mm, dtype=float)
        rel_rot = np.asarray(self._preview_relative_orientation_errors_deg, dtype=float)
        return {
            "frame_count": int(max(len(path_errors), len(rel_errors))),
            "max_tcp_path_error_mm": float(np.max(path_errors)) if path_errors.size else None,
            "max_flange_tcp_relative_error_mm": float(np.max(rel_errors)) if rel_errors.size else None,
            "max_flange_tcp_relative_orientation_error_deg": float(np.max(rel_rot)) if rel_rot.size else None,
            "tcp_offset_flange_m": None if self._tcp_offset_flange is None else self._tcp_offset_flange.tolist(),
            "T_flange_tcp": (None if self._tcp_offset_flange is None else
                             np.block([[self._tcp_rotation_flange, self._tcp_offset_flange.reshape(3, 1)],
                                       [np.zeros((1, 3)), np.ones((1, 1))]]).tolist()),
            "offset_source": "fixed display calibration" if self._tcp_offset_flange is not None else "unavailable",
            "live_tcp_source": self._live_tcp_source,
            "live_tcp_topic_age_sec": (
                None if self._live_tcp_topic_stamp <= 0
                else max(0.0, float(time.monotonic() - self._live_tcp_topic_stamp))
            ),
        }

    def set_urdf_text(self, urdf_text: str) -> None:
        try:
            self.model = UrdfKinematicModel(urdf_text=urdf_text)
            self._load_meshes()
            self._load_scene_geometry()
            self.renderer.ResetCamera()
        except Exception:
            # Preserve the already loaded calibrated model on malformed input.
            pass

    def _mesh_center_radius(self, include_workpiece: bool = False):
        """Return bounds of robot meshes only, excluding labels/axes/TCP marker.

        The previous implementation used ``ComputeVisiblePropBounds``.  That
        also includes the axes and text actors, so the camera could zoom to an
        unexpected scale and crop the robot at high-DPI window sizes.

        ``include_workpiece`` additionally includes the read-only workpiece
        boxes so a combined robot+workpiece framing is possible.  The default
        keeps the historical robot-only framing.
        """
        bounds = self._mesh_bounds(include_workpiece=include_workpiece)
        if bounds is None:
            center = np.array([0.0, -0.15, 0.48], dtype=float)
            return center, 0.55
        center = np.array([
            0.5 * (bounds[0] + bounds[1]),
            0.5 * (bounds[2] + bounds[3]),
            0.5 * (bounds[4] + bounds[5]),
        ], dtype=float)
        extent = np.array([bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]])
        radius = max(0.5 * float(np.linalg.norm(extent)), 0.1)
        return center, radius

    def _mesh_bounds(self, include_workpiece: bool = False):
        """Return only robot/workpiece mesh bounds for camera framing."""
        bounds = None
        sources = list(self._actors.values())
        if include_workpiece:
            sources += [(actor, None) for actor in self._workpiece_actors]
        for actor, _origin in sources:
            current = actor.GetBounds()
            if current is None or len(current) != 6:
                continue
            if any(not np.isfinite(value) for value in current):
                continue
            if bounds is None:
                bounds = np.asarray(current, dtype=float)
            else:
                bounds[0] = min(bounds[0], current[0])
                bounds[1] = max(bounds[1], current[1])
                bounds[2] = min(bounds[2], current[2])
                bounds[3] = max(bounds[3], current[3])
                bounds[4] = min(bounds[4], current[4])
                bounds[5] = max(bounds[5], current[5])
        if bounds is None or bounds[0] >= bounds[1] or bounds[2] >= bounds[3] or bounds[4] >= bounds[5]:
            return None
        return bounds

    def fit_robot_view(self, view_name: str = "iso") -> None:
        """Frame only the UR5 meshes using bounds that exclude annotations.

        The final camera zoom is deliberately shared with the combined scene
        view so the robot remains legible when switching between the two
        framing modes.
        """
        bounds = self._mesh_bounds()
        center, radius = self._mesh_center_radius()
        camera = self.renderer.GetActiveCamera()
        if bounds is not None:
            self.renderer.ResetCamera([float(value) for value in bounds])
        camera.SetViewAngle(self._default_view_angle)
        view_angle = max(self._default_view_angle, 20.0)
        half_angle = np.deg2rad(view_angle * 0.5)
        target_fraction = 0.50
        distance = radius / max(target_fraction * np.tan(half_angle), 1e-3)
        distance = max(distance * 1.02, 0.9)
        directions = {
            "iso": (np.array([0.92, -1.12, 0.78]), (0.0, 0.0, 1.0)),
            "top": (np.array([0.0, 0.0, 1.0]), (0.0, 1.0, 0.0)),
            "front": (np.array([1.0, 0.0, 0.0]), (0.0, 0.0, 1.0)),
            "side": (np.array([0.0, 1.0, 0.0]), (0.0, 0.0, 1.0)),
        }
        direction, view_up = directions.get(view_name, directions["iso"])
        direction = direction / max(float(np.linalg.norm(direction)), 1e-9)
        camera.SetFocalPoint(*[float(value) for value in center])
        camera.SetPosition(*[float(value) for value in center + direction * distance])
        camera.SetViewUp(*view_up)
        camera.Zoom(3.5)
        self.renderer.ResetCameraClippingRange()
        self.vtk_widget.GetRenderWindow().Render()
        self._camera_initialized = True

    def fit_gun_view(self, view_name: str = "iso") -> None:
        """Frame only gun, wire and TCP helpers for installation inspection."""
        actors = [a for a in (self._gun_actor, self._wire_actor, self._tcp_actor) if a is not None and a.GetVisibility()]
        bounds = None
        for actor in actors:
            current = actor.GetBounds()
            if current is None or len(current) != 6 or any(not np.isfinite(v) for v in current):
                continue
            if bounds is None:
                bounds = np.asarray(current, dtype=float)
            else:
                bounds[0] = min(bounds[0], current[0]); bounds[1] = max(bounds[1], current[1])
                bounds[2] = min(bounds[2], current[2]); bounds[3] = max(bounds[3], current[3])
                bounds[4] = min(bounds[4], current[4]); bounds[5] = max(bounds[5], current[5])
        if bounds is None:
            return
        center = np.array([
            0.5 * (bounds[0] + bounds[1]),
            0.5 * (bounds[2] + bounds[3]),
            0.5 * (bounds[4] + bounds[5]),
        ])
        radius = max(0.5 * float(np.linalg.norm([bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]])), 0.08)
        direction, view_up = {
            "iso": (np.array([0.9, -1.0, 0.75]), np.array([0.0, 0.0, 1.0])),
            "top": (np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0])),
            "front": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
            "side": (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        }.get(view_name, (np.array([0.9, -1.0, 0.75]), np.array([0.0, 0.0, 1.0])))
        direction = direction / max(float(np.linalg.norm(direction)), 1e-9)
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(*center)
        # Perspective-aware fit; a constant distance factor cropped the gun
        # when the installation inspector was zoomed in.
        half_angle = np.deg2rad(self._default_view_angle * 0.5)
        distance = radius / max(0.75 * np.tan(half_angle), 1e-3)
        camera.SetPosition(*(center + direction * max(distance, 0.25)))
        camera.SetViewUp(*view_up)
        camera.SetViewAngle(self._default_view_angle)
        camera.Zoom(1.0)
        self.renderer.ResetCameraClippingRange()
        self.vtk_widget.GetRenderWindow().Render()
        self._camera_initialized = True

    def set_view(self, view_name: str) -> None:
        """Set a predictable camera view without changing robot state."""
        self.fit_robot_view("iso" if view_name == "reset" else view_name)

    def set_view_with_scene(self, view_name: str) -> None:
        """Camera view that includes the workpiece when it is visible."""
        if self._visibility.get("workpiece", True):
            self.fit_scene_view("iso" if view_name == "reset" else view_name)
        else:
            self.fit_robot_view("iso" if view_name == "reset" else view_name)

    def _install_preview_tool_actors(self):
        """Separate snapshot actors; live calibration and installation stay intact."""
        for actor in getattr(self, "_preview_tool_actors", []):
            self.renderer.RemoveActor(actor)
        self._preview_tool_actors = []
        self._preview_gun_visual = None
        self._preview_gun_hulls = []
        self._preview_wire_layers = []
        self._preview_visual_pair_cache = {}
        self._preview_surface_filters = []
        data = getattr(self, "_preview_scene_snapshot", None)
        self._replace_environment_actors(getattr(self, "_preview_environment_components", []), preview=True)
        if data is None:
            return
        visual = Path(str(data.get("visual_path", "")))
        import hashlib
        if visual.is_file() and hashlib.sha256(visual.read_bytes()).hexdigest() == data.get("visual_sha256"):
            reader = vtk.vtkSTLReader(); reader.SetFileName(str(visual)); reader.Update()
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(reader.GetOutputPort())
            actor = vtk.vtkActor(); actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.35, 0.37, 0.39)
            self._preview_gun_visual = actor
            self._preview_tool_actors.append(actor)
        else:
            self._preview_snapshot_error = "预览 STL 缺失/哈希不符，仅显示保存的碰撞几何"
        components = data.get("components", [])
        if isinstance(components, dict): components = [components]
        for component in components:
            vertices = np.asarray(component["vertices_m"], dtype=float)
            actor = _triangle_mesh_actor(vertices, ConvexHull(vertices).simplices + 1)
            actor.GetProperty().SetColor(*GUN_COLLISION_COLOR)
            actor.GetProperty().SetOpacity(0.2)
            actor.GetProperty().SetEdgeVisibility(True)
            self._preview_gun_hulls.append(actor); self._preview_tool_actors.append(actor)
        for opacity in (1.0, 0.35):
            line = vtk.vtkLineSource(); tube = vtk.vtkTubeFilter()
            tube.SetInputConnection(line.GetOutputPort()); tube.SetRadius(float(data["wire_diameter_m"])/2)
            tube.SetNumberOfSides(20); tube.CappingOn()
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(tube.GetOutputPort())
            actor = vtk.vtkActor(); actor.SetMapper(mapper)
            actor.GetProperty().SetColor(1.0, 0.85, 0.1); actor.GetProperty().SetOpacity(opacity)
            self._preview_wire_layers.append((actor, line)); self._preview_tool_actors.append(actor)
        for actor in self._preview_tool_actors:
            self.renderer.AddActor(actor); actor.SetVisibility(False)
        if self._preview_gun_visual is not None:
            skip_rows = data.get("self_collision_skip_pairs", [])
            if skip_rows and isinstance(skip_rows[0], str):
                # Legacy MATLAB jsonencode flattens a cell Nx2 column-major.
                count = len(skip_rows)//2
                skip_rows = list(zip(skip_rows[:count], skip_rows[count:]))
            skip = {frozenset(pair) for pair in skip_rows}
            for name, (actor, _) in self._actors.items():
                if frozenset((name, "WeldingGunMount")) in skip:
                    continue
                surface = vtk.vtkCollisionDetectionFilter()
                surface.SetInputData(0, self._preview_gun_visual.GetMapper().GetInput())
                surface.SetInputData(1, actor.GetMapper().GetInput())
                surface.SetCollisionModeToFirstContact()
                self._preview_surface_filters.append((name, actor, surface))

    def _refresh_preview_tool_actors(self, transforms):
        if not self.preview_active or getattr(self, "_preview_scene_snapshot", None) is None:
            for actor in getattr(self, "_preview_tool_actors", []): actor.SetVisibility(False)
            return False
        d = self.preview_frame_diagnostics(transforms)
        # No live gun, stale user adjustment or live wire may leak into preview.
        for actor in [self._gun_actor, self._wire_actor, self._wire_collision_actor,
                      *getattr(self, "_gun_collision_actors", [])]:
            if actor is not None: actor.SetVisibility(False)
        self._wire_start_marker.SetVisibility(False)
        visible = bool(self._visibility.get("gun", True))
        if self._preview_gun_visual is not None:
            self._preview_gun_visual.SetUserMatrix(_vtk_matrix(d["gun_pose"]))
            self._preview_gun_visual.SetVisibility(visible)
        for actor in self._preview_gun_hulls:
            actor.SetUserMatrix(_vtk_matrix(d["gun_pose"]))
            actor.SetVisibility(visible and bool(self._visibility.get("collision", True)))
        for i, (actor, line) in enumerate(self._preview_wire_layers):
            a = d["gun_tip_world"] if i == 0 else d["collision_start"]
            b = d["wire_tip_world"] if i == 0 else d["collision_end"]
            line.SetPoint1(*a); line.SetPoint2(*b); line.Update()
            actor.SetVisibility(bool(self._visibility.get("wire", True)) and
                                (i == 0 or bool(self._visibility.get("collision", True))))
        index = int(self._preview_tcp_index or 0)
        if index not in self._preview_visual_pair_cache:
            pairs = []
            # Read-only mesh-surface diagnostics fill the missing-coverage
            # warning with observable pairs. These are NEVER a planning PASS.
            for name, actor, surface in self._preview_surface_filters:
                surface.SetMatrix(0, _vtk_matrix(d["gun_pose"]))
                surface.SetMatrix(1, actor.GetUserMatrix()); surface.Update()
                if surface.GetNumberOfContacts(): pairs.append(name + " <-> WeldingGun_visual")
            self._preview_visual_pair_cache[index] = pairs
        return True

    def _refresh(self) -> None:
        self._check_environment_binding()
        self._sync_environment_visibility()
        self._wire_start_marker.SetVisibility(False)
        display_q = self._preview_q if self.preview_active else self._live_q
        if display_q is None or len(display_q) != 6:
            self._update_wire_diagnostic_display()
            return
        try:
            transforms = self.model.link_transforms_in_base(display_q)
        except Exception:
            return
        for link_name, (actor, origin) in self._actors.items():
            if link_name not in transforms:
                continue
            actor.SetUserMatrix(_vtk_matrix(transforms[link_name] @ origin))
        for joint_name, (label, child_link) in self._joint_labels.items():
            if child_link not in transforms:
                continue
            position = transforms[child_link][:3, 3] + np.array([0.018, 0.0, 0.018])
            label.SetPosition(*[float(value) for value in position])
        if not self._camera_initialized:
            self.fit_scene_view("iso")
        # Draw the live TCP independently from the preview TCP.  Both points
        # are FK results from their own q snapshot and the same fixed
        # flange->TCP offset; the ROS TCP topic is retained only for diagnostics.
        live_transforms = None
        if self._live_q is not None:
            try:
                live_transforms = self.model.link_transforms_in_base(self._live_q)
            except Exception:
                live_transforms = None
        self._draw_tcp_marker(
            self._tcp_actor,
            self._tcp_line_actor,
            self._tcp_line_source,
            self._tcp_label,
            live_transforms,
            self._live_tcp_fk,
            self._live_tcp_label_text(),
            LIVE_TCP_COLOR,
            self._visibility.get("current_tcp", True),
        )
        preview_transforms = transforms if self.preview_active else None
        self._draw_tcp_marker(
            self._preview_tcp_actor,
            self._preview_tcp_line_actor,
            self._preview_tcp_line_source,
            self._preview_tcp_label,
            preview_transforms,
            self._preview_tcp_fk,
            "TCP PREVIEW",
            PREVIEW_TCP_COLOR,
            self.preview_active and self._visibility.get("preview_tcp", True),
        )
        # The old cyan calibration annotation is retained as an explicit
        # diagnostic layer only; it is no longer used as the live marker.
        for actor in (self._calibrated_tcp_actor, self._calibrated_tcp_line_actor, self._calibrated_tcp_label):
            if actor is not None:
                actor.SetVisibility(False)
        if self._gun_actor is not None and self._gun_actor_mount_transform is not None and "flange" in transforms:
            self._gun_actor.SetUserMatrix(_vtk_matrix(transforms["flange"] @ self._gun_actor_mount_transform))
            self._gun_actor.SetVisibility(bool(self._visibility.get("gun", True)))
        for index, actor in enumerate(getattr(self, "_gun_collision_actors", [])):
            mount_transform = (self._gun_collision_mount_transforms[index]
                               if index < len(self._gun_collision_mount_transforms)
                               else self._gun_actor_mount_transform)
            if mount_transform is not None and "flange" in transforms:
                actor.SetUserMatrix(_vtk_matrix(transforms["flange"] @ mount_transform))
                actor.SetVisibility(bool(self._visibility.get("gun", True)) and bool(self._visibility.get("collision", True)))
        if self._wire_actor is not None and hasattr(self, "_wire_line_source"):
            # Visual endpoint is exactly the marker coordinate. Planning
            # collision geometry remains fixed and separately displayed.
            wire_frames = transforms if self.preview_active else live_transforms
            visible = (wire_frames is not None and "flange" in wire_frames
                       and self._wire_end_source is not None)
            if visible:
                T = wire_frames["flange"] @ self._gun_actor_mount_transform
                start = (T @ np.r_[self._wire_start_source, 1.0])[:3]
                planning_endpoint = (T @ np.r_[self._wire_end_source, 1.0])[:3]
                endpoint = self._preview_tcp_fk if self.preview_active else self._live_tcp_fk
                self._wire_collision_line_source.SetPoint1(*start)
                self._wire_collision_line_source.SetPoint2(*planning_endpoint)
                self._wire_collision_line_source.Update()
                self._wire_live_planning_error_mm = (
                    None if endpoint is None else float(np.linalg.norm(np.asarray(endpoint) - planning_endpoint) * 1000))
                visible = endpoint is not None
            if visible:
                self._wire_line_source.SetPoint1(*start)
                self._wire_line_source.SetPoint2(*endpoint)
                self._wire_line_source.Update()
                self._wire_start_marker.SetPosition(*start)
                self._wire_start_marker.SetVisibility(bool(self._visibility.get("wire", True)))
            self._wire_actor.SetVisibility(visible and bool(self._visibility.get("wire", True)))
            if self._wire_collision_actor is not None:
                self._wire_collision_actor.SetVisibility(
                    wire_frames is not None and "flange" in wire_frames
                    and self._wire_end_source is not None
                    and bool(self._visibility.get("wire", True))
                    and bool(self._visibility.get("collision", True)))
        self._refresh_preview_tool_actors(transforms)
        self._update_wire_diagnostic_display()
        self.vtk_widget.GetRenderWindow().Render()

    def _live_tcp_label_text(self) -> str:
        """Make the red marker's source explicit without changing its data."""
        source = str(self._live_tcp_source)
        if source.startswith("controller RTDE actual_TCP_pose"):
            return "TCP LIVE [RTDE]"
        if source.startswith("controller tcp_pose_broadcaster/pose"):
            return "TCP LIVE [CTRL]"
        if source.startswith("HMI flange FK fallback"):
            return "TCP LIVE [HMI FK]"
        return "TCP LIVE [UNKNOWN]"

    @staticmethod
    def _draw_tcp_marker(marker, line_actor, line_source, label, transforms, point, text, color, visible):
        if marker is None or line_actor is None or line_source is None or label is None:
            return
        if transforms is None or point is None or not visible:
            marker.SetVisibility(False)
            line_actor.SetVisibility(False)
            label.SetVisibility(False)
            return
        p = np.asarray(point, dtype=float).reshape(3)
        flange = np.asarray(transforms["flange"][:3, 3], dtype=float) if "flange" in transforms else p
        marker.SetPosition(*[float(v) for v in p])
        label.SetInput(text)
        label.SetPosition(*[float(v) for v in (p + np.array([0.025, 0.0, 0.025]))])
        line_source.SetPoint1(*[float(v) for v in flange])
        line_source.SetPoint2(*[float(v) for v in p])
        line_source.Update()
        marker.GetProperty().SetColor(*color)
        line_actor.GetProperty().SetColor(*color)
        label.GetTextProperty().SetColor(*color)
        marker.SetVisibility(True)
        line_actor.SetVisibility(True)
        label.SetVisibility(True)
