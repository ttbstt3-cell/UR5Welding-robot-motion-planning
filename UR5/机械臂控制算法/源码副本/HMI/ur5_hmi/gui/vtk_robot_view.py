from __future__ import annotations

from pathlib import Path
import time

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget

from ros.state_monitor import RobotState
from gui.robot_view import URDF_PATH, UrdfKinematicModel
from gui.scene_geometry import SceneGeometry, base_link_to_base
from safety.joint_limits import JOINT_NAMES

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


class VtkRobotView(QWidget):
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
        self._wire_tcp_transform = None
        self._wire_nominal_tcp_transform = None
        self._flange_tcp_nominal_transform = None
        self._wire_tip_tcp = None
        self._wire_length_m = None
        self._wire_display_status = "unavailable"
        # Diagnostic-only overlay.  The yellow marker is the physical wire
        # endpoint reconstructed from the same flange chain as the gun actor;
        # the magenta line connects it to the red controller LIVE TCP.  These
        # actors never feed FK, collision, planning, or ROS2 commands.
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
        }

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.09, 0.11, 0.15)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self._default_view_angle = float(self.renderer.GetActiveCamera().GetViewAngle())
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.vtk_widget)
        self._add_axes()
        self._load_meshes()
        self._load_scene_geometry()
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
        if getattr(self, "_wire_actor", None) is not None:
            self.renderer.RemoveActor(self._wire_actor)
            self._wire_actor = None
            self._wire_tcp_transform = None
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
        sphere.SetRadius(0.025)
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
        # Wire-endpoint diagnostics are deliberately separate from the red
        # LIVE TCP marker.  A fresh controller TCP is required before the
        # connecting error line is shown; the wire tip itself can still be
        # inspected when ROS feedback is unavailable.
        wire_tip = vtk.vtkSphereSource()
        wire_tip.SetRadius(0.010)
        wire_tip.SetThetaResolution(18)
        wire_tip.SetPhiResolution(10)
        wire_tip_mapper = vtk.vtkPolyDataMapper()
        wire_tip_mapper.SetInputConnection(wire_tip.GetOutputPort())
        self._wire_tip_actor = vtk.vtkActor()
        self._wire_tip_actor.SetMapper(wire_tip_mapper)
        self._wire_tip_actor.GetProperty().SetColor(1.0, 0.82, 0.05)
        self.renderer.AddActor(self._wire_tip_actor)
        self._wire_tip_label = vtk.vtkBillboardTextActor3D()
        self._wire_tip_label.SetInput("WIRE TIP")
        wire_text = self._wire_tip_label.GetTextProperty()
        wire_text.SetFontSize(13)
        wire_text.SetBold(True)
        wire_text.SetColor(1.0, 0.86, 0.10)
        wire_text.SetBackgroundColor(0.05, 0.05, 0.05)
        wire_text.SetBackgroundOpacity(0.65)
        self.renderer.AddActor(self._wire_tip_label)
        error_line = vtk.vtkLineSource()
        error_line.SetPoint1(0.0, 0.0, 0.0)
        error_line.SetPoint2(0.0, 0.0, 0.0)
        error_line_mapper = vtk.vtkPolyDataMapper()
        error_line_mapper.SetInputConnection(error_line.GetOutputPort())
        self._wire_tcp_error_line_actor = vtk.vtkActor()
        self._wire_tcp_error_line_actor.SetMapper(error_line_mapper)
        self._wire_tcp_error_line_actor.GetProperty().SetColor(1.0, 0.10, 0.85)
        self._wire_tcp_error_line_actor.GetProperty().SetLineWidth(3.0)
        self.renderer.AddActor(self._wire_tcp_error_line_actor)
        self._wire_tcp_error_line_source = error_line
        self._wire_tcp_error_label = vtk.vtkBillboardTextActor3D()
        self._wire_tcp_error_label.SetInput("ΔTCP —")
        error_text = self._wire_tcp_error_label.GetTextProperty()
        error_text.SetFontSize(13)
        error_text.SetBold(True)
        error_text.SetColor(1.0, 0.25, 0.90)
        error_text.SetBackgroundColor(0.05, 0.05, 0.05)
        error_text.SetBackgroundOpacity(0.70)
        self.renderer.AddActor(self._wire_tcp_error_label)
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

    def _create_gun_collision_actor(self) -> None:
        """Display the exported GunCollisionHull with the live flange pose.

        MATLAB uses the same exported hull as the gun collision geometry.  The
        HMI only visualizes it; it is not added to ROS2 or to any collision
        checker.  The flange->mount transform comes from the frozen
        ``ur5_tool_calibration.mat`` and is read-only.
        """
        self._gun_display_status = "unavailable"
        self._gun_actor = None
        self._gun_actor_mount_transform = None
        self._gun_nominal_mount_transform = None
        self._wire_nominal_tcp_transform = None
        self._flange_tcp_nominal_transform = None
        self._wire_tip_tcp = None
        self._wire_length_m = None
        try:
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
                self._tcp_rotation_flange = self._flange_tcp_nominal_transform[:3, :3]
            else:
                T_mount_tcp = None
            self._gun_actor = actor
            self._gun_display_status = (
                "GunCollisionHull（HMI flange局部调整；MATLAB场景未改写）"
            )

            # The audit MAT is the recorded source for the validated wire
            # length/diameter.  Build the same TCP-relative cylinder transform
            # as sync_ur5_joint_states.m; if the audit is absent, leave the
            # wire layer unavailable instead of inventing dimensions.
            self._wire_display_status = "unavailable"
            audit_path = Path(__file__).resolve().parents[2] / "matlab仿真" / "current_weld_geometry_audit.mat"
            if T_mount_tcp is not None and audit_path.is_file():
                audit = loadmat(str(audit_path), squeeze_me=True, struct_as_record=False).get("audit")
                wire_length = float(getattr(audit, "wireLengthM", np.nan))
                wire_diameter = float(getattr(audit, "wireDiameterM", np.nan))
                if np.isfinite(wire_length) and wire_length > 0 and np.isfinite(wire_diameter) and wire_diameter > 0:
                    hull_mount = (T_mount_hull @ np.column_stack((vertices, np.ones(len(vertices)))).T).T[:, :3]
                    p_tcp_mount = T_mount_tcp[:3, 3]
                    p_head_mount = hull_mount[np.argmin(np.linalg.norm(hull_mount - p_tcp_mount, axis=1))]
                    p_head_tcp = (np.linalg.inv(T_mount_tcp) @ np.r_[p_head_mount, 1.0])[:3]
                    z_wire = -p_head_tcp / max(float(np.linalg.norm(p_head_tcp)), 1e-12)
                    x_ref = np.array([1.0, 0.0, 0.0])
                    if abs(float(np.dot(x_ref, z_wire))) > 0.95:
                        x_ref = np.array([0.0, 1.0, 0.0])
                    x_wire = np.cross(x_ref, z_wire)
                    x_wire /= max(float(np.linalg.norm(x_wire)), 1e-12)
                    y_wire = np.cross(z_wire, x_wire)
                    y_wire /= max(float(np.linalg.norm(y_wire)), 1e-12)
                    T_tcp_wire = np.eye(4, dtype=float)
                    T_tcp_wire[:3, :3] = np.column_stack((x_wire, y_wire, z_wire))
                    T_tcp_wire[:3, 3] = p_head_tcp + wire_length / 2.0 * z_wire
                    cylinder = vtk.vtkCylinderSource()
                    cylinder.SetRadius(wire_diameter / 2.0)
                    cylinder.SetHeight(wire_length)
                    cylinder.SetResolution(16)
                    cylinder.SetCenter(0.0, 0.0, 0.0)
                    cylinder.Update()
                    wire_mapper = vtk.vtkPolyDataMapper()
                    wire_mapper.SetInputConnection(cylinder.GetOutputPort())
                    wire_actor = vtk.vtkActor()
                    wire_actor.SetMapper(wire_mapper)
                    wire_actor.GetProperty().SetColor(0.95, 0.75, 0.12)
                    wire_actor.GetProperty().SetOpacity(0.95)
                    self.renderer.AddActor(wire_actor)
                    # vtkCylinderSource's native cylinder axis is +Y.  Rotate
                    # that axis into the TCP-wire +Z axis before applying the
                    # dynamic base->flange->tool0->mount->TCP chain.
                    R_y_to_z = np.eye(4, dtype=float)
                    angle = np.pi / 2.0
                    R_y_to_z[:3, :3] = np.array([
                        [1.0, 0.0, 0.0],
                        [0.0, np.cos(angle), -np.sin(angle)],
                        [0.0, np.sin(angle), np.cos(angle)],
                    ])
                    self._wire_actor = wire_actor
                    self._wire_nominal_tcp_transform = (
                        T_flange_tool0
                        @ T_tool0_mount
                        @ T_mount_tcp
                        @ T_tcp_wire
                        @ R_y_to_z
                    )
                    # The endpoint furthest from the gun head is kept in the
                    # TCP frame for an independent LIVE-TCP mismatch check.
                    self._wire_tip_tcp = p_head_tcp + wire_length * z_wire
                    self._wire_length_m = wire_length
                    self._wire_display_status = (
                        f"wireCollision（{wire_length * 1000:.1f}×"
                        f"{wire_diameter * 1000:.2f} mm，HMI flange局部调整）"
                    )
            if self._wire_actor is None:
                self._wire_display_status = "wireCollision 显示不可用（缺少已审计尺寸）"
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

    def gun_mount_adjustment(self) -> dict:
        axis = None if self._gun_mount_roll_axis_flange is None else np.asarray(self._gun_mount_roll_axis_flange, dtype=float)
        return {
            "roll_deg": float(self._gun_mount_roll_deg),
            "translation_mm": (np.asarray(self._gun_mount_translation_m, dtype=float) * 1000.0).tolist(),
            "rotation_axis_flange": None if axis is None else axis.tolist(),
            "coordinate_frame": "URDF flange",
            "status": self._gun_mount_adjustment_status,
        }

    def set_gun_mount_calibration_status(self, status: str) -> None:
        self._gun_mount_adjustment_status = str(status).upper()

    def gun_mount_wire_diagnostics(self) -> dict:
        """Return the physical wire tip vs the fresh controller LIVE TCP."""
        if self._wire_tip_tcp is None or self._wire_nominal_tcp_transform is None:
            return {"available": False, "reason": "wireCollision geometry unavailable"}
        if self._live_q is None or self._live_tcp_fk is None:
            return {"available": False, "reason": "fresh LIVE TCP unavailable"}
        if not str(self._live_tcp_source).startswith("controller "):
            return {"available": False, "reason": "controller LIVE TCP stale/unavailable"}
        try:
            transforms = self.model.link_transforms_in_base(self._live_q)
            T_flange = transforms["flange"]
            axis = self._gun_mount_roll_axis_flange
            T_adjust = _axis_angle_rotation(axis, np.deg2rad(float(self._gun_mount_roll_deg)))
            T_adjust[:3, 3] = np.asarray(self._gun_mount_translation_m, dtype=float)
            # The cylinder actor has an additional visual Y→Z orientation,
            # but the physical endpoint is stored in TCP coordinates.  Use
            # the chain only up to TCP so the endpoint is not transformed
            # twice.
            if self._flange_tcp_nominal_transform is None:
                return {"available": False, "reason": "flange→TCP transform unavailable"}
            p_tip = (T_flange @ T_adjust @ self._flange_tcp_nominal_transform @ np.r_[self._wire_tip_tcp, 1.0])[:3]
            live = np.asarray(self._live_tcp_fk, dtype=float).reshape(3)
            delta = (p_tip - live) * 1000.0
            return {
                "available": True,
                "wire_tip_base_m": p_tip.tolist(),
                "live_tcp_base_m": live.tolist(),
                "delta_xyz_mm": delta.tolist(),
                "distance_mm": float(np.linalg.norm(delta)),
                "live_tcp_source": self._live_tcp_source,
                "tcp_live_policy": "fresh controller ROS/RTDE TCP; HMI FK fallback excluded",
                "wire_tip_source": "current_weld_geometry_audit.mat + nearest GunCollisionHull vertex estimate",
                "wire_length_m": float(self._wire_length_m) if self._wire_length_m is not None else None,
                "mount_transform_frame": "URDF flange",
                "mount_roll_deg": float(self._gun_mount_roll_deg),
                "mount_translation_mm": (np.asarray(self._gun_mount_translation_m) * 1000.0).tolist(),
            }
        except Exception as exc:
            return {"available": False, "reason": str(exc)}

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
        for actor in getattr(self, "_workpiece_actors", []):
            self.renderer.RemoveActor(actor)
        for actor in getattr(self, "_workpiece_outline_actors", []):
            self.renderer.RemoveActor(actor)
        for actor in getattr(self, "_seam_actors", {}).values():
            self.renderer.RemoveActor(actor)
        if getattr(self, "_planned_tcp_actor", None) is not None:
            self.renderer.RemoveActor(self._planned_tcp_actor)
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

    def set_planned_tcp_path(self, points, frame: str = "base_link") -> None:
        """Show a planned TCP path.

        ``frame='base_link'`` applies the verified Rz(180 deg) conversion from
        the MATLAB planner output; ``frame='base'`` uses the points unchanged.
        """
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
        self._apply_actor_workpiece_transform(actor)
        self._apply_visibility()

    def clear_planned_tcp_path(self) -> None:
        self.set_planned_tcp_path(None)

    def scene_status(self) -> str:
        return f"{self._scene_status} | {self._gun_display_status} | 工件位姿: {self._workpiece_pose_source}"

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
        if self._planned_tcp_actor is not None:
            actors.append(self._planned_tcp_actor)
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
        if self._gun_actor is not None:
            self._gun_actor.SetVisibility(bool(self._visibility.get("gun", True)))
        if self._wire_actor is not None:
            self._wire_actor.SetVisibility(bool(self._visibility.get("wire", True)))
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
        self.vtk_widget.GetRenderWindow().Render()

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
        self._select_live_tcp()

    set_display_tcp_offset = set_calibrated_tcp_offset

    def clear_calibrated_tcp_offset(self) -> None:
        self._calibrated_tcp_offset = None
        self._tcp_offset_flange = None
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
            self._preview_tcp_fk = self._fk_tcp_from_flange(self._preview_q)[1]
            self._preview_tcp_index = tcp_index
            if tcp_index is not None and self._preview_tcp_fk is not None:
                point = self._planned_tcp_raw_point_base(tcp_index)
                if point is not None:
                    self._preview_tcp_path_errors_mm.append(
                        float(np.linalg.norm(self._preview_tcp_fk - point) * 1000.0)
                    )
            self._check_preview_relative_transform()
            self.q = list(self._preview_q)

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
        if self._preview_q is None or self._tcp_offset_flange is None:
            return
        try:
            transforms = self.model.link_transforms_in_base(self._preview_q)
            flange = transforms["flange"]
            T_expected = np.eye(4, dtype=float)
            T_expected[:3, :3] = self._tcp_rotation_flange
            T_expected[:3, 3] = self._tcp_offset_flange
            T_tcp = np.eye(4, dtype=float)
            T_tcp[:3, :3] = flange[:3, :3] @ self._tcp_rotation_flange
            T_tcp[:3, 3] = self._preview_tcp_fk
            T_actual = np.linalg.inv(flange) @ np.block([
                [T_tcp[:3, :3], T_tcp[:3, 3].reshape(3, 1)],
                [np.zeros((1, 3)), np.ones((1, 1))],
            ])
            self._preview_relative_errors_mm.append(
                float(np.linalg.norm(T_actual[:3, 3] - T_expected[:3, 3]) * 1000.0)
            )
            R_error = T_actual[:3, :3] @ self._tcp_rotation_flange.T
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
        camera.SetPosition(*(center + direction * max(radius / 0.45, 0.25)))
        camera.SetViewUp(*view_up)
        camera.SetViewAngle(self._default_view_angle)
        camera.Zoom(1.6)
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

    def _refresh(self) -> None:
        display_q = self._preview_q if self.preview_active else self._live_q
        if display_q is None or len(display_q) != 6:
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
        if self._wire_actor is not None and self._wire_tcp_transform is not None and "flange" in transforms:
            self._wire_actor.SetUserMatrix(_vtk_matrix(transforms["flange"] @ self._wire_tcp_transform))
            self._wire_actor.SetVisibility(bool(self._visibility.get("wire", True)))
        self._update_wire_tcp_overlay(
            transforms,
            bool(self._visibility.get("wire", True)),
            bool(self._visibility.get("current_tcp", True)),
        )
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

    def _update_wire_tcp_overlay(self, transforms, wire_visible: bool, tcp_visible: bool) -> None:
        """Update wire-tip/error overlays from one fixed flange transform chain.

        The gun, wire actor and tip marker all use ``T_base_flange @ T_adjust``.
        The red point is always the independently selected LIVE TCP.  In
        particular, preview q never overwrites ``_live_tcp_fk``.
        """
        actors = (
            self._wire_tip_actor,
            self._wire_tip_label,
            self._wire_tcp_error_line_actor,
            self._wire_tcp_error_label,
        )
        if any(actor is None for actor in actors):
            return
        hide_error = lambda: (
            self._wire_tcp_error_line_actor.SetVisibility(False),
            self._wire_tcp_error_label.SetVisibility(False),
        )
        if (transforms is None or "flange" not in transforms
                or self._wire_tip_tcp is None or not wire_visible):
            self._wire_tip_actor.SetVisibility(False)
            self._wire_tip_label.SetVisibility(False)
            hide_error()
            return
        try:
            axis = self._gun_mount_roll_axis_flange
            if axis is None or self._flange_tcp_nominal_transform is None:
                raise RuntimeError("flange gun/TCP transform unavailable")
            T_adjust = _axis_angle_rotation(axis, np.deg2rad(float(self._gun_mount_roll_deg)))
            T_adjust[:3, 3] = np.asarray(self._gun_mount_translation_m, dtype=float)
            T_flange = np.asarray(transforms["flange"], dtype=float)
            p_tip = (T_flange @ T_adjust @ self._flange_tcp_nominal_transform
                     @ np.r_[self._wire_tip_tcp, 1.0])[:3]
            self._wire_tip_actor.SetPosition(*[float(value) for value in p_tip])
            self._wire_tip_label.SetPosition(*[float(value) for value in (p_tip + np.array([0.018, 0.0, 0.018]))])
            self._wire_tip_label.SetInput("WIRE TIP")
            self._wire_tip_actor.SetVisibility(True)
            self._wire_tip_label.SetVisibility(True)

            # Only a fresh controller topic is a real-time comparison.  A
            # local FK fallback is intentionally not presented as a measured
            # TCP-vs-wire error.
            live = self._live_tcp_fk
            controller_live = (
                tcp_visible and live is not None
                and str(self._live_tcp_source).startswith("controller ")
            )
            if not controller_live:
                hide_error()
                return
            p_live = np.asarray(live, dtype=float).reshape(3)
            delta_mm = (p_tip - p_live) * 1000.0
            distance_mm = float(np.linalg.norm(delta_mm))
            self._wire_tcp_error_line_source.SetPoint1(*[float(value) for value in p_tip])
            self._wire_tcp_error_line_source.SetPoint2(*[float(value) for value in p_live])
            self._wire_tcp_error_line_source.Update()
            midpoint = 0.5 * (p_tip + p_live)
            self._wire_tcp_error_label.SetPosition(*[float(value) for value in (midpoint + np.array([0.018, 0.0, 0.018]))])
            self._wire_tcp_error_label.SetInput(
                f"ΔTCP {delta_mm[0]:+.1f},{delta_mm[1]:+.1f},{delta_mm[2]:+.1f} mm\n"
                f"|Δ| {distance_mm:.1f} mm"
            )
            self._wire_tcp_error_line_actor.SetVisibility(True)
            self._wire_tcp_error_label.SetVisibility(True)
        except Exception:
            self._wire_tip_actor.SetVisibility(False)
            self._wire_tip_label.SetVisibility(False)
            hide_error()

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
