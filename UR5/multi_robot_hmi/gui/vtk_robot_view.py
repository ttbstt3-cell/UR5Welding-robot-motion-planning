from __future__ import annotations

from pathlib import Path

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QVBoxLayout, QWidget

from ros.state_monitor import RobotState
from gui.robot_view import URDF_PATH, UrdfKinematicModel
from gui.scene_geometry import DEFAULT_SCENE_JSON, SceneGeometry, base_link_to_base

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
    direct = Path(filename.removeprefix("file://")).expanduser()
    if direct.is_file():
        return direct.resolve()
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
        self.q = [0.0] * 6
        self.live_feedback = False
        self.tcp = None
        self.tcp_frame_id = ""
        self._actors = {}
        self._joint_labels = {}
        self._tcp_label = None
        self._camera_initialized = False
        # Read-only scene layers (workpiece / seam / planned TCP path).
        self._workpiece_actors = []
        self._workpiece_outline_actors = []
        self._seam_actors = {}
        self._planned_tcp_actor = None
        self._planned_tcp_points = None
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
        self._actors = {}
        self._joint_labels = {}
        self._tcp_label = None
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
            mesh_transform = np.eye(4, dtype=float)
            mesh_transform[:3, :3] = np.diag(info.get("scale", (1.0, 1.0, 1.0)))
            self._actors[link_name] = (actor, info["origin"] @ mesh_transform)

        # Put a Chinese J1--J6 label at each corresponding URDF joint frame.
        # These are read-only annotations and follow the live robot pose.
        for index, joint_name in enumerate(self.model.revolute_names, start=1):
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
        self._tcp_label.SetInput("TCP")
        tcp_text = self._tcp_label.GetTextProperty()
        tcp_text.SetFontSize(18)
        tcp_text.SetBold(True)
        tcp_text.SetColor(1.0, 1.0, 1.0)
        tcp_text.SetBackgroundColor(0.05, 0.05, 0.05)
        tcp_text.SetBackgroundOpacity(0.65)
        self.renderer.AddActor(self._tcp_label)
        # Camera is initialized after the first real joint transform is
        # applied.  Resetting it here would frame all links at their zero-pose
        # origins and lose the robot once the live transforms arrive.

    # ------------------------------------------------------------------
    # Read-only scene geometry: workpiece boxes, seam lines, planned TCP path
    # ------------------------------------------------------------------
    def _load_scene_geometry(self, path: Path | str | None = DEFAULT_SCENE_JSON) -> None:
        """Build the workpiece/seam actors from scene_geometry.json.

        Nothing here creates geometry of its own: box sizes and poses are the
        values exported by the CoppeliaSim geometry exporter, converted into the
        URDF ``base`` frame with the verified transform.
        """
        self._clear_scene_actors()
        if path is None:
            self._scene = None
            self._scene_status = "not configured for this robot"
            return
        try:
            scene = SceneGeometry(path)
        except Exception as exc:  # missing file / malformed json
            self._scene = None
            self._scene_status = f"unavailable: {exc}"
            return
        self._scene = scene

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
        return f"{self._scene_status} | 工件位姿: {self._workpiece_pose_source}"

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

    def set_state(self, q, tcp_position=None, tcp_frame_id="") -> None:
        if q is not None and len(q) == 6:
            self.q = list(q)
            self.live_feedback = True
        elif self.q is None:
            self.q = [0.0] * 6
            self.live_feedback = False
        else:
            # Freeze the last known pose for display only.  The main window
            # still marks the feedback stale and disables motion controls.
            self.live_feedback = False
        self.tcp = None if tcp_position is None else list(tcp_position)
        self.tcp_frame_id = str(tcp_frame_id or "")

    def set_preview_state(self, q, tcp_index: int | None = None) -> None:
        """Display a local preview pose without presenting it as live TCP data.

        When ``tcp_index`` is given, the planned TCP sample at that index is
        shown with the same red marker used for the live TCP, so the slider
        position and the TCP position always correspond.  The marker is drawn
        in the URDF base frame exactly like the live one.
        """
        if q is not None and len(q) == 6:
            self.q = list(q)
        self.live_feedback = False
        if tcp_index is None:
            self.tcp = None
            self.tcp_frame_id = ""
        else:
            point = self.preview_point_base(tcp_index)
            self.tcp = None if point is None else list(point)
            # The planned path was converted into the base frame already, so it
            # is displayed with the same rule as the live /tcp_pose topic.
            self.tcp_frame_id = "base" if point is not None else ""

    def set_urdf_text(self, urdf_text: str) -> None:
        try:
            self.model = UrdfKinematicModel(urdf_text=urdf_text)
            self._load_meshes()
            self.renderer.ResetCamera()
        except Exception:
            # Preserve the already loaded calibrated model on malformed input.
            pass

    def set_robot_profile(self, robot_description: Path, joint_names, scene_geometry_path=None) -> None:
        self.model = UrdfKinematicModel(Path(robot_description), joint_names=joint_names)
        self.q = [0.0] * len(joint_names)
        self.live_feedback = False
        self.tcp = None
        self.tcp_frame_id = ""
        self._planned_tcp_points = None
        self._workpiece_calibration_delta = np.eye(4, dtype=float)
        self._workpiece_pose_source = "SIMULATION"
        self._load_meshes()
        self._load_scene_geometry(scene_geometry_path)
        self._refresh()

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
        if self.q is None or len(self.q) != 6:
            return
        try:
            transforms = self.model.link_transforms_in_base(self.q)
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
        tcp_is_base_frame = self.tcp_frame_id in ("base", "base_link")
        if self.tcp is not None and len(self.tcp) == 3 and tcp_is_base_frame:
            self._tcp_actor.SetPosition(*[float(v) for v in self.tcp])
            tcp_label_position = np.asarray(self.tcp, dtype=float) + np.array([0.03, 0.0, 0.03])
            self._tcp_label.SetPosition(*[float(v) for v in tcp_label_position])
            if "tool0" in transforms:
                self._tcp_line_source.SetPoint1(*[float(v) for v in transforms["tool0"][:3, 3]])
                self._tcp_line_source.SetPoint2(*[float(v) for v in self.tcp])
                self._tcp_line_source.Update()
            self._tcp_line_actor.SetVisibility(True)
            self._tcp_label.SetVisibility(True)
            self._tcp_actor.SetVisibility(True)
        else:
            self._tcp_line_actor.SetVisibility(False)
            self._tcp_actor.SetVisibility(False)
            self._tcp_label.SetVisibility(False)
        self.vtk_widget.GetRenderWindow().Render()
