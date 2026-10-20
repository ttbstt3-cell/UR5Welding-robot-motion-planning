from __future__ import annotations

import math
import os
import csv
import hashlib
import json
import shutil
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from PyQt5.QtCore import QObject, QProcess, QThread, QTimer, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QButtonGroup,
    QComboBox,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSlider,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from gui.ui_metrics import UiMetrics
from gui.widgets import JogButtons, StatusRow, set_status_label
try:
    # QVTK requires a real X11 window; use the existing read-only skeleton for
    # CI/offscreen layout checks only.  Normal HMI sessions still use VTK.
    if os.environ.get("QT_QPA_PLATFORM", "").lower() == "offscreen":
        raise ImportError("offscreen layout smoke test")
    from gui.vtk_robot_view import VtkRobotView as Robot3DView
except ImportError:
    # Matplotlib skeleton remains available if VTK Qt integration is absent.
    from gui.robot_view import Robot3DView
from ros.ros_node import JOINT_NAMES, UR5RosNode
from ros.state_monitor import RobotState
from ros.trajectory_client import MotionState
from safety.safety_checks import check_motion_ready
from trajectory.trajectory_loader import load_trajectory
from trajectory.trajectory_utils import (
    TrajectoryData,
    duration_for_jog,
    make_two_point_trajectory,
    trajectory_to_ros_message,
)
from trajectory.trajectory_validator import ValidationReport, validate_trajectory
from calibration.tcp_pivot import (
    PivotCalibrationError,
    fit_pivot_calibration,
    verify_pivot_sample,
)
from gui.robot_view import UrdfKinematicModel
from gui.tcp_robot_panel import TcpRobotPanel


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "hmi_config.yaml"

JOINT_CN = [
    "肩部旋转",
    "肩部俯仰",
    "肘部",
    "腕部1",
    "腕部2",
    "腕部3",
]

# Set to True temporarily when diagnosing Qt geometry; production remains off.
UI_DEBUG_LAYOUT = False


class RosWorker(QObject):
    state = pyqtSignal(object)
    log = pyqtSignal(str)
    goal_active = pyqtSignal(bool)
    goal_done = pyqtSignal(bool, str)
    # (motion_state, busy, goal_label) - mirrors TrajectoryClient's lifecycle.
    motion = pyqtSignal(str, bool, str)
    # Hands the TrajectoryClient object to the GUI so it can read `.busy`.
    client_ready = pyqtSignal(object)
    request_jog = pyqtSignal(int, float, float)
    request_stop = pyqtSignal()
    request_trajectory = pyqtSignal(object)
    request_refresh = pyqtSignal()
    quit_requested = pyqtSignal()
    finished = pyqtSignal()

    def __init__(self, action_name: str, robot_ip: str = "192.168.0.10"):
        super().__init__()
        self.action_name = action_name
        self.robot_ip = robot_ip
        self.node = None
        self._stop = False
        self.request_jog.connect(self._jog)
        self.request_stop.connect(self._stop_goal)
        self.request_trajectory.connect(self._send_trajectory)
        self.request_refresh.connect(self._refresh)
        self.quit_requested.connect(self._quit)

    @pyqtSlot()
    def run(self):
        import rclpy
        rclpy.init()
        self.node = UR5RosNode(
            self.state.emit,
            self.log.emit,
            self.goal_active.emit,
            self.goal_done.emit,
            self.action_name,
            on_motion=self.motion.emit,
            robot_ip=self.robot_ip,
        )
        self.client_ready.emit(self.node.trajectory)
        self.spin_timer = QTimer(self)
        self.spin_timer.setInterval(20)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start()

    @pyqtSlot()
    def _spin_once(self):
        import rclpy
        if self.node is not None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    @pyqtSlot()
    def _quit(self):
        self._stop = True
        import rclpy
        if hasattr(self, "spin_timer"):
            self.spin_timer.stop()
        if self.node is not None:
            self.node.shutdown()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()
        self.finished.emit()

    @pyqtSlot()
    def _refresh(self):
        if self.node is not None:
            self.node._poll_status()

    @pyqtSlot(int, float, float)
    def _jog(self, index: int, direction: float, speed_percent: float):
        if self.node is None:
            return
        state = self.node.state.copy()
        if len(state.positions) != 6:
            self.log.emit("Jog rejected: no fresh joint state")
            return
        q0 = list(state.positions)
        q1 = list(q0)
        delta = math.radians(0.5) * float(direction)
        q1[index] += delta
        result = check_motion_ready(state, q1)
        if not result.ok:
            self.log.emit("Jog rejected: " + "; ".join(result.reasons))
            return
        duration = duration_for_jog(delta, speed_percent)
        detail = f"J{index + 1} {math.degrees(delta):+.3f} deg ({duration:.1f}s)"
        if not self.node.send(make_two_point_trajectory(q0, q1, duration), kind="JOG", detail=detail):
            self.log.emit("Jog blocked: motion busy (see Motion State / Current Goal)")
            # Release the GUI debounce lock when the worker rejected the
            # request before creating a goal.  If another goal is still busy,
            # the subsequent terminal callback will release it safely.
            self.motion.emit(
                MotionState.IDLE.value,
                bool(self.node.trajectory.busy),
                self.node.trajectory.current_goal_label,
            )

    @pyqtSlot()
    def _stop_goal(self):
        if self.node is not None and not self.node.cancel():
            self.log.emit("No active trajectory goal")

    @pyqtSlot(object)
    def _send_trajectory(self, trajectory):
        if self.node is None:
            return
        state = self.node.state.copy()
        target = list(trajectory.points[0].positions) if trajectory.points else None
        result = check_motion_ready(state, target)
        if not result.ok:
            self.log.emit("Trajectory rejected: " + "; ".join(result.reasons))
            return
        if not self.node.send(trajectory, kind="TRAJ", detail=f"{len(trajectory.points)} points"):
            self.log.emit("Trajectory blocked: motion busy (see Motion State / Current Goal)")


class MainWindow(QMainWindow, TcpRobotPanel):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5 Safe HMI — monitor first")
        self.setMinimumSize(1440, 850)
        self.resize(1600, 1000)
        self.state = RobotState()
        self.goal_active = False
        # GUI-side debounce for jog clicks.  The ROS worker has its own busy
        # guard, but this lock closes the short queued-signal window where a
        # rapid/held click could enqueue several one-step requests before the
        # worker reports SENDING.
        self._jog_pending = False
        # Direct reference to the ROS-side TrajectoryClient.  The motion gate
        # reads `trajectory_client.busy` directly instead of relying only on the
        # GUI-side `goal_active` flag, which flips too early (it cannot see the
        # window between send_goal_async() and goal acceptance, nor a pending
        # cancel).
        self.trajectory_client = None
        self.motion_state_text = MotionState.IDLE.value
        self.motion_goal_label = ""
        self.loaded_path = None  # legacy numeric accessor retained for UI/tests
        self.loaded_trajectory: TrajectoryData | None = None
        self.trajectory_validation: ValidationReport | None = None
        # A trajectory is executable only against the TCP/workpiece/URDF
        # calibration versions it was planned and collision-checked with.
        # Missing binding metadata is deliberately treated as legacy/STALE;
        # it remains previewable but can never pass the final execution gate.
        self.trajectory_stale_reason: str | None = None
        self.loaded_path_name = "未加载"
        self.preview_active = False
        self.preview_start_monotonic = None
        self.preview_index = 0
        self.execution_started_monotonic = None
        self.execution_trajectory: TrajectoryData | None = None
        self.execution_log: list[dict] = []
        # Offline MATLAB planner state is deliberately separate from the ROS2
        # motion/action state.  A planning process never sends a ROS command.
        self.planning_busy = False
        self.planning_process: QProcess | None = None
        self.planning_request_path: Path | None = None
        self.planning_output_path: Path | None = None
        self.planning_report_path: Path | None = None
        self.preview_timer = QTimer(self)
        self.preview_timer.setInterval(30)
        self.preview_timer.timeout.connect(self._preview_tick)
        self._description_loaded = False
        # Workpiece calibration is a read-only TCP teaching workflow.  The
        # points are stored in the live URDF ``base`` frame and are never used
        # to command the robot.
        self.calibration_points: dict[str, np.ndarray | None] = {
            "P0": None,
            "PX": None,
            "PY": None,
            "P_CHECK": None,
        }
        self.calibration_result: dict | None = None
        self.calibration_file = ROOT.parent / "workpiece_pose_real.json"
        # Precision required before applying a measured workpiece frame to
        # the planning scene.  This is deliberately separate from the
        # green/amber/red quality bands (which remain 2/5 mm).
        self.calibration_residual_limit_mm = 6.0
        # A looser, display-only ceiling.  It never replaces the stricter
        # real-planning application gate above.
        self.calibration_preview_limit_mm = 10.0
        self.calibration_point_timestamps: dict[str, str | None] = {
            "P0": None, "PX": None, "PY": None, "P_CHECK": None,
        }
        self.calibration_point_frames: dict[str, str | None] = {
            "P0": None, "PX": None, "PY": None, "P_CHECK": None,
        }
        # Fixed-point TCP calibration is kept independent from the frozen
        # CoppeliaSim/MATLAB simulation calibration.  These samples contain
        # flange FK poses computed from q_actual, never TCP-topic poses.
        self.tcp_calibration_file = ROOT.parent / "real_tcp_calibration.json"
        self.tcp_calibration_applied_file = ROOT.parent / "real_tcp_calibration_applied.json"
        # Explicitly adopted controller TCP versions are kept beside the
        # historical applied file.  The active pointer is written only after
        # two operator confirmations and never overwrites the old record.
        self.tcp_calibration_active_file = ROOT.parent / "real_tcp_calibration_active.json"
        self.tcp_calibration_poses: list[dict] = []
        self.tcp_verification_poses: list[dict] = []
        self.tcp_calibration_result = None
        self.tcp_calibration_applied = False
        # Offline physical gun-installation adjustment.  This transform is
        # deliberately independent from TCP calibration and MATLAB planning;
        # preview edits only the 3D actors until an operator saves/ confirms a
        # separate record.
        self.gun_mount_calibration_file = ROOT.parent / "real_gun_mount_calibration.json"
        self.gun_mount_adjustment = {
            "roll_deg": 90.0,
            "translation_mm": [0.0, 0.0, 0.0],
            "status": "DEFAULT",
        }
        self.config = self._load_config()
        self.ui_scale = float(self.config.get("ui", {}).get("scale", 1.0))
        self.metrics = UiMetrics(self.ui_scale)
        jog_config = self.config.get("jog", {})
        self.max_hmi_speed_percent = min(20, max(1, int(jog_config.get("max_hmi_speed_percent", 20))))
        trajectory_config = self.config.get("trajectory", {})
        self.start_tolerance_rad = math.radians(float(trajectory_config.get("start_tolerance_deg", 1.0)))
        self._build_ui()
        self._configure_display_tcp_offset()
        action_name = self.config.get("ros", {}).get(
            "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )
        robot_ip = self.config.get("ros", {}).get(
            "robot_ip", os.environ.get("UR5_ROBOT_IP", "192.168.0.10")
        )
        self.thread = QThread(self)
        self.worker = RosWorker(action_name, robot_ip)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.state.connect(self._on_state)
        self.worker.log.connect(self._log)
        self.worker.goal_active.connect(self._on_goal_active)
        self.worker.goal_done.connect(self._on_goal_done)
        self.worker.motion.connect(self._on_motion)
        self.worker.client_ready.connect(self._on_client_ready)
        self.worker.finished.connect(self.thread.quit)
        self.thread.start()
        self._log("界面启动：不会自动发送运动")
        self.showMaximized()

    def _load_config(self):
        try:
            return yaml.safe_load(CONFIG.read_text()) or {}
        except Exception:
            return {}

    def _u(self, value: float) -> int:
        """Scale a base UI dimension without touching robot behavior."""
        return self.metrics.px(value)

    def apply_ui_scale(self, scale: float) -> None:
        """Apply one consistent scale to fonts, rows, buttons and spacing."""
        self.ui_scale = min(1.5, max(1.0, float(scale)))
        self.metrics = UiMetrics(self.ui_scale)
        font = QFont("Noto Sans CJK SC")
        font.setPointSizeF(11.0 * self.ui_scale)
        self.setFont(font)
        self.setStyleSheet(
            f"QWidget {{ font-size:{self._u(14)}px; color:#222831; }}"
            f"QGroupBox {{ border:1px solid #d5d9de; border-radius:4px; margin-top:{self._u(10)}px; padding-top:{self._u(10)}px; }}"
            "QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 5px; font-weight:bold; }"
            f"QPushButton {{ min-height:{self._u(40)}px; }}"
        )
        if hasattr(self, "joint_table"):
            table_height = self._u(44) + self.metrics.joint_row * 6 + self._u(8)
            self.joint_table.setMinimumHeight(table_height)
            # Keep the table to exactly the six-row content height.  Let the
            # surrounding tab expand instead of adding blank table height that
            # hides the last row on a 1080p/125% display.
            self.joint_table.setMaximumHeight(table_height)
            self.joint_table.verticalHeader().setDefaultSectionSize(self.metrics.joint_row)
            self.joint_table.setStyleSheet(
                f"QTableWidget {{ font-size:{self._u(17)}px; }}"
                f"QTableWidget::item {{ padding:{self._u(8)}px; }}"
                f"QHeaderView::section {{ font-size:{self._u(17)}px; font-weight:bold; padding:{self._u(8)}px; }}"
            )
        if hasattr(self, "gun_roll_spin"):
            self.gun_roll_spin.setMinimumHeight(self._u(36))
            for spin in getattr(self, "gun_translation_spins", []):
                spin.setMinimumHeight(self._u(36))
            for button in getattr(self, "gun_roll_buttons", []):
                button.setMinimumSize(self._u(70), self._u(36))
            for button_name in ("gun_load_button", "gun_draft_button", "gun_confirm_button", "gun_reset_button"):
                button = getattr(self, button_name, None)
                if button is not None:
                    button.setMinimumHeight(self._u(42))
            self.joint_table.scrollToTop()
            self.joint_table.verticalScrollBar().setValue(0)
            QTimer.singleShot(0, self.joint_table.scrollToTop)
            QTimer.singleShot(0, lambda: self.joint_table.verticalScrollBar().setValue(0))
            for row in range(self.joint_table.rowCount()):
                for column, point_size in ((0, 16), (1, 18), (2, 14), (3, 16)):
                    item = self.joint_table.item(row, column)
                    if item is None:
                        continue
                    item_font = item.font()
                    item_font.setPointSizeF(point_size * self.ui_scale / 1.25)
                    item.setFont(item_font)
        if hasattr(self, "jog_widgets"):
            for jog in self.jog_widgets:
                jog.set_scale(self.ui_scale)
        if hasattr(self, "step_buttons"):
            for button in self.step_buttons:
                button.setMinimumSize(self._u(72), self.metrics.button_height)
                button.setStyleSheet(
                    f"QPushButton {{ font-size:{self._u(16)}px; font-weight:bold; }}"
                    "QPushButton:checked { background:#3578e5; color:white; }"
                    "QPushButton:hover { background:#dbeafe; }"
                )
        if hasattr(self, "speed_shortcuts"):
            for button in self.speed_shortcuts:
                button.setMinimumSize(self._u(60), self.metrics.button_height)
        if hasattr(self, "speed_slider"):
            self.speed_slider.setMinimumHeight(self._u(36))
            self.speed_slider.setMinimumWidth(self._u(320))
        if hasattr(self, "speed_value_label"):
            self.speed_value_label.setStyleSheet(
                f"font-size:{self._u(18)}px;font-weight:bold;"
            )
        if hasattr(self, "stop_button"):
            self.stop_button.setMinimumHeight(self._u(56))
            self.stop_button.setStyleSheet(
                f"background:#c62828;color:white;font-size:{self._u(18)}px;font-weight:bold;"
            )
        if hasattr(self, "online_setup_box"):
            self.online_setup_box.setMinimumHeight(self._u(210))
            self.online_start_button.setMinimumSize(self._u(150), self._u(44))
            self.online_cancel_button.setMinimumSize(self._u(150), self._u(44))
            self.online_stage_label.setMinimumWidth(self._u(230))
            self.online_start_snapshot_label.setMinimumHeight(self._u(42))
            self.online_output_view.setMinimumHeight(self._u(105))
            self.online_output_view.setMaximumHeight(self._u(145))
            for value in self.online_result_labels.values():
                value.setMinimumWidth(self._u(108))
        if hasattr(self, "calibration_source_combo"):
            self.calibration_source_combo.setMinimumHeight(self._u(34))
            for button in (
                self.calibration_compute_button,
                self.calibration_save_button,
                self.calibration_reset_button,
                self.calibration_restore_button,
            ):
                button.setMinimumHeight(self._u(40))
            for label in self.calibration_point_labels.values():
                label.setMinimumHeight(self._u(38))
        if hasattr(self, "tcp_capture_button"):
            for button in (
                self.tcp_capture_button,
                self.tcp_clear_button,
                self.tcp_compute_button,
                self.tcp_verify_capture_button,
                self.tcp_verify_button,
                self.tcp_save_button,
                self.tcp_apply_button,
                self.tcp_robot_read_button,
                self.tcp_saved_load_button,
                self.tcp_robot_write_button,
            ):
                button.setMinimumHeight(self._u(42))
            self.tcp_pose_table.setMinimumHeight(self._u(170))
            self.tcp_pose_table.setMaximumHeight(self._u(280))
        for name in ("refresh_button", "rviz_button"):
            if hasattr(self, name):
                getattr(self, name).setMinimumHeight(self.metrics.button_height)
        if hasattr(self, "ui_scale_combo"):
            self.ui_scale_combo.setMinimumHeight(self.metrics.button_height)
        if hasattr(self, "warning_label"):
            self.warning_label.setMinimumHeight(self._u(30))
            self.warning_label.setContentsMargins(0, self._u(8), 0, self._u(10))
            self.warning_label.setWordWrap(True)
            self.warning_label.setStyleSheet(
                f"color:#9b1c1c;font-size:{self._u(14)}px;font-weight:bold;"
            )
        if hasattr(self, "power_status_label"):
            self.power_status_label.setMinimumWidth(self._u(110))
            self.power_status_label.setMinimumHeight(self.metrics.status_row)
            self.power_status_label.setMaximumHeight(self.metrics.status_row)
            self._update_power_status()
        if hasattr(self, "safety_layout"):
            self.safety_layout.setSpacing(self._u(8))
        if hasattr(self, "tcp_labels"):
            for label in self.tcp_labels.values():
                label.setStyleSheet(
                    f"font-size:{self._u(16)}px;font-family:monospace;"
                )
        if hasattr(self, "ui_tabs"):
            self.ui_tabs.setStyleSheet(
                f"QTabBar::tab {{ min-width:{self._u(150)}px; min-height:{self._u(42)}px; "
                f"font-size:{self._u(15)}px; padding:{self._u(7)}px {self._u(18)}px; }}"
                "QTabWidget::pane { border:1px solid #d5d9de; }"
            )
        if hasattr(self, "view_buttons_widgets"):
            for button in self.view_buttons_widgets:
                button.setMinimumHeight(self._u(32))
        if hasattr(self, "top_splitter"):
            # The main splitter is horizontal: keep both panes usable without
            # forcing a fixed divider position after the operator drags it.
            self.top_splitter.setMinimumHeight(self._u(620))
            self.top_splitter.setMinimumWidth(self._u(1100))
        if hasattr(self, "robot_view"):
            self.robot_view.setMinimumSize(self._u(480), self._u(300))
        if hasattr(self, "ui_tabs"):
            self.ui_tabs.setMinimumHeight(self._u(240))
        if hasattr(self, "main_splitter"):
            self.main_splitter.setChildrenCollapsible(False)
        if hasattr(self, "status_labels"):
            if hasattr(self, "status_layout"):
                self.status_layout.setSpacing(self._u(2))
            for row in getattr(self, "status_rows", {}).values():
                row.set_scale(self.ui_scale)
            for label in getattr(self, "status_name_labels", []):
                label.setMinimumWidth(self._u(135))
            for row_label, value_label, unit_label in getattr(self, "tcp_row_labels", []):
                row_label.setMinimumWidth(self._u(45))
                row_label.setMinimumHeight(self.metrics.tcp_row)
                value_label.setMinimumWidth(self._u(130))
                value_label.setMinimumHeight(self.metrics.tcp_row)
                unit_label.setMinimumWidth(self._u(45))
                unit_label.setMinimumHeight(self.metrics.tcp_row)
            for card in getattr(self, "tcp_cards", []):
                card.setMinimumHeight(0)
            self._update_status_widgets()
        if hasattr(self, "log_status_box"):
            self.log_status_box.setMinimumHeight(self._u(230))
            self.log_status_box.setMaximumHeight(self._u(290))
        if hasattr(self, "log_detail_box"):
            self.log_detail_box.setMaximumHeight(self._u(330))
        if hasattr(self, "simple_state_labels"):
            for label in self.simple_state_labels.values():
                label.setMinimumHeight(self.metrics.status_row)
            self._update_status_widgets()
        if hasattr(self, "jog_status_labels"):
            for row in getattr(self, "jog_status_rows", {}).values():
                row.set_scale(self.ui_scale)
            for label in self.jog_status_labels.values():
                label.setMinimumHeight(self.metrics.status_row)
            self._update_jog_status()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        # Keep all inherited widgets readable on a high-resolution display.
        # Explicit safety/status fonts below remain intentionally larger.
        font = QFont("Noto Sans CJK SC")
        font.setPointSizeF(11.0 * self.ui_scale)
        self.setFont(font)
        self.setStyleSheet(
            f"QWidget {{ font-size:{round(14 * self.ui_scale)}px; color:#222831; }}"
            "QGroupBox { border:1px solid #d5d9de; border-radius:4px; margin-top:10px; padding-top:10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left:10px; padding:0 5px; font-weight:bold; }"
            "QPushButton { min-height:40px; }"
        )
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # Main layout: the complete left half is the 3D scene and the
        # complete right half is safety + functional tabs.  This splitter is
        # deliberately horizontal and user-adjustable; no vertical splitter
        # is used to squeeze the model when a tab is selected.
        top_splitter = QSplitter(Qt.Horizontal)
        self.top_splitter = top_splitter
        self.main_splitter = top_splitter
        top_splitter.setChildrenCollapsible(False)
        top_splitter.setMinimumHeight(self._u(620))
        model_box = QGroupBox("UR5 3D 模型")
        model_box.setMinimumHeight(self._u(380))
        model_layout = QVBoxLayout(model_box)
        model_layout.setContentsMargins(self._u(6), self._u(6), self._u(6), self._u(6))
        model_layout.setSpacing(self._u(3))
        self.robot_view = Robot3DView()
        self.robot_view.setMinimumSize(self._u(480), self._u(300))
        self.robot_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        model_layout.addWidget(self.robot_view, 1)
        view_buttons = QHBoxLayout()
        view_buttons.setSpacing(self._u(4))
        self.view_buttons_widgets = []
        for text, view_name in [
            ("等轴", "iso"),
            ("俯视", "top"),
            ("正视", "front"),
            ("侧视", "side"),
            ("机器人", "robot"),
            ("机器人+工件", "robot_scene"),
        ]:
            button = QPushButton(text)
            button.setMinimumHeight(self._u(32))
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            button.setToolTip(f"设置 3D 视图：{text}")
            button.clicked.connect(lambda _checked=False, name=view_name: self._set_robot_view(name))
            view_buttons.addWidget(button)
            self.view_buttons_widgets.append(button)
        model_layout.addLayout(view_buttons)

        # Read-only scene layer switches.  They only change what the 3D widget
        # draws; none of them publish, subscribe or command anything.  Keep
        # Keep the switches in two compact rows so the additional live/preview
        # TCP and gun/wire layers remain readable even when the main splitter
        # is dragged close to 50:50.
        scene_toggles = QGridLayout()
        scene_toggles.setContentsMargins(0, 0, 0, 0)
        scene_toggles.setHorizontalSpacing(self._u(4))
        scene_toggles.setVerticalSpacing(0)
        self.scene_visibility_boxes = {}
        for index, (key, text) in enumerate([
            ("robot", "Robot"),
            ("workpiece", "Workpiece"),
            ("seam", "Seam"),
            ("tcp_path", "Planned TCP Path"),
            ("current_tcp", "Current TCP"),
            ("preview_tcp", "Preview TCP"),
            ("gun", "Gun Collision"),
            ("wire", "Wire Collision"),
            ("collision", "Collision Geometry"),
        ]):
            box = QCheckBox(text)
            box.setMinimumHeight(self._u(24))
            box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            box.setChecked(key in ("robot", "workpiece", "seam", "current_tcp", "preview_tcp", "gun", "wire", "collision"))
            box.setToolTip(f"显示/隐藏 {text}（仅显示，不影响控制）")
            box.stateChanged.connect(self._on_scene_visibility_changed)
            scene_toggles.addWidget(box, index // 5, index % 5)
            self.scene_visibility_boxes[key] = box
        for col in range(5):
            scene_toggles.setColumnStretch(col, 1)
        model_layout.addLayout(scene_toggles)
        self.scene_status_label = QLabel("")
        self.scene_status_label.setMinimumHeight(self._u(18))
        self.scene_status_label.setMaximumHeight(self._u(28))
        self.scene_status_label.setStyleSheet(f"color:#8fb4d8;font-size:{self._u(11)}px;")
        self.scene_status_label.setWordWrap(True)
        model_layout.addWidget(self.scene_status_label)
        self._refresh_scene_status_label()
        top_splitter.addWidget(model_box)

        right_content = QWidget()
        right_content.setMinimumWidth(self._u(520))
        right_content.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        right_layout = QVBoxLayout(right_content)
        right_layout.setContentsMargins(self._u(4), self._u(4), self._u(4), self._u(4))
        right_layout.setSpacing(self._u(4))

        status_box = QGroupBox("机器人状态")
        status_box.setMinimumWidth(0)
        status_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Minimum)
        status_layout = QVBoxLayout(status_box)
        self.status_layout = status_layout
        status_layout.setContentsMargins(self._u(4), self._u(4), self._u(4), self._u(4))
        status_layout.setSpacing(self._u(2))
        self.status_labels = {}
        self.status_rows = {}
        self.status_name_labels = []
        for key, text in [
            ("driver", "ROS2 Driver"),
            ("program", "Robot Program"),
            ("mode", "robot_mode"),
            ("safety", "safety_mode"),
            ("speed", "speed_scaling"),
            ("controller", "scaled controller"),
            ("joint", "Joint State"),
        ]:
            row = StatusRow(text, self.ui_scale)
            self.status_rows[key] = row
            self.status_labels[key] = row.value_label
            self.status_name_labels.append(row.name_label)
            status_layout.addWidget(row)

        tcp_box = QGroupBox("TCP 状态（只读）")
        # Retain the full diagnostic card even though it is no longer shown
        # on the compact Run Log page.  Its labels are still updated by the
        # read-only state monitor and may be useful to future diagnostics.
        self._tcp_box_keepalive = tcp_box
        tcp_box.setMinimumWidth(0)
        tcp_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Minimum)
        tcp_layout = QVBoxLayout(tcp_box)
        tcp_layout.setSpacing(self._u(6))
        self.tcp_labels = {}
        self.tcp_rows = []
        self.tcp_row_labels = []
        self.tcp_cards = []
        for title, entries, unit in [
            ("TCP POSITION", [("x", "X"), ("y", "Y"), ("z", "Z")], "mm"),
            ("TCP ORIENTATION", [("rx", "Rx"), ("ry", "Ry"), ("rz", "Rz")], "deg"),
        ]:
            card = QWidget()
            card.setObjectName("tcp_card")
            card.setStyleSheet("QWidget#tcp_card { border:1px solid #d5d9de; border-radius:4px; }")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(self._u(6), self._u(4), self._u(6), self._u(4))
            card_layout.setSpacing(self._u(1))
            title_label = QLabel(title)
            title_label.setStyleSheet(f"font-size:{self._u(14)}px;font-weight:bold;")
            card_layout.addWidget(title_label)
            self.tcp_cards.append(card)
            for key, text in entries:
                row = QHBoxLayout()
                row.setSpacing(self._u(6))
                row_label = QLabel(text)
                row_label.setMinimumWidth(self._u(45))
                value_label = QLabel("—")
                value_label.setMinimumWidth(self._u(130))
                value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                value_label.setStyleSheet(f"font-size:{self._u(16)}px;font-family:monospace;")
                unit_label = QLabel(unit)
                unit_label.setMinimumWidth(self._u(45))
                unit_label.setStyleSheet(f"font-size:{self._u(13)}px;color:#586174;")
                self.tcp_labels[key] = value_label
                row.addWidget(row_label)
                row.addWidget(value_label, 1)
                row.addWidget(unit_label)
                card_layout.addLayout(row)
                self.tcp_rows.append(row)
                self.tcp_row_labels.append((row_label, value_label, unit_label))
            tcp_layout.addWidget(card)

        safety_box = QGroupBox("安全控制")
        safety_box.setMinimumWidth(0)
        safety_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Minimum)
        safety_layout = QVBoxLayout(safety_box)
        self.safety_layout = safety_layout
        safety_layout.setContentsMargins(self._u(4), self._u(4), self._u(4), self._u(4))
        safety_content = QVBoxLayout()
        safety_content.setSpacing(self._u(6))
        self.stop_button = QPushButton("STOP MOTION")
        self.stop_button.setMinimumHeight(self._u(56))
        self.stop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.stop_button.setStyleSheet("background:#c62828;color:white;font-size:18px;font-weight:bold;")
        self.stop_button.clicked.connect(self._stop_motion)
        safety_content.addWidget(self.stop_button)
        warning = QLabel("⚠ 软件停止不能替代示教器急停")
        self.warning_label = warning
        warning.setContentsMargins(0, self._u(8), 0, self._u(10))
        warning.setMinimumHeight(self._u(30))
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#9b1c1c;font-size:14px;font-weight:bold;")
        warning_power_row = QHBoxLayout()
        warning_power_row.setSpacing(self._u(6))
        warning_power_row.addWidget(warning, 1)
        self.power_status_label = QLabel("电源：未知")
        self.power_status_label.setAlignment(Qt.AlignCenter)
        self.power_status_label.setMinimumWidth(self._u(110))
        self.power_status_label.setMinimumHeight(self.metrics.status_row)
        self.power_status_label.setMaximumHeight(self.metrics.status_row)
        safety_content.addLayout(warning_power_row)
        safety_button_row = QHBoxLayout()
        safety_button_row.setSpacing(self._u(6))
        self.refresh_button = QPushButton("刷新状态")
        self.refresh_button.setMinimumHeight(42)
        self.refresh_button.clicked.connect(lambda: self.worker.request_refresh.emit())
        safety_button_row.addWidget(self.refresh_button)
        self.rviz_button = QPushButton("打开 RViz 3D 模型")
        self.rviz_button.setMinimumHeight(42)
        self.rviz_button.clicked.connect(self._open_rviz)
        safety_button_row.addWidget(self.rviz_button)
        self.ui_scale_combo = QComboBox()
        self.ui_scale_combo.addItems(["100%", "125%", "150%"])
        self.ui_scale_combo.setCurrentText(f"{round(self.ui_scale * 100):.0f}%")
        self.ui_scale_combo.setMinimumHeight(36)
        self.ui_scale_combo.currentTextChanged.connect(self._ui_scale_changed)
        safety_button_row.addWidget(QLabel("缩放"))
        safety_button_row.addWidget(self.ui_scale_combo, 1)
        safety_content.addLayout(safety_button_row)
        safety_layout.addLayout(safety_content)
        # Compact summary shown permanently in the right header.  The
        # original ROS/controller/TCP cards are retained as diagnostic
        # widgets and moved to the Run Log tab below; no state logic changes.
        simple_box = QGroupBox("机器人状态")
        simple_layout = QGridLayout(simple_box)
        simple_layout.setContentsMargins(self._u(8), self._u(6), self._u(8), self._u(6))
        simple_layout.setHorizontalSpacing(self._u(8))
        simple_layout.setVerticalSpacing(self._u(4))
        self.simple_state_labels = {}
        for row, (key, title) in enumerate((
            ("power", "电源"),
            ("robot", "机器人"),
            ("safety", "安全"),
        )):
            name = QLabel(title)
            value = self.power_status_label if key == "power" else QLabel("未知")
            value.setAlignment(Qt.AlignCenter)
            value.setMinimumHeight(self.metrics.status_row)
            value.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.simple_state_labels[key] = value
            simple_layout.addWidget(name, row, 0)
            simple_layout.addWidget(value, row, 1)
        simple_layout.setColumnStretch(1, 1)

        right_layout.addWidget(safety_box)
        right_layout.addWidget(simple_box)
        if UI_DEBUG_LAYOUT:
            status_box.setStyleSheet("QGroupBox { border:2px solid #d33; }")
            tcp_box.setStyleSheet("QGroupBox { border:2px solid #2673c9; }")
            safety_box.setStyleSheet("QGroupBox { border:2px solid #d5a400; }")
        top_splitter.addWidget(right_content)
        top_splitter.setStretchFactor(0, 1)
        top_splitter.setStretchFactor(1, 1)
        top_splitter.setSizes([800, 800])

        tabs = QTabWidget()
        self.tabs = tabs
        self.ui_tabs = tabs
        # The Jog tab needs enough vertical room to show all six rows.  Other
        # tabs keep the larger 3D area requested for monitoring.  This only
        # changes the user-visible splitter preference; it does not affect
        # any motion or safety logic.
        tabs.currentChanged.connect(self._on_main_tab_changed)
        tabs.setMinimumHeight(self._u(280))
        tabs.setStyleSheet(
            "QTabBar::tab { min-width:150px; min-height:42px; font-size:16px; padding:7px 18px; }"
            "QTabWidget::pane { border:1px solid #d5d9de; }"
        )

        # Tab 1: main jog control.
        jog_tab = QWidget()
        jog_tab_layout = QVBoxLayout(jog_tab)
        joint_box = QGroupBox("六关节实时状态与单关节点动")
        joint_box.setStyleSheet(
            "QGroupBox { font-size:18px; font-weight:bold; }"
            "QLabel { font-size:17px; }"
            "QPushButton { font-size:17px; padding:5px 12px; }"
        )
        joint_layout = QVBoxLayout(joint_box)
        joint_layout.setContentsMargins(12, 12, 12, 12)
        joint_layout.setSpacing(10)

        step_row = QHBoxLayout()
        step_row.setAlignment(Qt.AlignVCenter)
        step_row.setSpacing(self._u(8))
        step_row.addWidget(QLabel("步长"))
        self.step_deg = float(self.config.get("jog", {}).get("step_deg", 0.5))
        self.step_group = QButtonGroup(self)
        self.step_buttons = []
        for value in (0.1, 0.5, 1.0, 2.0):
            button = QPushButton(f"{value:g}°")
            button.setCheckable(True)
            button.setMinimumSize(self._u(72), self.metrics.button_height)
            button.clicked.connect(lambda _checked=False, v=value: self._set_step_size(v))
            self.step_group.addButton(button)
            self.step_buttons.append(button)
            step_row.addWidget(button)
            if abs(value - self.step_deg) < 1e-6:
                button.setChecked(True)
        step_row.addSpacing(self._u(16))
        step_row.addWidget(QLabel("速度"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, self.max_hmi_speed_percent)
        self.speed_slider.setValue(min(
            self.max_hmi_speed_percent,
            max(1, int(self.config.get("jog", {}).get("default_speed_percent", self.config.get("jog", {}).get("speed_percent", 5)))),
        ))
        self.speed_slider.setMinimumWidth(self._u(320))
        self.speed_slider.setMinimumHeight(36)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        step_row.addWidget(self.speed_slider, 1)
        self.speed_value_label = QLabel(f"{self.speed_slider.value()} %")
        self.speed_value_label.setMinimumWidth(70)
        self.speed_value_label.setStyleSheet("font-size:20px;font-weight:bold;")
        step_row.addWidget(self.speed_value_label)
        for value in (5, 10, 20):
            button = QPushButton(f"{value}%")
            button.setMinimumSize(self._u(60), self.metrics.button_height)
            button.clicked.connect(lambda _checked=False, v=value: self.speed_slider.setValue(v))
            step_row.addWidget(button)
            self.speed_shortcuts = getattr(self, "speed_shortcuts", [])
            self.speed_shortcuts.append(button)
        joint_layout.addLayout(step_row)

        self.joint_table = QTableWidget(6, 4)
        self.joint_table.setHorizontalHeaderLabels(["关节", "当前角度", "连续值", "Jog"])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.joint_table.setMinimumHeight(self._u(44) + self.metrics.joint_row * 6 + self._u(8))
        table_height = self._u(44) + self.metrics.joint_row * 6 + self._u(8)
        self.joint_table.setMaximumHeight(table_height)
        self.joint_table.setFont(self.font())
        self.joint_table.setStyleSheet(
            "QTableWidget { font-size:17px; }"
            "QTableWidget::item { padding:8px; }"
            "QHeaderView::section { font-size:17px; font-weight:bold; padding:8px; }"
        )
        self.joint_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.joint_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.joint_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.joint_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.joint_table.horizontalHeader().setDefaultAlignment(Qt.AlignCenter)
        self.joint_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for column in (1, 2, 3):
            self.joint_table.horizontalHeader().setSectionResizeMode(column, QHeaderView.Fixed)
        self.joint_table.verticalHeader().setDefaultSectionSize(self.metrics.joint_row)
        self.jog_widgets = []
        for i, _name in enumerate(JOINT_NAMES):
            self.joint_table.setItem(i, 0, QTableWidgetItem(f"J{i + 1}  {JOINT_CN[i]}"))
            for col in (1, 2):
                self.joint_table.setItem(i, col, QTableWidgetItem("—"))
            jog = JogButtons(i, self._jog_clicked, self.ui_scale)
            self.joint_table.setCellWidget(i, 3, jog)
            self.jog_widgets.append(jog)
        self.joint_table.setColumnWidth(1, self._u(150))
        self.joint_table.setColumnWidth(2, self._u(150))
        self.joint_table.setColumnWidth(3, self._u(190))
        joint_layout.addWidget(self.joint_table, 1)

        jog_content = QHBoxLayout()
        jog_content.setSpacing(12)
        jog_content.addWidget(joint_box, 1)

        jog_status_box = QGroupBox("Jog 状态 / 当前指令")
        jog_status_layout = QVBoxLayout(jog_status_box)
        jog_status_layout.setSpacing(self._u(6))
        self.jog_status_labels = {}
        self.jog_status_rows = {}
        for key, text in [
            ("motion", "Motion State"),
            ("goal", "Current Goal ID"),
            ("action", "Action State"),
            ("joint", "Current Joint"),
            ("target", "Target"),
            ("command_speed", "Command Speed"),
            ("robot_speed", "Robot Speed Scaling"),
            ("start_check", "Start Check"),
            ("control_source", "Control Source"),
        ]:
            row = StatusRow(text, self.ui_scale)
            row.set_value("—", "unknown", self.ui_scale)
            self.jog_status_rows[key] = row
            self.jog_status_labels[key] = row.value_label
            jog_status_layout.addWidget(row)
        safety_row = QHBoxLayout()
        safety_row.addWidget(QLabel("安全停止"))
        safety_row.addStretch(1)
        safety_row.addWidget(QLabel("右上角 STOP MOTION"))
        jog_status_layout.addLayout(safety_row)
        jog_status_layout.addStretch(1)
        # Do not place the Jog diagnostic card beside the controls.  It is
        # retained for the Run Log/diagnostic page so the main Jog tab can use
        # the full right pane for J1--J6 and their buttons.
        self.jog_status_box = jog_status_box
        jog_tab_layout.addLayout(jog_content, 1)
        tabs.addTab(jog_tab, "关节点动")

        # Tab 2: trajectory import, validation, preview and gated execution.
        trajectory_tab = QWidget()
        trajectory_layout = QVBoxLayout(trajectory_tab)
        trajectory_layout.setSpacing(self._u(8))
        traj_box = QGroupBox("轨迹文件（默认不执行）")
        traj_layout = QHBoxLayout(traj_box)
        self.load_button = QPushButton("加载轨迹")
        self.load_button.setMinimumSize(130, 45)
        self.load_button.clicked.connect(self._load_path)
        traj_layout.addWidget(self.load_button)
        self.validate_button = QPushButton("验证轨迹")
        self.validate_button.setMinimumSize(130, 45)
        self.validate_button.clicked.connect(self._validate_path)
        traj_layout.addWidget(self.validate_button)
        self.preview_button = QPushButton("3D轨迹预演")
        self.preview_button.setMinimumSize(145, 45)
        self.preview_button.clicked.connect(self._start_preview)
        traj_layout.addWidget(self.preview_button)
        self.execute_button = QPushButton("低速执行（5%）")
        self.execute_button.setMinimumSize(175, 45)
        self.execute_button.setEnabled(False)
        self.execute_button.clicked.connect(self._execute_path)
        traj_layout.addWidget(self.execute_button)
        self.path_label = QLabel("未加载轨迹")
        self.path_label.setStyleSheet("font-size:16px;")
        traj_layout.addWidget(self.path_label, 1)
        trajectory_layout.addWidget(traj_box)

        summary_box = QGroupBox("轨迹信息与验证")
        summary_layout = QGridLayout(summary_box)
        summary_layout.setContentsMargins(self._u(8), self._u(8), self._u(8), self._u(8))
        summary_layout.setHorizontalSpacing(self._u(10))
        summary_layout.setVerticalSpacing(self._u(4))
        self.trajectory_info_labels = {}
        summary_fields = [
            ("name", "Trajectory Name", "—"),
            ("source", "Source", "—"),
            ("coordinate", "Coordinate Space", "—"),
            ("collision", "Collision Check", "—"),
            ("points", "Point Count", "—"),
            ("duration", "Duration", "—"),
            ("start", "Start Joint", "—"),
            ("end", "End Joint", "—"),
            ("travel", "Max Joint Travel", "—"),
            ("step", "Max Adjacent Step", "—"),
            ("velocity", "Estimated Max Velocity", "—"),
            ("acceleration", "Estimated Max Acceleration", "—"),
            ("start_error", "Start Error (J1..J6)", "—"),
            ("validation", "Validation", "NOT VALIDATED"),
        ]
        for index, (key, title, value) in enumerate(summary_fields):
            row, col = divmod(index, 2)
            title_label = QLabel(title)
            title_label.setStyleSheet("font-weight:bold;")
            value_label = QLabel(value)
            value_label.setWordWrap(True)
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.trajectory_info_labels[key] = value_label
            summary_layout.addWidget(title_label, row, col * 2)
            summary_layout.addWidget(value_label, row, col * 2 + 1)
        summary_layout.setColumnStretch(1, 1)
        summary_layout.setColumnStretch(3, 1)
        trajectory_layout.addWidget(summary_box)

        preview_box = QGroupBox("3D轨迹预演（仅更新左侧模型，不发送 ROS2 指令）")
        preview_layout = QHBoxLayout(preview_box)
        self.preview_play_button = QPushButton("播放")
        self.preview_play_button.setMinimumSize(90, 40)
        self.preview_play_button.clicked.connect(self._start_preview)
        preview_layout.addWidget(self.preview_play_button)
        self.preview_pause_button = QPushButton("暂停")
        self.preview_pause_button.setMinimumSize(90, 40)
        self.preview_pause_button.clicked.connect(self._pause_preview)
        preview_layout.addWidget(self.preview_pause_button)
        self.preview_stop_button = QPushButton("停止")
        self.preview_stop_button.setMinimumSize(90, 40)
        self.preview_stop_button.clicked.connect(self._stop_preview)
        preview_layout.addWidget(self.preview_stop_button)
        preview_layout.addWidget(QLabel("速度"))
        self.preview_speed_combo = QComboBox()
        self.preview_speed_combo.addItems(["0.5x", "1x", "2x"])
        self.preview_speed_combo.setCurrentText("1x")
        preview_layout.addWidget(self.preview_speed_combo)
        self.preview_slider = QSlider(Qt.Horizontal)
        self.preview_slider.setRange(0, 0)
        self.preview_slider.valueChanged.connect(self._preview_slider_changed)
        preview_layout.addWidget(self.preview_slider, 1)
        self.preview_status_label = QLabel("未加载轨迹")
        self.preview_status_label.setMinimumWidth(230)
        preview_layout.addWidget(self.preview_status_label)
        trajectory_layout.addWidget(preview_box)

        self.final_gate_label = QLabel("FINAL EXECUTION CHECK：BLOCKED")
        self.final_gate_label.setStyleSheet("background:#fff2cc;color:#8a5a00;font-weight:bold;padding:8px;")
        self.final_gate_label.setToolTip("必须通过所有状态、轨迹、起点和控制器检查后才允许执行")
        trajectory_layout.addWidget(self.final_gate_label)
        self.final_gate_details_label = QLabel("ROS / Program / Safety / Controller / Start：等待检查")
        self.final_gate_details_label.setWordWrap(True)
        trajectory_layout.addWidget(self.final_gate_details_label)
        self.execution_progress_label = QLabel("Trajectory Progress：—    Current point：—    Elapsed：—    Remaining：—    Action State：—")
        trajectory_layout.addWidget(self.execution_progress_label)
        trajectory_layout.addStretch(1)
        trajectory_scroll = QScrollArea()
        trajectory_scroll.setWidgetResizable(True)
        trajectory_scroll.setFrameShape(QFrame.NoFrame)
        trajectory_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        trajectory_scroll.setWidget(trajectory_tab)
        self.trajectory_scroll = trajectory_scroll
        tabs.addTab(trajectory_scroll, "轨迹执行")
        self.trajectory_tab_index = tabs.indexOf(trajectory_scroll)

        # Tab 3: asynchronous offline MATLAB multi-algorithm planner.  Keep
        # the controls and data flow unchanged; this layout intentionally
        # separates setup, run control, summary and MATLAB output so the page
        # remains readable at 100/125/150% UI scale.
        online_scroll = QScrollArea()
        online_scroll.setWidgetResizable(True)
        online_scroll.setFrameShape(QFrame.NoFrame)
        online_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        online_page = QWidget()
        online_layout = QVBoxLayout(online_page)
        online_layout.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        online_layout.setSpacing(self._u(8))
        online_scroll.setWidget(online_page)
        self.online_scroll = online_scroll
        self.online_page = online_page

        setup_box = QGroupBox("1. 规划设置")
        setup_box.setMinimumHeight(self._u(210))
        self.online_setup_box = setup_box
        setup_grid = QGridLayout(setup_box)
        setup_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        setup_grid.setHorizontalSpacing(self._u(12))
        setup_grid.setVerticalSpacing(self._u(7))
        setup_grid.setColumnMinimumWidth(0, self._u(74))
        setup_grid.setColumnMinimumWidth(2, self._u(74))
        for _row in range(5):
            setup_grid.setRowMinimumHeight(_row, self._u(36))
        setup_grid.setRowMinimumHeight(5, self._u(48))

        self.online_field_widgets = {}
        self.online_field_widgets["task_label"] = QLabel("任务类型")
        setup_grid.addWidget(self.online_field_widgets["task_label"], 0, 0)
        self.online_task_combo = QComboBox()
        self.online_task_combo.addItem("RealHome → PreWeld", "RealHome_to_PreWeld")
        self.online_task_combo.addItem("Weld Seam", "Weld_Seam")
        self.online_task_combo.addItem("PostWeld → RealHome", "PostWeld_to_RealHome")
        self.online_task_combo.addItem("Full Welding Task", "Full_Welding_Task")
        self.online_task_combo.currentIndexChanged.connect(lambda _i: self._refresh_online_goal_display())
        setup_grid.addWidget(self.online_task_combo, 0, 1, 1, 3)

        self.online_field_widgets["start_label"] = QLabel("起点")
        setup_grid.addWidget(self.online_field_widgets["start_label"], 1, 0)
        self.online_start_combo = QComboBox()
        self.online_start_combo.addItem("Current Robot（最新 /joint_states）", "current")
        self.online_start_combo.addItem("Saved RealHome（HMI配置文件）", "real_home")
        self.online_start_combo.currentIndexChanged.connect(lambda _index: self._update_online_start_display())
        setup_grid.addWidget(self.online_start_combo, 1, 1)
        self.online_field_widgets["goal_label"] = QLabel("目标")
        setup_grid.addWidget(self.online_field_widgets["goal_label"], 1, 2)
        self.online_goal_label = QLabel("PreWeld")
        self.online_goal_label.setStyleSheet("font-weight:bold;color:#1f2937;")
        self.online_goal_label.setWordWrap(True)
        setup_grid.addWidget(self.online_goal_label, 1, 3)

        self.online_field_widgets["free_algorithm_label"] = QLabel("自由空间算法")
        setup_grid.addWidget(self.online_field_widgets["free_algorithm_label"], 2, 0)
        self.online_algorithm_combo = QComboBox()
        self.online_algorithm_combo.addItem("RRT", "RRT")
        self.online_algorithm_combo.addItem("RRT-Connect", "RRT-Connect")
        self.online_algorithm_combo.addItem("Bi-RRT*", "Bi-RRT*")
        self.online_algorithm_combo.addItem("V1 FastConnectAnytimeRRTStar", "V1 FastConnectAnytimeRRTStar")
        configured_algorithm = str(self.config.get("online_planning", {}).get(
            "default_algorithm", "V1 FastConnectAnytimeRRTStar"))
        match = self.online_algorithm_combo.findData(configured_algorithm)
        self.online_algorithm_combo.setCurrentIndex(match if match >= 0 else 3)
        self.online_algorithm_combo.setToolTip("自由空间段算法；焊缝段固定使用 A-ARC-IK+。")
        setup_grid.addWidget(self.online_algorithm_combo, 2, 1)
        self.online_field_widgets["weld_algorithm_label"] = QLabel("焊缝算法")
        setup_grid.addWidget(self.online_field_widgets["weld_algorithm_label"], 2, 2)
        self.online_weld_algorithm_label = QLabel("A-ARC-IK+（固定）")
        self.online_weld_algorithm_label.setStyleSheet(
            "background:#e8f5e9;color:#146c2e;font-weight:bold;padding:4px 8px;"
        )
        self.online_weld_algorithm_label.setWordWrap(True)
        self.online_weld_algorithm_label.setToolTip(
            "replan_weld_actual_axis_adaptive_feasibility.m + branchReferenceQ + Forward→Reverse"
        )
        setup_grid.addWidget(self.online_weld_algorithm_label, 2, 3)

        self.online_field_widgets["seed_label"] = QLabel("随机种子")
        setup_grid.addWidget(self.online_field_widgets["seed_label"], 3, 0)
        self.online_seed_spin = QSpinBox()
        self.online_seed_spin.setRange(1, 2147483647)
        self.online_seed_spin.setValue(int(self.config.get("online_planning", {}).get("seed", 20260917)))
        setup_grid.addWidget(self.online_seed_spin, 3, 1)
        self.online_field_widgets["budget_label"] = QLabel("时间预算")
        setup_grid.addWidget(self.online_field_widgets["budget_label"], 3, 2)
        self.online_budget_spin = QSpinBox()
        self.online_budget_spin.setRange(5, 600)
        self.online_budget_spin.setSuffix(" s")
        self.online_budget_spin.setValue(int(self.config.get("online_planning", {}).get(
            "max_planning_time_sec", 30.0)))
        setup_grid.addWidget(self.online_budget_spin, 3, 3)

        self.online_field_widgets["source_label"] = QLabel("目标来源")
        setup_grid.addWidget(self.online_field_widgets["source_label"], 4, 0)
        self.online_goal_source_label = QLabel("—")
        self.online_goal_source_label.setStyleSheet("color:#4b5563;")
        self.online_goal_source_label.setWordWrap(True)
        setup_grid.addWidget(self.online_goal_source_label, 4, 1)
        setup_grid.addWidget(QLabel("状态"), 4, 2)
        self.online_field_widgets["snapshot_label"] = QLabel("起点快照")
        setup_grid.addWidget(self.online_field_widgets["snapshot_label"], 5, 0)
        self.online_start_snapshot_label = QLabel("等待最新关节反馈")
        self.online_start_snapshot_label.setStyleSheet(
            "background:#f3f4f6;color:#374151;padding:6px 8px;"
        )
        self.online_start_snapshot_label.setWordWrap(True)
        setup_grid.addWidget(self.online_start_snapshot_label, 5, 1, 1, 3)
        self.online_task_status_label = QLabel("READY")
        self.online_task_status_label.setStyleSheet("color:#146c2e;font-weight:bold;")
        setup_grid.addWidget(self.online_task_status_label, 4, 3)
        self.online_field_widgets.update({
            "free_algorithm_label": self.online_field_widgets["free_algorithm_label"],
            "free_algorithm": self.online_algorithm_combo,
            "weld_algorithm_label": self.online_field_widgets["weld_algorithm_label"],
            "weld_algorithm": self.online_weld_algorithm_label,
            "start_label": self.online_field_widgets["start_label"],
            "start": self.online_start_combo,
        })
        online_layout.addWidget(setup_box)

        control_box = QGroupBox("2. 规划控制")
        control_layout = QGridLayout(control_box)
        control_layout.setContentsMargins(self._u(12), self._u(8), self._u(12), self._u(8))
        control_layout.setHorizontalSpacing(self._u(10))
        control_layout.setVerticalSpacing(self._u(5))
        self.online_start_button = QPushButton("开始规划")
        self.online_start_button.setMinimumSize(self._u(150), self._u(44))
        self.online_start_button.setStyleSheet("font-weight:bold;background:#d8f5dc;color:#146c2e;")
        self.online_start_button.clicked.connect(self._start_online_planning)
        control_layout.addWidget(self.online_start_button, 0, 0)
        self.online_cancel_button = QPushButton("取消规划")
        self.online_cancel_button.setMinimumSize(self._u(150), self._u(44))
        self.online_cancel_button.setEnabled(False)
        self.online_cancel_button.setToolTip(
            "只终止 MATLAB 规划进程，不发送任何机器人运动指令，也不使用 STOP MOTION。"
        )
        self.online_cancel_button.clicked.connect(self._cancel_online_planning)
        control_layout.addWidget(self.online_cancel_button, 0, 1)
        control_layout.addWidget(QLabel("当前阶段"), 0, 2)
        self.online_stage_label = QLabel("IDLE")
        self.online_stage_label.setMinimumWidth(self._u(230))
        self.online_stage_label.setStyleSheet(
            "background:#fff7d6;color:#8a6d00;font-weight:bold;padding:6px 10px;"
        )
        control_layout.addWidget(self.online_stage_label, 0, 3)
        self.online_status_label = QLabel("未开始")
        self.online_status_label.setWordWrap(True)
        self.online_status_label.setStyleSheet("color:#4b5563;padding-left:4px;")
        control_layout.addWidget(self.online_status_label, 1, 0, 1, 4)
        online_layout.addWidget(control_box)

        result_row = QHBoxLayout()
        result_row.setSpacing(self._u(8))
        basic_box = QGroupBox("3. 结果摘要")
        basic_grid = QGridLayout(basic_box)
        basic_grid.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        basic_grid.setHorizontalSpacing(self._u(10))
        basic_grid.setVerticalSpacing(self._u(4))
        detail_box = QGroupBox("焊缝 / 分段信息")
        detail_grid = QGridLayout(detail_box)
        detail_grid.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        detail_grid.setHorizontalSpacing(self._u(10))
        detail_grid.setVerticalSpacing(self._u(4))
        self.online_result_labels = {}

        def add_metrics(grid, items):
            for index, (key, title) in enumerate(items):
                row, col = divmod(index, 2)
                value = QLabel("—")
                value.setTextInteractionFlags(Qt.TextSelectableByMouse)
                value.setWordWrap(True)
                value.setMinimumWidth(self._u(108))
                self.online_result_labels[key] = value
                grid.addWidget(QLabel(title), row, col * 2)
                grid.addWidget(value, row, col * 2 + 1)
            grid.setColumnStretch(1, 1)
            grid.setColumnStretch(3, 1)

        add_metrics(basic_grid, [
            ("task", "任务"), ("algorithm", "算法"), ("seed", "Seed"),
            ("planning_time", "规划时间"), ("path_points", "轨迹点数"),
            ("duration", "轨迹时长"), ("collision", "碰撞检查"),
            ("continuity", "连续性"), ("tcp_error", "TCP误差"),
            ("max_joint_jump", "最大关节跳变"),
        ])
        add_metrics(detail_grid, [
            ("seam_initial_points", "初始焊点"),
            ("seam_final_points", "最终焊点"),
            ("seam_exec_points", "执行点数"),
            ("seam_inserted_points", "插入点数"),
            ("seam_direction", "方向"),
            ("seam_result", "A-ARC-IK+"),
            ("stage_approach_points", "接近段"),
            ("stage_weld_points", "焊缝段"),
            ("stage_exit_points", "退出段"),
            ("stage_total_points", "总点数"),
        ])
        result_row.addWidget(basic_box, 1)
        result_row.addWidget(detail_box, 1)
        online_layout.addLayout(result_row)

        output_box = QGroupBox("4. MATLAB 输出（只读）")
        output_layout = QVBoxLayout(output_box)
        output_layout.setContentsMargins(self._u(8), self._u(6), self._u(8), self._u(6))
        self.online_output_view = QPlainTextEdit()
        self.online_output_view.setReadOnly(True)
        self.online_output_view.setPlaceholderText("规划阶段和 MATLAB 输出将显示在这里。")
        self.online_output_view.setMinimumHeight(self._u(82))
        self.online_output_view.setMaximumHeight(self._u(125))
        self.online_output_view.setStyleSheet("font-family:monospace;font-size:12px;")
        output_layout.addWidget(self.online_output_view)
        online_layout.addWidget(output_box)
        online_note = QLabel(
            "规划完成后自动加载到“轨迹执行”页；必须人工验证和3D预演。不会自动发送真机指令。"
        )
        online_note.setWordWrap(True)
        online_note.setStyleSheet("color:#6b7280;padding:2px 4px;")
        online_layout.addWidget(online_note)
        tabs.addTab(online_scroll, "在线规划")
        self.online_tab_index = tabs.indexOf(online_scroll)
        self._refresh_online_goal_display()
        self._update_online_start_display()
        self._reset_online_result()

        # Tab 4: three-point workpiece calibration.  Every point is captured
        # from the latest live TCP sample; no button in this page commands a
        # robot motion.
        calibration_tab = QWidget()
        calibration_scroll = QScrollArea()
        calibration_scroll.setWidgetResizable(True)
        calibration_scroll.setFrameShape(QFrame.NoFrame)
        calibration_scroll.setWidget(calibration_tab)
        calibration_layout = QVBoxLayout(calibration_tab)
        calibration_layout.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        calibration_layout.setSpacing(self._u(8))

        mode_box = QGroupBox("1. 标定模式")
        mode_grid = QGridLayout(mode_box)
        mode_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        mode_grid.setHorizontalSpacing(self._u(12))
        mode_grid.setVerticalSpacing(self._u(6))
        mode_grid.addWidget(QLabel("标定模式"), 0, 0)
        mode_grid.addWidget(QLabel("三点示教标定"), 0, 1)
        mode_grid.addWidget(QLabel("当前工件位姿源"), 1, 0)
        self.calibration_source_combo = QComboBox()
        self.calibration_source_combo.addItems(["SIMULATION", "REAL CALIBRATION"])
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.currentTextChanged.connect(self._calibration_source_changed)
        mode_grid.addWidget(self.calibration_source_combo, 1, 1)
        source_path = ROOT.parent / "scene_geometry.json"
        source_label = QLabel(
            f"仿真几何：{source_path}" if source_path.exists() else "仿真几何：scene_geometry.json（未找到）"
        )
        source_label.setWordWrap(True)
        source_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        mode_grid.addWidget(source_label, 0, 2, 2, 1)
        mode_grid.setColumnStretch(1, 1)
        mode_grid.setColumnStretch(2, 3)
        calibration_layout.addWidget(mode_box)

        points_box = QGroupBox("2. 三点示教（只读取当前 TCP）")
        points_grid = QGridLayout(points_box)
        points_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        points_grid.setHorizontalSpacing(self._u(10))
        points_grid.setVerticalSpacing(self._u(7))
        points_grid.setColumnStretch(2, 1)
        self.calibration_point_labels = {}

        def add_calibration_point_row(row, key, title, description):
            title_label = QLabel(f"{title}\n{description}")
            title_label.setMinimumWidth(self._u(140))
            title_label.setWordWrap(True)
            points_grid.addWidget(title_label, row, 0)
            button = QPushButton("读取当前 TCP")
            button.setMinimumSize(self._u(130), self._u(38))
            button.clicked.connect(lambda _checked=False, point_key=key: self._capture_calibration_point(point_key))
            points_grid.addWidget(button, row, 1)
            value = QLabel("X: —   Y: —   Z: — mm")
            value.setMinimumHeight(self._u(38))
            value.setStyleSheet("background:#f3f4f6;color:#374151;padding:6px 8px;font-family:monospace;")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setToolTip("等待有效、最新的实时 TCP")
            value.setWordWrap(True)
            points_grid.addWidget(value, row, 2)
            self.calibration_point_labels[key] = value

        add_calibration_point_row(0, "P0", "P0 工件原点", "工件坐标系原点")
        add_calibration_point_row(1, "PX", "PX 工件 +X 方向点", "用于确定 X 轴方向")
        add_calibration_point_row(2, "PY", "PY 工件 +Y 方向点", "用于确定 Y 轴方向")
        add_calibration_point_row(3, "P_CHECK", "P_CHECK 建议验证点", "可选重复测量 P0，检查误差")
        calibration_layout.addWidget(points_box)

        action_box = QGroupBox("3. 计算、恢复与显示（P_CHECK 可选）")
        action_layout = QGridLayout(action_box)
        action_layout.setContentsMargins(self._u(12), self._u(8), self._u(12), self._u(8))
        action_layout.setHorizontalSpacing(self._u(8))
        action_layout.setVerticalSpacing(self._u(7))
        self.calibration_compute_button = QPushButton("计算标定")
        self.calibration_save_button = QPushButton("保存标定")
        self.calibration_reset_button = QPushButton("重新标定")
        self.calibration_load_button = QPushButton("加载已保存标定")
        self.calibration_display_button = QPushButton("显示到3D（预览）")
        self.calibration_restore_button = QPushButton("恢复仿真位姿")
        for button in (self.calibration_compute_button, self.calibration_save_button,
                       self.calibration_reset_button, self.calibration_load_button,
                       self.calibration_display_button, self.calibration_restore_button):
            button.setMinimumHeight(self._u(40))
        self.calibration_compute_button.clicked.connect(self._calculate_calibration)
        self.calibration_save_button.clicked.connect(self._save_calibration)
        self.calibration_reset_button.clicked.connect(self._reset_calibration)
        self.calibration_load_button.clicked.connect(self._load_saved_calibration_action)
        self.calibration_display_button.clicked.connect(self._display_saved_calibration)
        self.calibration_restore_button.clicked.connect(self._restore_simulation_calibration)
        self.calibration_save_button.setEnabled(False)
        self.calibration_display_button.setEnabled(False)
        action_layout.addWidget(self.calibration_compute_button, 0, 0)
        action_layout.addWidget(self.calibration_save_button, 0, 1)
        action_layout.addWidget(self.calibration_reset_button, 0, 2)
        action_layout.addWidget(self.calibration_load_button, 1, 0)
        action_layout.addWidget(self.calibration_display_button, 1, 1)
        action_layout.addWidget(self.calibration_restore_button, 1, 2)
        action_layout.setColumnStretch(3, 1)
        calibration_layout.addWidget(action_box)

        result_box = QGroupBox("4. 标定结果")
        result_grid = QGridLayout(result_box)
        result_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        result_grid.setHorizontalSpacing(self._u(12))
        result_grid.setVerticalSpacing(self._u(5))
        self.calibration_result_labels = {}
        result_items = [
            ("position", "位置 (m)"), ("rpy", "RPY (ZYX, deg)"),
            ("residual", "残差 (mm)"), ("quality", "质量提示"),
            ("status", "标定状态"),
            ("check", "P_CHECK 预测 / 实测"),
        ]
        for row, (key, title) in enumerate(result_items):
            result_grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setMinimumHeight(self._u(28))
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            result_grid.addWidget(value, row, 1, 1, 3)
            self.calibration_result_labels[key] = value
        # Keep the optional P_CHECK row separate from the rotation matrix.
        # They previously shared row 5, which caused label overlap at 125%+
        # UI scale and on narrow windows.
        result_grid.addWidget(QLabel("旋转矩阵"), 6, 0, Qt.AlignTop)
        self.calibration_matrix_label = QLabel("—")
        self.calibration_matrix_label.setMinimumHeight(self._u(70))
        self.calibration_matrix_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.calibration_matrix_label.setStyleSheet("font-family:monospace;background:#f3f4f6;padding:6px;")
        self.calibration_matrix_label.setWordWrap(True)
        result_grid.addWidget(self.calibration_matrix_label, 6, 1, 1, 3)
        result_grid.setColumnStretch(1, 1)
        result_grid.setColumnStretch(3, 1)
        calibration_layout.addWidget(result_box)

        self.calibration_note = QLabel(
            "只读取实时 TCP，不自动移动机器人。三点几何有效即可保存为 DRAFT；"
            "P_CHECK 为建议验证。误差 ≤2 mm 绿色、2–5 mm 黄色、>5 mm 红色；"
            "5–6 mm 可在人工确认后申请应用，6–10 mm 仅可保存和做 3D 诊断预览；超过 10 mm 建议重新标定。"
            "这些是质量提示，不是官方精度标准。应用到真实工件规划仍需独立验证和人工确认。"
            f"保存文件：{self.calibration_file}"
        )
        self.calibration_note.setWordWrap(True)
        self.calibration_note.setStyleSheet("color:#6b7280;")
        calibration_layout.addWidget(self.calibration_note)
        calibration_layout.addStretch(1)
        tabs.addTab(calibration_scroll, "工件标定")

        # Tab 5: fixed-point TCP position calibration.  This page reads fresh
        # flange feedback/FK locally.  The separate panel below can read the
        # controller's active tcp_offset and, only after manual confirmation,
        # set that runtime value and verify it by RTDE readback.
        tcp_tab = QWidget()
        tcp_scroll = QScrollArea()
        tcp_scroll.setWidgetResizable(True)
        tcp_scroll.setFrameShape(QFrame.NoFrame)
        tcp_scroll.setWidget(tcp_tab)
        tcp_layout = QVBoxLayout(tcp_tab)
        tcp_layout.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        tcp_layout.setSpacing(self._u(8))

        tcp_intro = QLabel(
            "普通四点 TCP 标定：采集 4 组不同法兰姿态即可计算（可选第 5/6 组增加冗余）。"
            "每次使用最新关节角，由已标定 URDF 计算 base→flange；不使用已包含 TCP 偏移的 TCP 话题。"
        )
        tcp_intro.setWordWrap(True)
        tcp_intro.setStyleSheet("color:#4b5563;padding:2px 4px;")
        tcp_layout.addWidget(tcp_intro)

        tcp_mode_box = QGroupBox("1. 标定配置")
        tcp_mode_grid = QGridLayout(tcp_mode_box)
        tcp_mode_grid.setContentsMargins(self._u(12), self._u(9), self._u(12), self._u(9))
        tcp_mode_grid.setHorizontalSpacing(self._u(12))
        tcp_mode_grid.setVerticalSpacing(self._u(6))
        tcp_mode_grid.addWidget(QLabel("求解方法"), 0, 0)
        tcp_mode_grid.addWidget(QLabel("定点旋转法（只标定位置）"), 0, 1)
        tcp_mode_grid.addWidget(QLabel("法兰坐标"), 0, 2)
        tcp_mode_grid.addWidget(QLabel("URDF flange / base"), 0, 3)
        tcp_mode_grid.addWidget(QLabel("工具姿态"), 1, 0)
        tcp_mode_grid.addWidget(QLabel("保持现有设置"), 1, 1)
        tcp_mode_grid.addWidget(QLabel("冻结文件"), 1, 2)
        tcp_mode_grid.addWidget(QLabel("ur5_tool_calibration.mat（不覆盖）"), 1, 3)
        tcp_mode_grid.setColumnStretch(1, 1)
        tcp_mode_grid.setColumnStretch(3, 2)
        tcp_layout.addWidget(tcp_mode_box)

        tcp_samples_box = QGroupBox("2. 采集法兰姿态（普通四点，4 组即可）")
        tcp_samples_layout = QVBoxLayout(tcp_samples_box)
        tcp_samples_layout.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        tcp_samples_layout.setSpacing(self._u(6))
        tcp_actions = QHBoxLayout()
        self.tcp_capture_button = QPushButton("采集姿态")
        self.tcp_capture_button.setMinimumSize(self._u(130), self._u(42))
        self.tcp_capture_button.clicked.connect(self._capture_tcp_pose)
        self.tcp_clear_button = QPushButton("清空姿态")
        self.tcp_clear_button.setMinimumSize(self._u(110), self._u(42))
        self.tcp_clear_button.clicked.connect(self._clear_tcp_calibration)
        self.tcp_sample_count_label = QLabel("0 / 6（4 组即可）")
        self.tcp_sample_count_label.setStyleSheet("font-weight:bold;color:#4b5563;")
        tcp_actions.addWidget(self.tcp_capture_button)
        tcp_actions.addWidget(self.tcp_clear_button)
        tcp_actions.addWidget(self.tcp_sample_count_label)
        tcp_actions.addStretch(1)
        tcp_samples_layout.addLayout(tcp_actions)
        self.tcp_pose_table = QTableWidget(0, 4)
        self.tcp_pose_table.setHorizontalHeaderLabels(["姿态", "采集时间", "关节角（deg）", "flange 原点 base（mm）"])
        self.tcp_pose_table.verticalHeader().setVisible(False)
        self.tcp_pose_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.tcp_pose_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tcp_pose_table.setMinimumHeight(self._u(170))
        self.tcp_pose_table.setMaximumHeight(self._u(280))
        self.tcp_pose_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tcp_pose_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.tcp_pose_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.tcp_pose_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        tcp_samples_layout.addWidget(self.tcp_pose_table)
        tcp_layout.addWidget(tcp_samples_box)

        tcp_compute_box = QGroupBox("3. 计算 TCP（独立验证可选）")
        tcp_compute_layout = QGridLayout(tcp_compute_box)
        tcp_compute_layout.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        tcp_compute_layout.setHorizontalSpacing(self._u(10))
        tcp_compute_layout.setVerticalSpacing(self._u(6))
        self.tcp_compute_button = QPushButton("计算 TCP")
        self.tcp_compute_button.setMinimumSize(self._u(145), self._u(42))
        self.tcp_compute_button.setEnabled(False)
        self.tcp_compute_button.clicked.connect(self._compute_tcp_calibration)
        self.tcp_verify_capture_button = QPushButton("可选：采集验证姿态（0）")
        self.tcp_verify_capture_button.setMinimumSize(self._u(145), self._u(42))
        self.tcp_verify_capture_button.setEnabled(False)
        self.tcp_verify_capture_button.clicked.connect(self._capture_tcp_verification_pose)
        self.tcp_verify_button = QPushButton("独立验证（可选）")
        self.tcp_verify_button.setMinimumSize(self._u(110), self._u(42))
        self.tcp_verify_button.setEnabled(False)
        self.tcp_verify_button.clicked.connect(self._verify_tcp_calibration)
        self.tcp_compute_status_label = QLabel("等待 4 组不同姿态")
        self.tcp_compute_status_label.setWordWrap(True)
        self.tcp_compute_status_label.setStyleSheet("color:#6b7280;")
        tcp_compute_layout.addWidget(self.tcp_compute_button, 0, 0)
        tcp_compute_layout.addWidget(self.tcp_verify_capture_button, 0, 1)
        tcp_compute_layout.addWidget(self.tcp_verify_button, 0, 2)
        tcp_compute_layout.addWidget(self.tcp_compute_status_label, 0, 3)
        tcp_compute_layout.setColumnStretch(3, 1)
        tcp_layout.addWidget(tcp_compute_box)

        tcp_result_box = QGroupBox("4. 标定结果")
        tcp_result_grid = QGridLayout(tcp_result_box)
        tcp_result_grid.setContentsMargins(self._u(12), self._u(9), self._u(12), self._u(9))
        tcp_result_grid.setHorizontalSpacing(self._u(12))
        tcp_result_grid.setVerticalSpacing(self._u(5))
        self.tcp_result_labels = {}
        for row, (key, title) in enumerate([
            ("before", "仿真工具基线（非真机）"),
            ("offset", "求得 flange→TCP 位置"),
            ("pivot", "固定点 base 位置"),
            ("rank", "矩阵秩 / 条件数"),
            ("diversity", "姿态跨度"),
            ("residual", "RMS / 最大残差"),
            ("quality", "质量提示"),
            ("independent", "独立验证"),
            ("status", "Calibration Status"),
        ]):
            tcp_result_grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setStyleSheet("font-family:monospace;")
            tcp_result_grid.addWidget(value, row, 1, 1, 3)
            self.tcp_result_labels[key] = value
        tcp_result_grid.setColumnStretch(1, 1)
        tcp_layout.addWidget(tcp_result_box)

        tcp_save_box = QGroupBox("5. 保存与应用")
        tcp_save_layout = QHBoxLayout(tcp_save_box)
        tcp_save_layout.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        self.tcp_save_button = QPushButton("保存 real_tcp_calibration.json")
        self.tcp_save_button.setMinimumHeight(self._u(42))
        self.tcp_save_button.setEnabled(False)
        self.tcp_save_button.clicked.connect(self._save_tcp_calibration)
        self.tcp_apply_button = QPushButton("应用到软件显示 / 导出")
        self.tcp_apply_button.setMinimumHeight(self._u(42))
        self.tcp_apply_button.setEnabled(False)
        self.tcp_apply_button.setToolTip("仅软件标定对比与导出，不修改真机 TCP；旧轨迹失效")
        self.tcp_apply_button.clicked.connect(self._apply_tcp_calibration)
        tcp_save_layout.addWidget(self.tcp_save_button)
        tcp_save_layout.addWidget(self.tcp_apply_button)
        tcp_save_layout.addStretch(1)
        tcp_layout.addWidget(tcp_save_box)
        self.tcp_calibration_note = QLabel(
            "结果保存到 /home/ttbstt/jixiebi/UR5/real_tcp_calibration.json。"
            "RMS≤2 mm 为绿色、2–5 mm 为黄色、>5 mm 为红色；这是初步质量提示，不是 UR 官方精度标准。"
            "软件应用仅更新对比显示与导出，不会修改真机 TCP。"
            "真机修改请使用下方独立的读取/写入操作。冻结仿真标定不覆盖。"
        )
        self.tcp_calibration_note.setWordWrap(True)
        self.tcp_calibration_note.setStyleSheet("color:#6b7280;")
        tcp_layout.addWidget(self.tcp_calibration_note)
        self._init_robot_tcp_panel(tcp_layout)
        tcp_layout.addStretch(1)
        tabs.addTab(tcp_scroll, "TCP标定")
        self.tcp_tab_index = tabs.indexOf(tcp_scroll)
        self._reset_tcp_calibration_ui()

        # Tab 6: offline gun-installation adjustment.  All controls here are
        # display/calibration bookkeeping only: no ROS publisher, action,
        # pendant TCP write, or MATLAB scene write is reachable from them.
        gun_tab = QWidget()
        gun_scroll = QScrollArea()
        gun_scroll.setWidgetResizable(True)
        gun_scroll.setFrameShape(QFrame.NoFrame)
        gun_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        gun_scroll.setWidget(gun_tab)
        gun_layout = QVBoxLayout(gun_tab)
        gun_layout.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        gun_layout.setSpacing(self._u(8))

        gun_intro = QLabel(
            "焊枪安装姿态调整（离线3D预览）：绕 URDF flange/J6 实际局部轴旋转并做有物理依据的安装偏差微调。"
            "红色 LIVE TCP 来自示教器当前生效 TCP；调整焊枪不会改变 TCP、关节或 MATLAB 冻结场景。"
        )
        gun_intro.setWordWrap(True)
        gun_intro.setStyleSheet("color:#4b5563;padding:2px 4px;")
        gun_layout.addWidget(gun_intro)

        gun_adjust_box = QGroupBox("1. 法兰局部安装调整（预览）")
        gun_grid = QGridLayout(gun_adjust_box)
        gun_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        gun_grid.setHorizontalSpacing(self._u(10))
        gun_grid.setVerticalSpacing(self._u(7))
        gun_grid.addWidget(QLabel("旋转轴（flange）"), 0, 0)
        self.gun_axis_label = QLabel("读取中…")
        self.gun_axis_label.setStyleSheet("font-family:monospace;font-weight:bold;")
        gun_grid.addWidget(self.gun_axis_label, 0, 1, 1, 3)
        gun_grid.addWidget(QLabel("绕 J6 轴角度"), 1, 0)
        self.gun_roll_spin = QDoubleSpinBox()
        self.gun_roll_spin.setRange(-180.0, 180.0)
        self.gun_roll_spin.setDecimals(1)
        self.gun_roll_spin.setSingleStep(0.1)
        self.gun_roll_spin.setSuffix(" °")
        self.gun_roll_spin.setValue(90.0)
        self.gun_roll_spin.setMinimumHeight(self._u(36))
        self.gun_roll_spin.valueChanged.connect(self._on_gun_mount_adjustment_changed)
        gun_grid.addWidget(self.gun_roll_spin, 1, 1)
        self.gun_roll_label = QLabel("+90.0°")
        self.gun_roll_label.setStyleSheet("font-family:monospace;font-weight:bold;")
        gun_grid.addWidget(self.gun_roll_label, 1, 2)
        roll_buttons = QHBoxLayout()
        self.gun_roll_buttons = []
        for text, delta in (("−1°", -1.0), ("+1°", 1.0), ("−0.1°", -0.1), ("+0.1°", 0.1)):
            button = QPushButton(text)
            button.setMinimumSize(self._u(70), self._u(36))
            button.clicked.connect(lambda _checked=False, d=delta: self._nudge_gun_roll(d))
            roll_buttons.addWidget(button)
            self.gun_roll_buttons.append(button)
        for text, value in (("−90°", -90.0), ("+90°", 90.0)):
            button = QPushButton(text)
            button.setMinimumSize(self._u(70), self._u(36))
            button.clicked.connect(lambda _checked=False, v=value: self.gun_roll_spin.setValue(v))
            roll_buttons.addWidget(button)
        gun_grid.addLayout(roll_buttons, 1, 3)

        gun_grid.addWidget(QLabel("法兰局部平移（mm）"), 2, 0)
        self.gun_translation_spins = []
        translation_row = QHBoxLayout()
        for axis_name in ("X", "Y", "Z"):
            spin = QDoubleSpinBox()
            spin.setRange(-50.0, 50.0)
            spin.setDecimals(1)
            spin.setSingleStep(0.1)
            spin.setSuffix(f" {axis_name} mm")
            spin.setValue(0.0)
            spin.setMinimumHeight(self._u(36))
            spin.valueChanged.connect(self._on_gun_mount_adjustment_changed)
            translation_row.addWidget(spin)
            self.gun_translation_spins.append(spin)
        gun_grid.addLayout(translation_row, 2, 1, 1, 3)
        self.gun_mount_status_label = QLabel("PREVIEW ONLY")
        self.gun_mount_status_label.setWordWrap(True)
        self.gun_mount_status_label.setStyleSheet("color:#8a5a00;font-weight:bold;")
        gun_grid.addWidget(self.gun_mount_status_label, 3, 0, 1, 4)
        gun_layout.addWidget(gun_adjust_box)

        gun_view_box = QGroupBox("2. 焊枪 / 焊丝 / LIVE TCP 观察")
        gun_view_grid = QGridLayout(gun_view_box)
        gun_view_grid.setContentsMargins(self._u(12), self._u(9), self._u(12), self._u(9))
        gun_view_grid.setHorizontalSpacing(self._u(10))
        gun_view_grid.setVerticalSpacing(self._u(6))
        self.gun_visibility_boxes = {}
        for col, (key, text, checked) in enumerate((
            ("robot", "显示机器人", True),
            ("gun", "显示焊枪", True),
            ("wire", "显示焊丝", True),
            ("current_tcp", "显示 LIVE TCP", True),
        )):
            box = QCheckBox(text)
            box.setChecked(checked)
            box.stateChanged.connect(self._on_gun_visibility_changed)
            gun_view_grid.addWidget(box, 0, col)
            self.gun_visibility_boxes[key] = box
        view_row = QHBoxLayout()
        for text, name in (("等轴", "iso"), ("俯视", "top"), ("正视", "front"), ("侧视", "side"), ("焊枪视图", "gun")):
            button = QPushButton(text)
            button.setMinimumHeight(self._u(36))
            button.clicked.connect(lambda _checked=False, n=name: self._set_gun_view(n))
            view_row.addWidget(button)
        gun_view_grid.addLayout(view_row, 1, 0, 1, 4)
        gun_layout.addWidget(gun_view_box)

        gun_delta_box = QGroupBox("3. 焊丝物理末端 ↔ 示教器 LIVE TCP")
        gun_delta_grid = QGridLayout(gun_delta_box)
        gun_delta_grid.setContentsMargins(self._u(12), self._u(9), self._u(12), self._u(9))
        gun_delta_grid.setHorizontalSpacing(self._u(12))
        gun_delta_grid.setVerticalSpacing(self._u(5))
        self.gun_delta_labels = {}
        for row, (key, title) in enumerate((("dx", "ΔX"), ("dy", "ΔY"), ("dz", "ΔZ"), ("distance", "欧氏距离"), ("source", "LIVE TCP 来源"))):
            gun_delta_grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setStyleSheet("font-family:monospace;font-weight:bold;")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            gun_delta_grid.addWidget(value, row, 1, 1, 3)
            self.gun_delta_labels[key] = value
        gun_layout.addWidget(gun_delta_box)

        gun_save_box = QGroupBox("4. 保存草稿 / 确认安装标定")
        gun_save_layout = QHBoxLayout(gun_save_box)
        gun_save_layout.setContentsMargins(self._u(10), self._u(8), self._u(10), self._u(8))
        self.gun_load_button = QPushButton("加载标定")
        self.gun_load_button.clicked.connect(self._load_gun_mount_calibration)
        self.gun_draft_button = QPushButton("保存草稿")
        self.gun_draft_button.clicked.connect(self._save_gun_mount_draft)
        self.gun_confirm_button = QPushButton("确认安装标定")
        self.gun_confirm_button.setToolTip("需人工确认真实安装；确认后旧轨迹会标记 STALE，但不会自动应用 MATLAB 或发送运动")
        self.gun_confirm_button.clicked.connect(self._confirm_gun_mount_calibration)
        self.gun_reset_button = QPushButton("恢复 +90° / 0 mm")
        self.gun_reset_button.clicked.connect(self._reset_gun_mount_adjustment)
        for button in (self.gun_load_button, self.gun_draft_button, self.gun_confirm_button, self.gun_reset_button):
            button.setMinimumHeight(self._u(42))
            gun_save_layout.addWidget(button)
        gun_save_layout.addStretch(1)
        gun_layout.addWidget(gun_save_box)
        self.gun_mount_note = QLabel(
            f"独立记录：{self.gun_mount_calibration_file}。预览不改变 MATLAB 碰撞模型；确认只保存版本并使旧轨迹 STALE，"
            "后续规划脚本需显式加载同一文件并重新完成碰撞/TCP验证。冻结仿真 scene_geometry.json 与 ur5_tool_calibration.mat 不覆盖。"
        )
        self.gun_mount_note.setWordWrap(True)
        self.gun_mount_note.setStyleSheet("color:#6b7280;")
        gun_layout.addWidget(self.gun_mount_note)
        gun_layout.addStretch(1)
        tabs.addTab(gun_scroll, "焊枪安装调整")
        self.gun_mount_tab_index = tabs.indexOf(gun_scroll)
        self._load_gun_mount_calibration(silent=True)
        self._update_gun_mount_panel()

        # Tab 7: RealHome information and save action.
        home_tab = QWidget()
        home_layout = QVBoxLayout(home_tab)
        home_title = QLabel("RealHome（当前仅保存姿态，不自动移动机器人）")
        home_title.setStyleSheet("font-size:18px;font-weight:bold;")
        home_layout.addWidget(home_title)
        self.home_info = QLabel("尚未保存 RealHome")
        self.home_info.setStyleSheet("font-size:16px;font-family:monospace;")
        self.home_info.setWordWrap(True)
        home_layout.addWidget(self.home_info)
        self.home_button = QPushButton("保存当前姿态为 RealHome")
        self.home_button.setMinimumSize(240, 50)
        self.home_button.clicked.connect(self._save_home)
        home_layout.addWidget(self.home_button, 0, Qt.AlignLeft)
        home_layout.addStretch(1)
        tabs.addTab(home_tab, "RealHome")

        # Tab 6: full-size log view.
        log_tab = QWidget()
        log_layout = QVBoxLayout(log_tab)
        detail_box = QGroupBox("关键诊断状态")
        detail_layout = QVBoxLayout(detail_box)
        detail_layout.setContentsMargins(self._u(6), self._u(6), self._u(6), self._u(6))
        detail_layout.setSpacing(self._u(6))
        # Keep the run-log page readable in the narrow right pane.  The full
        # TCP and Jog cards were useful during bring-up but consume most of
        # the page and visually clip the status rows.  Their underlying state
        # and safety logic remain intact; the event log below is the operator
        # history, while this card shows only the compact key diagnostics.
        status_box.setTitle("")
        status_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        status_box.setMinimumHeight(self._u(230))
        status_box.setMaximumHeight(self._u(290))
        self.log_status_box = status_box
        detail_layout.addWidget(status_box)
        detail_note = QLabel("TCP 数值和 Jog 过程状态不在日志页长期展开；重要变化会记录在下方事件日志。")
        detail_note.setWordWrap(True)
        detail_note.setStyleSheet("color:#6b7280;padding:2px 4px;")
        detail_layout.addWidget(detail_note)
        detail_box.setMaximumHeight(self._u(330))
        self.log_detail_box = detail_box
        log_layout.addWidget(detail_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setStyleSheet("font-family:monospace;font-size:14px;")
        log_layout.addWidget(self.log_view, 1)
        tabs.addTab(log_tab, "运行日志")

        # Functional pages occupy the whole right pane below the compact
        # safety/status header.  Each heavy page already has its own
        # QScrollArea, so the global right panel does not need another scroll
        # layer that would hide the tab bar or STOP MOTION.
        right_layout.addWidget(tabs, 1)

        # The horizontal splitter is the complete application body: left is
        # the 3D scene, right is safety + tabs.  A 50:50 default is restored
        # only at construction; the user can drag it afterward.
        root.addWidget(top_splitter, 1)
        self.apply_ui_scale(self.ui_scale)
        QTimer.singleShot(0, lambda: self._on_main_tab_changed(tabs.currentIndex()))
        QTimer.singleShot(100, self._set_default_main_split)
        QTimer.singleShot(0, lambda: self._set_robot_view("robot_scene"))
        self._update_buttons()

    def _configure_display_tcp_offset(self) -> None:
        """Load the last manually approved flange->TCP offset for display FK.

        This is read-only and intentionally uses only the ``*_applied`` record;
        a draft calibration must not silently move the 3D TCP marker.  The
        frozen MATLAB baseline is a display fallback when no approved record
        exists.  Neither path writes a calibration file or changes planning.
        """
        offset = None
        source = ""
        try:
            for path, label in (
                (getattr(self, "tcp_calibration_active_file", None), "active pendant TCP version"),
                (self.tcp_calibration_applied_file, "approved real_tcp_calibration_applied.json"),
            ):
                if path is None or not path.is_file():
                    continue
                payload = json.loads(path.read_text(encoding="utf-8"))
                candidate = payload.get("position_flange_tcp_m")
                if payload.get("valid") and payload.get("applied_to_planning_model") and candidate is not None:
                    value = np.asarray(candidate, dtype=float).reshape(-1)
                    if value.size == 3 and np.all(np.isfinite(value)):
                        offset = value
                        source = label
                        break
        except Exception as exc:
            self._log(f"3D TCP 显示标定读取失败，回退冻结模型：{exc}")
        if offset is None:
            try:
                offset = self._existing_flange_tcp_offset()
                source = "frozen ur5_tool_calibration.mat baseline"
            except Exception:
                offset = None
        if offset is not None and hasattr(self.robot_view, "set_display_tcp_offset"):
            try:
                self.robot_view.set_display_tcp_offset(offset)
                self._log(f"3D TCP FK 显示偏移已固定：{source}；预演不会修改该变换")
            except Exception as exc:
                self._log(f"3D TCP FK 显示偏移无效：{exc}")
        gun_status = getattr(self.robot_view, "gun_display_status", None)
        if callable(gun_status):
            self._log(f"3D 焊枪显示：{gun_status()}；仅显示，不参与 HMI 碰撞/控制")

    def _set_default_main_split(self) -> None:
        """Set the initial 50:50 divider after the window gets real size.

        Calling this before the window is shown lets Qt's child minimum sizes
        dominate the requested sizes.  Delaying it one event-loop turn gives
        the user the intended default while still preserving later drags.
        """
        splitter = getattr(self, "main_splitter", None)
        if splitter is None or splitter.orientation() != Qt.Horizontal:
            return
        total = splitter.size().width()
        if total <= 0:
            return
        half = total // 2
        splitter.setSizes([half, total - half])

    def _on_main_tab_changed(self, index: int) -> None:
        """Keep the horizontal 3D/control split stable when changing tabs."""
        splitter = getattr(self, "main_splitter", None)
        if splitter is None:
            return
        # The splitter is now horizontal.  Do not resize it on tab changes;
        # this preserves the operator's manual divider position.
        if splitter.orientation() != Qt.Horizontal:
            return
        total = splitter.size().width()
        if total <= 0:
            return

    def _on_client_ready(self, client):
        """Receive the ROS-side TrajectoryClient so the gate can read `.busy`."""
        self.trajectory_client = client
        self._update_jog_status()
        self._update_buttons()

    def _on_motion(self, state_text: str, busy: bool, goal_label: str):
        self.motion_state_text = str(state_text)
        self.motion_goal_label = str(goal_label)
        if str(state_text) in (MotionState.IDLE.value, MotionState.FAULT_UNKNOWN.value) and not busy:
            self._jog_pending = False
        self._update_jog_status()
        self._update_buttons()
        self._update_trajectory_gate()

    def _on_state(self, state: RobotState):
        self.state = state
        self._update_online_start_display()
        if state.robot_description and not self._description_loaded:
            self.robot_view.set_urdf_text(state.robot_description)
            self._description_loaded = True
        if self._feedback_valid(state):
            # Always refresh the independent live snapshot.  The view decides
            # whether the mesh follows live_q or preview_q; preview never
            # replaces the real q/TCP state.
            setter = getattr(self.robot_view, "set_live_state", self.robot_view.set_state)
            setter(state.positions, state.tcp_position, state.tcp_frame_id, state.tcp_stamp)
        else:
            # Keep the reference/frozen robot mesh visible, but do not present
            # the last powered-on pose or TCP as live data.
            setter = getattr(self.robot_view, "set_live_state", self.robot_view.set_state)
            setter(None, None, "", 0.0)
        self._record_execution_sample()
        self._update_status_widgets()
        self._update_trajectory_gate()
        self._update_buttons()

    def _on_goal_active(self, active: bool):
        self.goal_active = active
        self.state.goal_active = active
        if active and self.execution_trajectory is not None and self.execution_started_monotonic is None:
            self.execution_started_monotonic = time.monotonic()
        if active and self.execution_trajectory is not None:
            self.execution_progress_label.setText("Trajectory Progress：0%    Current point：1 / %d    Elapsed：0.0 s    Remaining：—    Action State：ACTIVE" % self.execution_trajectory.point_count)
        self._update_jog_status()
        self._update_trajectory_gate()
        self._update_buttons()

    def _on_goal_done(self, ok: bool, message: str):
        self._log(("[PASS] " if ok else "[WARN] ") + message)
        if self.execution_trajectory is not None:
            if "cancel" in message.lower():
                outcome = "CANCELED"
            else:
                outcome = "SUCCESS" if ok else "FAILED"
            self._finalize_execution(outcome, message)
        self._on_goal_active(False)

    def _command_at_elapsed(self, elapsed: float):
        traj = self.execution_trajectory
        if traj is None or traj.time_from_start is None:
            return None, 0
        t = np.asarray(traj.time_from_start, dtype=float)
        elapsed = max(0.0, min(float(elapsed), float(t[-1])))
        index = int(np.searchsorted(t, elapsed, side="right") - 1)
        index = max(0, min(index, traj.point_count - 1))
        if index >= traj.point_count - 1 or t[index + 1] <= t[index]:
            return traj.positions[index].copy(), index
        alpha = (elapsed - t[index]) / (t[index + 1] - t[index])
        q = (1.0 - alpha) * traj.positions[index] + alpha * traj.positions[index + 1]
        return q, index

    def _record_execution_sample(self):
        if not self.goal_active or self.execution_trajectory is None or self.execution_started_monotonic is None:
            return
        if not self._feedback_valid(self.state):
            return
        now = time.monotonic()
        if self.execution_log and now - float(self.execution_log[-1]["monotonic"]) < 0.05:
            return
        elapsed = now - self.execution_started_monotonic
        q_command, index = self._command_at_elapsed(elapsed)
        if q_command is None:
            return
        q_actual = np.asarray(self.state.positions, dtype=float)
        error = q_actual - q_command
        duration = self.execution_trajectory.duration or 0.0
        progress = 100.0 if duration <= 0 else min(100.0, 100.0 * elapsed / duration)
        remaining = max(0.0, duration - elapsed)
        self.execution_progress_label.setText(
            f"Trajectory Progress：{progress:.1f}%    "
            f"Current point：{index + 1} / {self.execution_trajectory.point_count}    "
            f"Elapsed：{elapsed:.1f} s    Remaining：{remaining:.1f} s    Action State：ACTIVE"
        )
        self.execution_log.append({
            "monotonic": now,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "elapsed": float(elapsed),
            "command_point": int(index + 1),
            "q_command": q_command.tolist(),
            "q_actual": q_actual.tolist(),
            "joint_error": error.tolist(),
            "tcp_actual": None if self.state.tcp_position is None else list(self.state.tcp_position),
            "speed_scaling": self.state.speed_scaling,
            "safety_mode": self.state.safety_mode,
        })

    def _finalize_execution(self, outcome: str, message: str):
        traj = self.execution_trajectory
        if traj is None:
            return
        self._record_execution_sample()
        records = list(self.execution_log)
        errors = np.asarray([row["joint_error"] for row in records], dtype=float) if records else np.empty((0, 6))
        max_error = np.max(np.abs(errors), axis=0) if errors.size else np.full(6, np.nan)
        rms_error = np.sqrt(np.mean(errors ** 2, axis=0)) if errors.size else np.full(6, np.nan)
        final_error = errors[-1] if errors.size else np.full(6, np.nan)
        started = self.execution_started_monotonic
        elapsed = 0.0 if started is None else max(0.0, time.monotonic() - started)
        def finite_deg(values):
            return [None if not np.isfinite(value) else float(math.degrees(value)) for value in values]
        summary = {
            "outcome": outcome,
            "message": message,
            "trajectory_name": traj.metadata.get("trajectory_name", "未命名轨迹"),
            "source": traj.metadata.get("source", traj.source_path),
            "point_count": traj.point_count,
            "planned_duration_s": traj.duration,
            "wall_duration_s": elapsed,
            "command_speed_percent": int(self.speed_slider.value()),
            "max_joint_tracking_error_deg": finite_deg(max_error),
            "rms_joint_tracking_error_deg": finite_deg(rms_error),
            "final_joint_error_deg": finite_deg(final_error),
            "sample_count": len(records),
        }
        log_dir = ROOT / "logs" / "trajectory_runs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        json_path = log_dir / f"trajectory_run_{stamp}.json"
        csv_path = log_dir / f"trajectory_run_{stamp}.csv"
        json_path.write_text(json.dumps({"summary": summary, "samples": records}, ensure_ascii=False, indent=2), encoding="utf-8")
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            fields = ["timestamp", "elapsed", "command_point", "q_command", "q_actual", "joint_error", "tcp_actual", "speed_scaling", "safety_mode"]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in records:
                writer.writerow({key: json.dumps(row[key], ensure_ascii=False) if isinstance(row[key], (list, dict)) else row[key] for key in fields})
        self._log(f"Execution summary {outcome}: {json_path.name}, {csv_path.name}")
        self.execution_progress_label.setText(
            f"Trajectory Progress：{100 if outcome == 'SUCCESS' else '—'}%    "
            f"Current point：{traj.point_count} / {traj.point_count}    "
            f"Elapsed：{elapsed:.1f} s    Remaining：0.0 s    Action State：{outcome}"
        )
        self.execution_started_monotonic = None
        self.execution_trajectory = None
        self.execution_log = []

    @staticmethod
    def _feedback_valid(state: RobotState) -> bool:
        """Whether joint/TCP data can be presented as live robot feedback.

        ROS2 can keep publishing the last joint sample while the controller is
        powered off.  Robot mode is therefore required in addition to a fresh
        joint callback; modes 5/7 are the normal idle/running states.
        """
        fresh = len(state.positions) == 6 and time.monotonic() - state.joint_stamp < 1.0
        return fresh and state.robot_link_alive is True and state.robot_mode in (5, 7)

    def _update_status_widgets(self):
        fresh = len(self.state.positions) == 6 and time.monotonic() - self.state.joint_stamp < 1.0
        feedback_valid = self._feedback_valid(self.state)
        def status(key, text, level):
            row = self.status_rows.get(key) if hasattr(self, "status_rows") else None
            if row is not None:
                row.set_value(text, level, self.ui_scale)
            else:
                set_status_label(self.status_labels[key], text, level, self.ui_scale)

        link_ok = self.state.robot_link_alive is True
        status("driver", "Connected" if fresh and link_ok else "Disconnected", "ok" if fresh and link_ok else "bad")
        p = self.state.program_running
        program_live = p is True and self.state.robot_mode == 7
        program_text = "Running" if program_live else ("Stopped" if p is False or self.state.robot_mode in (3, 5) else "Unknown")
        status("program", program_text, "ok" if program_live else "bad")
        modes = {3: "POWER_OFF", 5: "IDLE", 7: "RUNNING"}
        mode = self.state.robot_mode
        status("mode", f"{mode} ({modes.get(mode, 'unknown')})", "ok" if mode in (5, 7) else "bad")
        sm = self.state.safety_mode
        safety_names = {
            1: "NORMAL",
            2: "REDUCED",
            3: "PROTECTIVE_STOP",
            4: "RECOVERY",
            5: "SAFEGUARD_STOP",
            6: "SYSTEM_EMERGENCY_STOP",
            7: "ROBOT_EMERGENCY_STOP",
            8: "VIOLATION",
            9: "FAULT",
            10: "VALIDATE_JOINT_ID",
            11: "UNDEFINED",
            12: "AUTO_SAFEGUARD_STOP",
            13: "THREE_POSITION_STOP",
        }
        safety_available = feedback_valid and sm == 1
        safety_name = "通信中断" if not link_ok else safety_names.get(sm, "UNKNOWN")
        status("safety", f"{sm} ({safety_name})", "ok" if safety_available else "warn")
        speed = self.state.speed_scaling
        status("speed", "—" if speed is None else f"{speed:.1f}%", "ok" if speed and speed > 0 else "warn")
        status("controller", "active" if self.state.controller_active else "inactive", "ok" if self.state.controller_active else "bad")
        status("joint", "updating" if feedback_valid else "stale", "ok" if feedback_valid else "bad")
        self._update_power_status()
        self._update_simple_status_widgets(link_ok, program_live, sm, safety_available)
        self._update_jog_status()
        for i, q in enumerate(self.state.positions[:6]):
            if feedback_valid:
                self.joint_table.item(i, 1).setText(f"{math.degrees(q):.3f}°")
                self.joint_table.item(i, 2).setText(f"{math.degrees(q):.3f}°")
                self.joint_table.item(i, 1).setToolTip(f"{q:.6f} rad")
                self.joint_table.item(i, 2).setToolTip(f"{q:.6f} rad")
            else:
                self.joint_table.item(i, 1).setText("—")
                self.joint_table.item(i, 2).setText("—")
                self.joint_table.item(i, 1).setToolTip("机器人反馈无效")
                self.joint_table.item(i, 2).setToolTip("机器人反馈无效")
        if self.state.tcp_position and feedback_valid:
            p = self.state.tcp_position
            for key, value in zip(("x", "y", "z"), p):
                self.tcp_labels[key].setText(f"{value * 1000:.2f}")
                self.tcp_labels[key].setToolTip(f"{value:.6f} m; frame={self.state.tcp_frame_id}")
        else:
            for key in ("x", "y", "z", "rx", "ry", "rz"):
                self.tcp_labels[key].setText("—")
                self.tcp_labels[key].setToolTip("机器人反馈无效")
        if self.state.tcp_rpy and feedback_valid:
            for key, value in zip(("rx", "ry", "rz"), self.state.tcp_rpy):
                self.tcp_labels[key].setText(f"{math.degrees(value):.3f}")
                self.tcp_labels[key].setToolTip(f"{value:.6f} rad")

    def _update_simple_status_widgets(self, link_ok: bool, program_live: bool, safety_mode, safety_available: bool):
        """Update the compact right-header summary without changing gates."""
        labels = getattr(self, "simple_state_labels", {})
        if not labels:
            return

        mode = self.state.robot_mode
        if not link_ok:
            robot_text, robot_level = "NOT READY", "bad"
        elif mode == 7 and program_live:
            robot_text, robot_level = "RUNNING", "ok"
        elif mode in (4, 5, 6):
            robot_text, robot_level = "READY", "ok"
        elif mode == 3:
            robot_text, robot_level = "NOT READY", "bad"
        else:
            robot_text, robot_level = "FAULT", "bad"

        if not link_ok:
            safety_text, safety_level = "UNKNOWN", "warn"
        elif safety_mode == 1:
            safety_text, safety_level = "NORMAL", "ok"
        elif safety_mode in (3, 5, 12, 13):
            safety_text, safety_level = "PROTECTIVE STOP", "bad"
        elif safety_mode in (6, 7, 8, 9):
            safety_text, safety_level = "FAULT", "bad"
        else:
            safety_text, safety_level = "UNKNOWN", "warn"

        if "robot" in labels:
            set_status_label(labels["robot"], robot_text, robot_level, self.ui_scale)
        if "safety" in labels:
            set_status_label(labels["safety"], safety_text, safety_level, self.ui_scale)

    def _update_power_status(self):
        """Show robot power state separately from ROS2 driver connectivity."""
        if not hasattr(self, "power_status_label"):
            return
        mode = self.state.robot_mode
        # RobotMode constants: POWER_ON=4, IDLE=5, BACKDRIVE=6, RUNNING=7.
        # These indicate that the robot is powered even when it is not yet
        # in the RUNNING state required by the motion safety gate.
        if self.state.robot_link_alive is not True:
            text, background, foreground = "电源：未知（通信中断）", "#fff2cc", "#8a5a00"
        elif mode in (4, 5, 6, 7):
            text, background, foreground = "电源：已上电", "#d8f5dc", "#146c2e"
        elif mode == 3:
            text, background, foreground = "电源：已断电", "#ffd9d9", "#9b1c1c"
        else:
            text, background, foreground = "电源：未知", "#fff2cc", "#8a5a00"
        self.power_status_label.setText(text)
        self.power_status_label.setStyleSheet(
            f"QLabel {{ background:{background}; color:{foreground}; "
            f"border:1px solid #999; border-radius:4px; "
            f"padding:{self._u(3)}px {self._u(6)}px; "
            f"font-size:{self._u(14)}px; font-weight:bold; }}"
        )

    def _update_jog_status(self):
        if not hasattr(self, "jog_status_labels"):
            return
        motion_text, motion_level = self._motion_state_display()
        set_status_label(self.jog_status_labels["motion"], motion_text, motion_level, self.ui_scale)
        set_status_label(
            self.jog_status_labels["goal"],
            self._current_goal_display(),
            "warn" if self._motion_busy() else "unknown",
            self.ui_scale,
        )
        set_status_label(
            self.jog_status_labels["action"],
            "执行中" if self.goal_active else "可用",
            "warn" if self.goal_active else "ok",
            self.ui_scale,
        )
        if self._feedback_valid(self.state):
            self.jog_status_labels["joint"].setText("当前反馈")
            self.jog_status_labels["target"].setText("单关节目标")
        else:
            self.jog_status_labels["joint"].setText("反馈无效")
            self.jog_status_labels["target"].setText("—")
        speed = int(self.speed_slider.value()) if hasattr(self, "speed_slider") else 0
        self.jog_status_labels["command_speed"].setText(f"{speed}%")
        scaling = self.state.speed_scaling
        self.jog_status_labels["robot_speed"].setText("—" if scaling is None else f"{scaling:.1f}%")
        ready = self._motion_ready()
        set_status_label(
            self.jog_status_labels["start_check"],
            "READY" if ready else "BLOCKED",
            "ok" if ready else "warn",
            self.ui_scale,
        )
        self.jog_status_labels["control_source"].setText("PC / ROS2")

    # ------------------------------------------------------------------
    # Motion busy gate
    #
    # The GUI reads `TrajectoryClient.busy` directly.  `goal_active` alone is
    # not sufficient: it is only raised once the goal has been accepted, so it
    # cannot see the SENDING window, and it drops as soon as a cancel is
    # requested while the goal is still pending (CANCELING).
    # ------------------------------------------------------------------
    def _motion_client_attr(self, name: str, default):
        client = self.trajectory_client
        if client is None:
            return default
        try:
            return getattr(client, name)
        except Exception:
            return default

    def _motion_busy(self) -> bool:
        """Authoritative motion gate: `trajectory_client.busy`."""
        client = self.trajectory_client
        if client is None:
            return False
        try:
            return bool(client.busy)
        except Exception:
            # Fail safe: an unreadable busy flag must not unlock motion.
            return True

    def _motion_state_display(self) -> tuple[str, str]:
        state = self._motion_client_attr("motion_state", None)
        text = getattr(state, "value", None) or self.motion_state_text or MotionState.IDLE.value
        level = {
            MotionState.IDLE.value: "ok",
            MotionState.SENDING.value: "warn",
            MotionState.EXECUTING.value: "warn",
            MotionState.CANCELING.value: "warn",
            MotionState.FAULT_UNKNOWN.value: "bad",
        }.get(str(text), "unknown")
        return str(text), level

    def _current_goal_display(self) -> str:
        label = self._motion_client_attr("current_goal_label", "") or self.motion_goal_label
        return str(label) if label else "—"

    def _motion_ready(self):
        return (
            check_motion_ready(self.state).ok
            and not self.goal_active
            and not self._motion_busy()
        )

    def _update_buttons(self):
        parameter_busy = getattr(self, "robot_tcp_busy", False)
        enabled = self._motion_ready() and not parameter_busy
        for jog in self.jog_widgets:
            jog.set_enabled(enabled)
        # `trajectory_client.busy` blocks new motion and new work: it covers
        # SENDING, EXECUTING and CANCELING, i.e. every moment in which a goal
        # exists but its outcome is not yet known.
        motion_blocked = self._motion_busy()
        blocked = motion_blocked or parameter_busy
        if hasattr(self, "load_button"):
            self.load_button.setEnabled(not blocked and not self.planning_busy)
        if hasattr(self, "validate_button"):
            self.validate_button.setEnabled(not blocked and not self.planning_busy and self.loaded_trajectory is not None)
        self.execute_button.setEnabled(not blocked and self._final_execution_gate_ok())
        if hasattr(self, "online_start_button"):
            self.online_start_button.setEnabled(not blocked and not self.planning_busy)
        if hasattr(self, "online_cancel_button"):
            self.online_cancel_button.setEnabled(self.planning_busy)
        # STOP MOTION is never disabled by the busy gate.
        self.stop_button.setEnabled(motion_blocked or self.goal_active)
        self.home_button.setEnabled(self._feedback_valid(self.state) and not blocked)
        if hasattr(self, "jog_status_labels"):
            self._update_jog_status()

    def _set_step_size(self, value: float):
        self.step_deg = float(value)

    def _on_speed_changed(self, value: int):
        self.speed_value_label.setText(f"{int(value)} %")
        self._update_jog_status()

    def _ui_scale_changed(self, text: str):
        try:
            scale = int(text.rstrip("%")) / 100.0
        except ValueError:
            return
        self.config.setdefault("ui", {})["scale"] = scale
        CONFIG.write_text(yaml.safe_dump(self.config, sort_keys=False, allow_unicode=True))
        self.apply_ui_scale(scale)
        self._log(f"界面缩放已应用并保存为 {text}")

    def _jog_clicked(self, index: int, direction: float):
        if getattr(self, "robot_tcp_busy", False):
            self._log("Jog blocked: TCP 参数读取/写入中")
            return
        if self._jog_pending:
            self._log("Jog ignored: previous one-step command is still pending")
            return
        if not self._motion_ready():
            self._log("Jog disabled: safety gate is not satisfied")
            return
        step = float(self.step_deg)
        speed = float(self.speed_slider.value())
        self._jog_pending = True
        # Disable all six rows immediately, before the queued ROS worker
        # callback runs.  They are re-enabled only after a known terminal
        # action result; STOP MOTION remains available.
        for jog in self.jog_widgets:
            jog.set_enabled(False)
        self.worker.request_jog.emit(index, direction * step / 0.5, speed)

    def _stop_motion(self):
        self.worker.request_stop.emit()
        self._log("STOP MOTION: cancel requested; this is not a hardware emergency stop")

    @staticmethod
    def _format_joint_row(row) -> str:
        if row is None:
            return "—"
        return "[" + ", ".join(f"{math.degrees(float(v)):.2f}°" for v in row) + "]"

    def _set_trajectory_summary(self, report: ValidationReport | None = None):
        labels = getattr(self, "trajectory_info_labels", {})
        traj = self.loaded_trajectory
        if not traj:
            return
        labels["name"].setText(str(traj.metadata.get("trajectory_name", "未命名轨迹")))
        labels["source"].setText(str(traj.metadata.get("source", traj.source_path or "unknown")))
        labels["coordinate"].setText(str(traj.metadata.get("coordinate_space", "unknown")))
        labels["collision"].setText(self._collision_status_text(traj))
        labels["points"].setText(str(traj.point_count))
        labels["duration"].setText("—" if traj.duration is None else f"{traj.duration:.3f} s")
        labels["start"].setText(self._format_joint_row(traj.positions[0]))
        labels["end"].setText(self._format_joint_row(traj.positions[-1]))
        if report is None:
            labels["travel"].setText("—")
            labels["step"].setText("—")
            labels["velocity"].setText("—")
            labels["acceleration"].setText("—")
            labels["start_error"].setText("—")
            labels["validation"].setText("NOT VALIDATED")
            return
        labels["travel"].setText(self._format_joint_row(report.max_joint_travel_rad))
        labels["step"].setText(f"{math.degrees(report.max_adjacent_step_rad):.3f}°")
        labels["velocity"].setText(
            "—" if report.estimated_max_velocity is None else f"{math.degrees(report.estimated_max_velocity):.3f} °/s"
        )
        labels["acceleration"].setText(
            "—" if report.estimated_max_acceleration is None else f"{math.degrees(report.estimated_max_acceleration):.3f} °/s²"
        )
        if report.start_error_rad is None:
            labels["start_error"].setText("等待实时反馈")
        else:
            labels["start_error"].setText(self._format_joint_row(report.start_error_rad))
        labels["validation"].setText("PASS" if report.ok else "FAIL: " + "; ".join(report.issues[:2]))

    def _final_gate_reasons(self) -> list[str]:
        reasons = []
        # Checked before the "no trajectory" early return so the gate label
        # always explains a busy motion state.
        if self.goal_active:
            reasons.append("已有 action 正在执行")
        elif self._motion_busy():
            reasons.append("motion busy：上一个 goal 尚未返回结果")
        traj = self.loaded_trajectory
        if traj is None:
            reasons.append("未加载轨迹")
            return reasons
        # Re-check on every gate evaluation so an externally replaced
        # calibration file cannot leave an already loaded path executable.
        if not self.trajectory_stale_reason:
            binding_reasons = self._trajectory_binding_reasons(traj)
            if binding_reasons:
                self.trajectory_stale_reason = "; ".join(binding_reasons)
        if self.trajectory_stale_reason:
            reasons.append("轨迹 STALE：" + self.trajectory_stale_reason)
        if self.trajectory_validation is None:
            reasons.append("尚未点击验证轨迹")
        elif not self.trajectory_validation.ok:
            reasons.extend(self.trajectory_validation.issues[:8])
        collision_required, collision_verified, collision_pass = self._collision_gate_state(traj)
        if collision_required:
            if not collision_verified:
                reasons.append("正式 MATLAB/RRT 轨迹缺少碰撞验证元数据")
            elif not collision_pass:
                reasons.append("正式 MATLAB/RRT 轨迹碰撞验证未通过")
        current_report = validate_trajectory(
            traj,
            q_actual=self.state.positions if self._feedback_valid(self.state) else None,
        )
        if not self._feedback_valid(self.state):
            reasons.append("joint state 无新鲜有效反馈")
        if self.state.program_running is not True:
            reasons.append("robot program 未运行")
        if self.state.robot_mode != 7:
            reasons.append(f"robot_mode={self.state.robot_mode}，不是 RUNNING(7)")
        if self.state.safety_mode != 1:
            reasons.append("safety_mode 不是 NORMAL(1)")
        if self.state.speed_scaling is None or self.state.speed_scaling <= 0:
            reasons.append("speed_scaling 不大于 0")
        if not self.state.controller_active:
            reasons.append("scaled controller 未 active")
        if not self.state.action_server_ready:
            reasons.append("FollowJointTrajectory action server 未就绪")
        if current_report.start_error_rad is None:
            reasons.append("无法计算起点误差")
        elif np.max(np.abs(current_report.start_error_rad)) > self.start_tolerance_rad:
            reasons.append(f"最大起点误差超过 {math.degrees(self.start_tolerance_rad):.1f}°")
        return reasons

    @staticmethod
    def _file_sha256(path: Path) -> str | None:
        if not path.is_file():
            return None
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    def _current_calibration_binding(self) -> dict:
        """Return the read-only calibration versions used by planning/display.

        The binding is metadata only.  It never changes the MATLAB scene or
        controller state.  A real workpiece frame is considered current only
        when the operator has explicitly selected REAL CALIBRATION; otherwise
        the immutable scene_geometry.json is the planning reference.
        """
        real_workpiece = (
            hasattr(self, "calibration_source_combo")
            and self.calibration_source_combo.currentText() == "REAL CALIBRATION"
            and self.calibration_file.is_file()
        )
        workpiece_path = self.calibration_file if real_workpiece else ROOT.parent / "scene_geometry.json"
        planning_scene = "REAL_CALIBRATION" if real_workpiece else "FROZEN_SIMULATION"
        applied_tcp_path = self.tcp_calibration_applied_file
        active_tcp_path = getattr(self, "tcp_calibration_active_file", None)
        frozen_tcp_path = ROOT.parent / "ur5_tool_calibration.mat"
        # The planner binding follows the selected planning scene.  The
        # applied-file fingerprint is also retained below so changing a real
        # TCP cannot silently leave an old frozen path executable.
        tcp_path = frozen_tcp_path
        tcp_source = "ur5_tool_calibration.mat (frozen)"
        try:
            candidates = []
            # The active pendant version is a real-planning input only.  A
            # frozen SIMULATION scene continues to bind to the frozen MAT
            # calibration and cannot be silently changed by this workflow.
            if real_workpiece and active_tcp_path is not None:
                candidates.append((active_tcp_path, "real_tcp_calibration_active.json"))
            if real_workpiece:
                candidates.append((applied_tcp_path, "real_tcp_calibration_applied.json"))
            for candidate_path, candidate_source in candidates:
                payload = json.loads(candidate_path.read_text(encoding="utf-8")) if candidate_path.is_file() else {}
                if (payload.get("valid") and payload.get("applied_to_planning_model")
                        and payload.get("active_for_planning", True)):
                    tcp_path = candidate_path
                    tcp_source = candidate_source
                    break
        except Exception:
            pass
        gun_mount_hash = None
        if self.gun_mount_calibration_file.is_file():
            try:
                gun_payload = json.loads(self.gun_mount_calibration_file.read_text(encoding="utf-8"))
                if str(gun_payload.get("status", "")).upper() == "CONFIRMED" and gun_payload.get("valid"):
                    gun_mount_hash = self._file_sha256(self.gun_mount_calibration_file)
            except Exception:
                gun_mount_hash = None
        return {
            "urdf_sha256": self._file_sha256(ROOT.parent / "real_robot_description_calibrated.urdf"),
            "tcp_calibration_sha256": self._file_sha256(tcp_path),
            "tcp_calibration_source": tcp_source,
            "tcp_calibration_active_sha256": self._file_sha256(active_tcp_path) if active_tcp_path else None,
            "tcp_applied_sha256": self._file_sha256(applied_tcp_path),
            "tcp_frozen_sha256": self._file_sha256(frozen_tcp_path),
            # A confirmed gun-installation record is an explicit planning
            # dependency.  A draft file is still preview-only and is not
            # silently treated as a planning geometry version.
            "gun_mount_sha256": gun_mount_hash,
            "workpiece_sha256": self._file_sha256(workpiece_path),
            "workpiece_source": "REAL CALIBRATION" if real_workpiece else "SIMULATION",
            "workpiece_file": str(workpiece_path),
            "planning_scene": planning_scene,
        }

    def _trajectory_binding_reasons(self, trajectory: TrajectoryData) -> list[str]:
        """Check calibration-version metadata without changing the trajectory."""
        meta = trajectory.metadata or {}
        binding = meta.get("calibration_binding")
        if not isinstance(binding, dict):
            return ["缺少 TCP/工件/URDF 标定版本绑定（旧轨迹）"]
        current = self._current_calibration_binding()
        reasons = []
        for key, label in (
            ("urdf_sha256", "URDF"),
            ("tcp_calibration_sha256", "TCP 标定"),
            ("tcp_applied_sha256", "已应用 TCP 记录"),
            ("tcp_frozen_sha256", "冻结 TCP 基准"),
            ("gun_mount_sha256", "焊枪安装标定"),
            ("workpiece_sha256", "工件标定"),
            ("workpiece_source", "工件位姿源"),
            ("planning_scene", "规划场景"),
        ):
            if binding.get(key) != current.get(key):
                reasons.append(f"{label}版本不一致")
        return reasons

    def _mark_trajectory_stale(self, reason: str) -> None:
        if self.loaded_trajectory is None:
            return
        self.trajectory_stale_reason = str(reason)
        if hasattr(self, "path_label"):
            self.path_label.setText(f"{Path(self.loaded_path_name).name}: STALE（{reason}）")
        if hasattr(self, "preview_status_label"):
            self.preview_status_label.setText("STALE：仅可预览，必须重新规划/验证")
        self._log("Trajectory marked STALE: " + str(reason))

    def _final_execution_gate_ok(self) -> bool:
        return self.loaded_trajectory is not None and self.trajectory_validation is not None and not self._final_gate_reasons()

    @staticmethod
    def _scalar_bool(value, default=False) -> bool:
        """Convert MATLAB/scipy scalar metadata without accepting arrays."""
        if value is None:
            return bool(default)
        try:
            arr = np.asarray(value)
            if arr.size != 1:
                return bool(default)
            return bool(arr.reshape(-1)[0])
        except Exception:
            return bool(default)

    @classmethod
    def _collision_gate_state(cls, traj: TrajectoryData):
        meta = traj.metadata or {}
        source = str(meta.get("source", "")).lower()
        kind = str(meta.get("trajectory_type", "")).lower()
        # Commissioning/test paths are deliberately allowed to use the
        # existing structure/start/safety gates without claiming collision
        # validation.  Formal MATLAB/RRT exports carry explicit fields.
        commissioning = any(token in (source + " " + kind) for token in ("test", "commissioning"))
        has_collision_fields = "collision_verified" in meta or "collision_check_pass" in meta
        formal = has_collision_fields or any(token in (source + " " + kind) for token in ("matlab_rrt", "rrt_connect", "rrt*", "rrt"))
        required = bool(formal and not commissioning)
        verified = cls._scalar_bool(meta.get("collision_verified"), False)
        passed = cls._scalar_bool(meta.get("collision_check_pass"), False)
        return required, verified, passed

    @classmethod
    def _collision_status_text(cls, traj: TrajectoryData) -> str:
        required, verified, passed = cls._collision_gate_state(traj)
        if verified and passed:
            return "PASS"
        if verified and not passed:
            return "FAIL"
        if required:
            return "REQUIRED / NOT VERIFIED"
        return "NOT PROVIDED (preview/test)"

    def _update_trajectory_gate(self):
        if not hasattr(self, "final_gate_label"):
            return
        if self.loaded_trajectory is None:
            self.final_gate_label.setText("FINAL EXECUTION CHECK：BLOCKED（未加载轨迹）")
            self.final_gate_label.setStyleSheet("background:#fff2cc;color:#8a5a00;font-weight:bold;padding:8px;")
            self.final_gate_details_label.setText("未加载轨迹")
            return
        current_report = validate_trajectory(
            self.loaded_trajectory,
            q_actual=self.state.positions if self._feedback_valid(self.state) else None,
        )
        self._set_trajectory_summary(current_report if self.trajectory_validation is not None else None)
        reasons = self._final_gate_reasons()
        if not reasons:
            collision_required, collision_verified, collision_pass = self._collision_gate_state(self.loaded_trajectory)
            if collision_verified and collision_pass:
                collision_text = "Collision PASS ✓"
            elif collision_required:
                collision_text = "Collision PASS ✓"  # unreachable if the gate logic is consistent
            else:
                collision_text = "Collision gate not required (preview/test)"
            self.final_gate_label.setText("FINAL EXECUTION CHECK：PASS（全部绿色）")
            self.final_gate_label.setStyleSheet("background:#d8f5dc;color:#146c2e;font-weight:bold;padding:8px;")
            self.final_gate_details_label.setText(
                "ROS connected ✓ | Program running ✓ | robot_mode RUNNING ✓ | "
                "Safety NORMAL ✓ | Speed scaling > 0 ✓ | Controller active ✓ | "
                f"Joint fresh ✓ | Validation PASS ✓ | {collision_text} | "
                "Start check ≤ 1° ✓ | Action idle ✓"
            )
        else:
            self.final_gate_label.setText("FINAL EXECUTION CHECK：BLOCKED")
            self.final_gate_label.setStyleSheet("background:#fff2cc;color:#8a5a00;font-weight:bold;padding:8px;")
            self.final_gate_label.setToolTip("\n".join(reasons[:12]))
            self.final_gate_details_label.setText("未通过：" + "；".join(reasons[:8]))
        if hasattr(self, "execute_button"):
            self.execute_button.setEnabled(
                not self._motion_busy() and not reasons and self.trajectory_validation is not None
            )

    def _online_goal_report_path(self) -> Path:
        configured = self.config.get("online_planning", {}).get("v1_report_file", "")
        if configured:
            return Path(str(configured)).expanduser()
        return ROOT.parent / "轨迹文件" / "realhome_to_preweld_v1_report.json"

    def _online_task(self) -> str:
        if not hasattr(self, "online_task_combo"):
            return "Weld_Seam"
        return str(self.online_task_combo.currentData() or "Weld_Seam")

    def _weld_anchor_path(self) -> Path:
        configured = str(self.config.get("online_planning", {}).get("anchor_file", "")).strip()
        if configured:
            return Path(configured).expanduser()
        return ROOT.parent / "规划文件" / "weld_anchor_points.json"

    def _load_weld_anchors(self) -> dict:
        path = self._weld_anchor_path()
        payload = json.loads(path.read_text(encoding="utf-8"))

        def as_vec(value, label):
            q = np.asarray(value, dtype=float).reshape(-1)
            if q.size != 6 or not np.isfinite(q).all():
                raise ValueError(f"{label} is not a finite six-joint vector")
            return q

        return {
            "path": path,
            "qHome": as_vec(payload["qHome"], "qHome"),
            "qPreWeld": as_vec(payload["qPreWeld"], "qPreWeld"),
            "qPostWeld": as_vec(payload["qPostWeld"], "qPostWeld"),
            "qWeldStart": as_vec(payload.get("branchReferenceQ", payload["qWeldStart"]), "branchReferenceQ"),
        }

    def _task_endpoints(self, task: str):
        """Return (q_start, q_goal, source_text) for the selected task.

        Weld/Full/PostWeld endpoints come from weld_anchor_points.json, which
        holds the validated PreWeld/Weld/PostWeld anchors.  qHome there is the
        measured physical robot home, never the CoppeliaSim qOffset pose.
        """
        anchors = self._load_weld_anchors()
        source_name = anchors["path"].name
        if task == "RealHome_to_PreWeld":
            selected = self.online_start_combo.currentData()
            if selected == "real_home":
                q_start = self._saved_real_home()
                if q_start is None:
                    raise ValueError("Saved RealHome 不存在或无效")
                start_text = "Saved RealHome"
            else:
                if not self._feedback_valid(self.state):
                    raise ValueError("Current Robot 没有新鲜有效的 /joint_states")
                q_start = np.asarray(self.state.positions, dtype=float).reshape(-1)
                start_text = "Current Robot snapshot"
            return q_start, anchors["qPreWeld"], f"{start_text} -> {source_name}:qPreWeld"
        if task == "PostWeld_to_RealHome":
            return anchors["qPostWeld"], anchors["qHome"], f"{source_name}:qPostWeld -> qHome"
        if task == "Weld_Seam":
            return anchors["qPreWeld"], anchors["qPostWeld"], f"{source_name}:qPreWeld -> qPostWeld"
        if task == "Full_Welding_Task":
            selected = self.online_start_combo.currentData()
            if selected == "real_home":
                q_start = self._saved_real_home()
                if q_start is None:
                    raise ValueError("Saved RealHome 不存在或无效")
                start_text = "Saved RealHome"
            else:
                if not self._feedback_valid(self.state):
                    raise ValueError("Current Robot 没有新鲜有效的 /joint_states")
                q_start = np.asarray(self.state.positions, dtype=float).reshape(-1)
                start_text = "Current Robot snapshot"
            return q_start, anchors["qHome"], f"{start_text} -> {source_name}:qHome"
        raise ValueError(f"unsupported task {task!r}")

    def _refresh_online_goal_display(self):
        if not hasattr(self, "online_goal_source_label"):
            return
        self.online_goal_rad = None
        task = self._online_task()
        task_labels = {
            "RealHome_to_PreWeld": "PreWeld",
            "Weld_Seam": "PreWeld → PostWeld",
            "PostWeld_to_RealHome": "RealHome",
            "Full_Welding_Task": "RealHome（经焊缝）",
        }
        self.online_goal_label.setText(task_labels.get(task, task))

        # The goal is an anchor-file value.  Do not make the goal display
        # depend on a fresh Current Robot snapshot: a stale/disconnected ROS
        # state should block planning, but it must not turn a valid qPreWeld
        # target into the misleading label “不可用”.
        try:
            anchors = self._load_weld_anchors()
            source_name = anchors["path"].name
            goal_map = {
                "RealHome_to_PreWeld": (anchors["qPreWeld"], f"{source_name}:qPreWeld"),
                "Weld_Seam": (anchors["qPostWeld"], f"{source_name}:qPreWeld → qPostWeld"),
                "PostWeld_to_RealHome": (anchors["qHome"], f"{source_name}:qHome"),
                "Full_Welding_Task": (anchors["qHome"], f"{source_name}:qHome（Full Task终点）"),
            }
            q_goal, text = goal_map[task]
            self.online_goal_rad = q_goal
            self.online_goal_source_label.setText(text)
            self.online_task_status_label.setText({
                "Weld_Seam": "READY · 初始焊点 16",
                "Full_Welding_Task": "READY · Approach V1 / Weld A-ARC-IK+ / Exit V1",
            }.get(task, "READY"))
            self.online_task_status_label.setStyleSheet("color:#146c2e;font-weight:bold;")
        except Exception as exc:
            self.online_goal_source_label.setText(f"目标数据不可用：{exc}")
            self.online_task_status_label.setText("BLOCKED · 目标数据不可用")
            self.online_task_status_label.setStyleSheet("color:#9b1c1c;font-weight:bold;")

        # Keep only task-relevant controls visible.  The controls remain
        # instantiated so existing planning code and signal connections are
        # untouched.
        free_space_task = task in ("RealHome_to_PreWeld", "PostWeld_to_RealHome", "Full_Welding_Task")
        weld_task = task in ("Weld_Seam", "Full_Welding_Task")
        for key in ("free_algorithm_label", "free_algorithm"):
            self.online_field_widgets[key].setVisible(free_space_task)
        for key in ("weld_algorithm_label", "weld_algorithm"):
            self.online_field_widgets[key].setVisible(weld_task)
        start_selectable = task in ("RealHome_to_PreWeld", "Full_Welding_Task")
        self.online_start_combo.setEnabled(start_selectable)
        self.online_start_combo.setToolTip(
            "选择 Current Robot 或 Saved RealHome。" if start_selectable
            else "该任务使用固定的焊缝锚点起点。"
        )

    def _set_online_stage(self, stage: str) -> None:
        if hasattr(self, "online_stage_label"):
            self.online_stage_label.setText(stage)
            colour = "#146c2e" if stage in ("SUCCESS",) else "#9b1c1c" if stage == "FAILED" else "#8a6d00"
            self.online_stage_label.setStyleSheet(f"font-weight:bold;color:{colour};")

    def _saved_real_home(self):
        """Read the HMI-owned RealHome YAML instead of using a hardcoded q."""
        try:
            config = yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {}
            q = np.asarray(config.get("real_home", []), dtype=float).reshape(-1)
            if q.size != 6 or not np.isfinite(q).all():
                return None
            return q
        except Exception:
            return None

    def _update_online_start_display(self):
        if not hasattr(self, "online_start_combo"):
            return
        source = self.online_start_combo.currentData()
        if source == "real_home":
            q = self._saved_real_home()
            if q is None:
                self.online_start_snapshot_label.setText("Saved RealHome 不存在或无效")
            else:
                self.online_start_snapshot_label.setText(
                    "Source: Saved RealHome  |  Freshness: CONFIG\n"
                    "q = " + self._format_joint_row(q)
                )
            return
        if self._feedback_valid(self.state):
            self.online_start_snapshot_label.setText(
                "Source: Current Robot  |  Freshness: LIVE\n"
                "q = " + self._format_joint_row(self.state.positions)
            )
        else:
            self.online_start_snapshot_label.setText(
                "Source: Current Robot  |  Freshness: WAITING\n"
                "等待新鲜 /joint_states"
            )

    @staticmethod
    def _matlab_quote(path: Path) -> str:
        return "'" + str(path).replace("'", "''") + "'"

    def _online_log_process_output(self, stream="stdout"):
        process = self.planning_process
        if process is None:
            return
        if stream == "stderr":
            raw = bytes(process.readAllStandardError()).decode("utf-8", errors="replace")
        else:
            raw = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in raw.splitlines():
            if not line.strip():
                continue
            self.online_output_view.appendPlainText(line)
            self._log("[MATLAB online] " + line)
            self._parse_online_stage_line(line)
        scrollbar = self.online_output_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    _STAGE_TEXT = {
        "PREPARING": "PREPARING（加载场景与标定）",
        "APPROACH": "PLANNING APPROACH（自由空间接近）",
        "WELD": "PLANNING WELD（A-ARC-IK+ 焊缝）",
        "EXIT": "PLANNING EXIT（自由空间退出）",
        "VALIDATING": "VALIDATING（节点/边碰撞 + 连续性 + TCP）",
        "EXPORTING": "EXPORTING（时间参数化 + 导出 MAT/JSON）",
    }

    def _parse_online_stage_line(self, line: str) -> None:
        """Mirror the MATLAB stage markers into the GUI stage field."""
        text = line.strip()
        if text.startswith("HMI_WELD_STAGE stage="):
            stage = text.split("stage=", 1)[1].strip()
            label = self._STAGE_TEXT.get(stage, stage)
            self._set_online_stage(label)
            self.online_status_label.setText(f"规划中：{label}（GUI 仍可操作）")
            return
        if text.startswith("HMI_WELD_PLAN_START"):
            self._set_online_stage("STARTING MATLAB")
            return
        if text.startswith("HMI_WELD_SEAM_FORWARD_FAIL"):
            self._set_online_stage("WELD FORWARD FAIL → REVERSE")
            self.online_status_label.setText("Forward 焊缝失败，正在尝试 Reverse（A-ARC-IK+）")
            return
        if text.startswith("HMI_WELD_PLAN_DONE"):
            return

    def _online_process_error(self, error):
        self._online_log_process_output("stderr")
        self._log(f"Online MATLAB process error: {error}")
        if error == QProcess.FailedToStart:
            self.planning_busy = False
            self.online_status_label.setText("规划失败：无法启动 MATLAB")
            process = self.planning_process
            self.planning_process = None
            if process is not None:
                process.deleteLater()
            self._update_buttons()

    def _cancel_online_planning(self):
        """Terminate only the MATLAB planning process.

        This never calls STOP MOTION, never cancels a FollowJointTrajectory
        goal and never publishes any ROS2 message.  A planning process cannot
        move the robot because planning is an offline MATLAB computation.
        """
        if self.planning_process is None:
            return
        self._set_online_stage("CANCELLED")
        self.planning_process.terminate()
        self.online_status_label.setText("已请求取消 MATLAB 规划进程（未发送任何机器人指令）")
        self._log("Online planning cancel requested: only the MATLAB process was signalled; "
                  "no STOP MOTION and no robot command was issued")

    def _reset_online_result(self):
        for label in getattr(self, "online_result_labels", {}).values():
            label.setText("—")

    def _show_online_result(self, report_path: Path):
        self._reset_online_result()
        payload = json.loads(report_path.read_text(encoding="utf-8"))
        labels = self.online_result_labels

        def num(value, fmt="{:.3f}", suffix=""):
            if value is None:
                return "—"
            try:
                f = float(value)
            except (TypeError, ValueError):
                return str(value)
            if not math.isfinite(f):
                return "—"
            return fmt.format(f) + suffix

        def flag(value):
            return "PASS" if bool(value) else "FAIL"

        labels["task"].setText(str(payload.get("task", "—")))
        labels["algorithm"].setText(str(payload.get("algorithm", "—")))
        labels["seed"].setText(
            f"{int(float(payload['seed']))}" if str(payload.get("seed", "")).replace(".", "", 1).isdigit()
            else str(payload.get("seed", "—")))
        labels["planning_time"].setText(num(payload.get("planning_time", payload.get("wall_time_sec"))) + " s")
        labels["path_points"].setText(str(
            payload.get("execution_point_count", payload.get("raw_point_count", "—"))))
        labels["duration"].setText(num(payload.get("duration", payload.get("duration_sec"))) + " s")

        collision_pass = bool(payload.get("collision_check_pass", False))
        continuity_pass = bool(payload.get("continuity_check_pass", False))
        labels["collision"].setText(flag(collision_pass))
        labels["collision"].setStyleSheet(
            "color:#146c2e;font-weight:bold;" if collision_pass else "color:#9b1c1c;font-weight:bold;")
        labels["continuity"].setText(flag(continuity_pass))
        labels["continuity"].setStyleSheet(
            "color:#146c2e;font-weight:bold;" if continuity_pass else "color:#9b1c1c;font-weight:bold;")
        labels["tcp_error"].setText(num(payload.get("max_tcp_error_mm"), "{:.6f}", " mm"))
        labels["max_joint_jump"].setText(num(payload.get("max_joint_jump_deg"), "{:.3f}", " deg"))

        # Weld-seam specific fields.
        seam_points = payload.get("seam_point_count")
        seam_exec = payload.get("seam_execution_point_count")
        direction = str(payload.get("seam_direction", "") or "—")
        cascade = str(payload.get("seam_cascade", "") or "")
        is_weld_task = str(payload.get("task", "")) in ("Weld_Seam", "Full_Welding_Task")
        if is_weld_task:
            labels["seam_initial_points"].setText("16")
            labels["seam_final_points"].setText(str(seam_points if seam_points is not None else "—"))
            labels["seam_exec_points"].setText(str(seam_exec if seam_exec is not None else "—"))
            labels["seam_direction"].setText(f"{direction} {cascade}".strip())
            planner_wall = payload.get("seam_planner_time_sec")
            planner_alg = payload.get("seam_algorithm_time_sec")
            labels["seam_result"].setText(
                f"{str(payload.get('seam_planner', 'A-ARC-IK+'))[:44]}\n"
                f"wall {num(planner_wall)} s / alg {num(planner_alg)} s"
            )
            labels["seam_inserted_points"].setText(
                str(int(seam_exec) - int(seam_points)) if seam_exec is not None and seam_points is not None else "—")
        else:
            for key in ("seam_initial_points", "seam_final_points", "seam_exec_points",
                        "seam_inserted_points", "seam_direction", "seam_result"):
                labels[key].setText("—（无焊缝段）")

        counts = payload.get("stage_point_counts") or {}
        if str(payload.get("task", "")) == "Full_Welding_Task":
            for key, source in (("stage_approach_points", "approach"),
                                ("stage_weld_points", "weld"),
                                ("stage_exit_points", "exit"),
                                ("stage_total_points", "total")):
                value = counts.get(source)
                labels[key].setText(str(int(value)) if isinstance(value, (int, float)) else "—")
        else:
            for key in ("stage_approach_points", "stage_weld_points",
                        "stage_exit_points", "stage_total_points"):
                labels[key].setText("—（仅 Full Task）")
        return payload

    def _archive_online_result(self):
        """Copy an online-planning result to the user-facing planning folder.

        ``planning_runs/<request_id>`` remains the immutable/auditable run
        directory.  The additional ``规划文件`` copy is intentionally only an
        archive for browsing and loading; it is never used to bypass the
        trajectory validator or the execution gate.
        """
        request_path = self.planning_request_path
        output_path = self.planning_output_path
        report_path = self.planning_report_path
        if request_path is None:
            return None

        configured = str(self.config.get("online_planning", {}).get("planning_file_dir", "")).strip()
        archive_dir = Path(configured).expanduser() if configured else ROOT.parent / "规划文件"
        if not archive_dir.is_absolute():
            archive_dir = ROOT.parent / archive_dir
        archive_dir.mkdir(parents=True, exist_ok=True)

        request_id = request_path.parent.name
        algorithm = str(self.online_algorithm_combo.currentData() or "planner")
        task = getattr(self, "online_task_value", "") or self._online_task()
        if report_path is not None and report_path.is_file():
            try:
                report = json.loads(report_path.read_text(encoding="utf-8"))
                algorithm = str(report.get("algorithm", algorithm))
                task = str(report.get("task", task))
            except (OSError, json.JSONDecodeError):
                pass
        slug_map = {
            "RRT": "rrt",
            "RRT-Connect": "rrt_connect",
            "Bi-RRT*": "bi_rrt_star",
            "V1 FastConnectAnytimeRRTStar": "v1_fastconnect_anytime_rrt_star",
        }
        slug = slug_map.get(algorithm, "".join(ch.lower() if ch.isalnum() else "_" for ch in algorithm).strip("_"))
        task_slug = "".join(ch.lower() if ch.isalnum() else "_" for ch in str(task)).strip("_")
        prefix = f"online_{task_slug}_{slug}_{request_id}"

        copied = []
        for source, suffix in (
            (output_path, ".mat"),
            (report_path, "_report.json"),
            (request_path, "_request.json"),
        ):
            if source is None or not source.is_file():
                continue
            if suffix == ".mat":
                destination = archive_dir / f"{prefix}.mat"
            elif suffix == "_report.json":
                destination = archive_dir / f"{prefix}_report.json"
            else:
                destination = archive_dir / f"{prefix}_request.json"
            shutil.copy2(source, destination)
            copied.append(destination)

        if copied:
            self.online_output_view.appendPlainText(f"archived to: {archive_dir}")
            self._log("Online planning archive: " + ", ".join(str(path) for path in copied))
        return archive_dir

    def _start_online_planning(self):
        if getattr(self, "robot_tcp_busy", False):
            self._log("Planning blocked: TCP 参数读取/写入中")
            return
        if self.planning_busy:
            return
        if self._motion_busy() or self.goal_active:
            self._log("Online planning blocked: motion is busy")
            return

        task = self._online_task()
        try:
            q_start, q_goal, endpoints_text = self._task_endpoints(task)
        except Exception as exc:
            QMessageBox.warning(self, "在线规划不可用", str(exc))
            self.online_status_label.setText(f"端点不可用：{exc}")
            self._set_online_stage("FAILED")
            return
        if q_start.size != 6 or not np.isfinite(q_start).all():
            QMessageBox.warning(self, "在线规划不可用", "起点关节数据无效。")
            return

        algorithm = str(self.online_algorithm_combo.currentData())
        request_id = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000:06d}"
        run_dir = ROOT.parent / "planning_runs" / f"hmi_{request_id}"
        run_dir.mkdir(parents=True, exist_ok=True)
        output_path = run_dir / "trajectory.mat"
        report_path = run_dir / "report.json"
        request_path = run_dir / "request.json"
        max_time = float(self.online_budget_spin.value())

        # The weld wrapper handles all four tasks.  Free-space segments use the
        # selected comparison algorithm; the weld segment is always A-ARC-IK+
        # with branchReferenceQ and the Forward -> Reverse cascade.
        request_payload = {
            "request_id": request_id,
            "task": task,
            "qStart": np.asarray(q_start, dtype=float).tolist(),
            "qGoal": np.asarray(q_goal, dtype=float).tolist(),
            "algorithm": algorithm,
            "seed": int(self.online_seed_spin.value()),
            "maxPlanningTimeSec": max_time,
            "start_source": str(self.online_start_combo.currentData()),
            "endpoint_source": endpoints_text,
            "weld_algorithm": "A-ARC-IK+ (replan_weld_actual_axis_adaptive_feasibility.m)",
            # Carry the read-only calibration versions into MATLAB metadata so
            # a future trajectory can be checked against the exact scene it
            # was validated with.  This is not a planning/control parameter.
            "calibration_binding": self._current_calibration_binding(),
            "planning_scene": "REAL_CALIBRATION" if (
                hasattr(self, "calibration_source_combo")
                and self.calibration_source_combo.currentText() == "REAL CALIBRATION"
            ) else "FROZEN_SIMULATION",
        }
        request_path.write_text(
            json.dumps(request_payload, ensure_ascii=False, indent=1), encoding="utf-8"
        )

        matlab_dir = ROOT.parent / "matlab仿真"
        benchmark_dir = ROOT.parent.parent / "RRT" / "benchmark"
        wrapper = benchmark_dir / "plan_hmi_weld_request.m"
        if not wrapper.is_file():
            self.online_status_label.setText(f"找不到 MATLAB 入口：{wrapper}")
            self._set_online_stage("FAILED")
            return
        expression = (
            f"addpath({self._matlab_quote(benchmark_dir)}); "
            f"addpath({self._matlab_quote(matlab_dir)}); "
            f"req=jsondecode(fileread({self._matlab_quote(request_path)})); "
            f"req.outputFile={self._matlab_quote(output_path)}; "
            f"req.reportFile={self._matlab_quote(report_path)}; "
            f"req.requestFile={self._matlab_quote(request_path)}; "
            f"req.requestId='{request_id}'; "
            f"req.maxPlanningTimeSec={max_time}; "
            "plan_hmi_weld_request(req);"
        )
        self.planning_request_path = request_path
        self.planning_output_path = output_path
        self.planning_report_path = report_path
        self.online_task_value = task
        self.online_output_view.clear()
        self.online_output_view.appendPlainText(f"task: {task}")
        self.online_output_view.appendPlainText(f"endpoints: {endpoints_text}")
        self.online_output_view.appendPlainText(f"free-space algorithm: {algorithm}")
        self.online_output_view.appendPlainText("weld algorithm: A-ARC-IK+ (fixed)")
        self.online_output_view.appendPlainText(f"run directory: {run_dir}")
        self.online_output_view.appendPlainText("Launching MATLAB asynchronously…")
        self._reset_online_result()
        self._set_online_stage("STARTING MATLAB")
        self.online_status_label.setText(f"规划中：{task}（GUI 可继续响应）")
        process = QProcess(self)
        process.readyReadStandardOutput.connect(lambda: self._online_log_process_output("stdout"))
        process.readyReadStandardError.connect(lambda: self._online_log_process_output("stderr"))
        process.errorOccurred.connect(self._online_process_error)
        process.finished.connect(self._online_process_finished)
        self.planning_process = process
        self.planning_busy = True
        self._update_buttons()
        self._log(f"Online {task} planning started: {wrapper}; run={run_dir}")
        process.start("matlab", ["-batch", expression])

    def _online_process_finished(self, exit_code, _exit_status):
        self._online_log_process_output("stdout")
        self._online_log_process_output("stderr")
        process = self.planning_process
        self.planning_process = None
        self.planning_busy = False
        if process is not None:
            process.deleteLater()
        if int(exit_code) != 0:
            if self.planning_report_path is not None and self.planning_report_path.is_file():
                try:
                    self._show_online_result(self.planning_report_path)
                except Exception as exc:
                    self._log(f"Online failure report load failed: {exc}")
            try:
                self._archive_online_result()
            except Exception as exc:
                self._log(f"Online result archive failed: {exc}")
            self._set_online_stage("FAILED")
            self.online_status_label.setText(f"规划失败（MATLAB exit code {exit_code}）")
            self._log(f"Online {getattr(self, 'online_task_value', '?')} planning failed: exit code {exit_code}")
            self._update_buttons()
            return
        path = self.planning_output_path
        if path is None or not path.is_file():
            try:
                self._archive_online_result()
            except Exception as exc:
                self._log(f"Online result archive failed: {exc}")
            self._set_online_stage("FAILED")
            self.online_status_label.setText("规划失败：没有生成 trajectory.mat")
            self._log("Online planning finished without MAT output")
            self._update_buttons()
            return
        try:
            payload = self._show_online_result(self.planning_report_path)
            self._archive_online_result()
            self._adopt_trajectory_file(path, auto_validate=True)
            self._apply_planned_tcp_path()
            self.tabs.setCurrentIndex(self.trajectory_tab_index)
            if payload.get("collision_check_pass") and payload.get("continuity_check_pass"):
                self._set_online_stage("SUCCESS")
            else:
                self._set_online_stage("FAILED")
            self.online_status_label.setText(
                "规划成功：已加载到“轨迹执行”页，可预演（未自动执行）"
            )
            self._log(f"[PASS] Online {getattr(self, 'online_task_value', '?')} trajectory loaded: {path}")
        except Exception as exc:
            self._set_online_stage("FAILED")
            self.online_status_label.setText(f"规划完成但加载失败：{exc}")
            QMessageBox.critical(self, "在线规划结果加载失败", str(exc))
            self._log(f"Online weld output load failed: {exc}")
        self._update_buttons()

    def _adopt_trajectory_file(self, path: Path, auto_validate=False):
        self._stop_preview()
        trajectory = load_trajectory(str(path))
        self.loaded_trajectory = trajectory
        self._apply_planned_tcp_path()
        self.loaded_path = trajectory.positions
        self.loaded_path_name = str(path)
        binding_reasons = self._trajectory_binding_reasons(trajectory)
        self.trajectory_stale_reason = "; ".join(binding_reasons) if binding_reasons else None
        if self.trajectory_stale_reason:
            self._log(f"Loaded trajectory as STALE: {self.trajectory_stale_reason}")
        self.trajectory_validation = None
        self.path_label.setText(f"{path.name}: {trajectory.point_count} points")
        self.preview_slider.setRange(0, max(0, trajectory.point_count - 1))
        self.preview_slider.setValue(0)
        self._set_trajectory_summary(None)
        if trajectory.time_from_start is None:
            self.preview_status_label.setText("TIME PARAMETERIZATION REQUIRED")
            self._log(f"Loaded preview-only trajectory without timing: {path}")
        else:
            self.preview_status_label.setText("可预演；尚未验证")
            self._log(f"Loaded trajectory: {path}")
        if auto_validate:
            self._validate_path()
        else:
            self._update_trajectory_gate()
        self._update_buttons()

    def _load_path(self):
        if self.goal_active or self._motion_busy():
            self._log("Load blocked: motion busy (see Motion State / Current Goal)")
            return
        configured = str(self.config.get("online_planning", {}).get("planning_file_dir", "")).strip()
        default_dir = Path(configured).expanduser() if configured else ROOT.parent / "规划文件"
        if not default_dir.is_absolute():
            default_dir = ROOT.parent / default_dir
        if not default_dir.is_dir():
            default_dir = ROOT
        path, _ = QFileDialog.getOpenFileName(
            self, "加载轨迹", str(default_dir), "Trajectory (*.mat *.yaml *.yml *.json)"
        )
        if not path:
            return
        try:
            self._adopt_trajectory_file(Path(path), auto_validate=False)
        except Exception as exc:
            self.loaded_trajectory = None
            self.loaded_path = None
            self.trajectory_validation = None
            self.trajectory_stale_reason = None
            QMessageBox.critical(self, "轨迹加载失败", str(exc))
            self._log(f"Trajectory load failed: {exc}")

    def _validate_path(self):
        if self.loaded_trajectory is None:
            self._log("请先加载轨迹")
            return
        self.trajectory_validation = validate_trajectory(self.loaded_trajectory)
        report = self.trajectory_validation
        self._set_trajectory_summary(report)
        if report.ok:
            self._log(
                f"[PASS] trajectory validation: {report.point_count} points, "
                f"duration={report.duration if report.duration is not None else 'N/A'}"
            )
            if not report.time_valid:
                self.preview_status_label.setText("TIME PARAMETERIZATION REQUIRED")
        else:
            self._log("[FAIL] trajectory validation: " + "; ".join(report.issues[:8]))
        self._update_trajectory_gate()
        self._update_buttons()

    def _confirm_execute(self) -> bool:
        traj = self.loaded_trajectory
        report = validate_trajectory(traj, q_actual=self.state.positions)
        dialog = QDialog(self)
        dialog.setWindowTitle("二次确认：执行真实机器人轨迹")
        dialog.setMinimumWidth(560)
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel(
            f"即将执行真机轨迹\n"
            f"轨迹：{traj.metadata.get('trajectory_name', Path(self.loaded_path_name).stem)}\n"
            f"点数：{traj.point_count}\n"
            f"时长：{traj.duration:.3f} s\n"
            f"Command Speed：{int(self.speed_slider.value())}%\n"
            f"最大起点误差：{math.degrees(np.max(np.abs(report.start_error_rad))):.3f}°"
        ))
        checks = [
            "机器人周围无人",
            "工件/障碍位置已确认",
            "示教器急停可立即操作",
            "焊机当前关闭",
        ]
        boxes = []
        for text in checks:
            box = QCheckBox(text)
            boxes.append(box)
            layout.addWidget(box)
        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        confirm = buttons.addButton("CONFIRM EXECUTE", QDialogButtonBox.AcceptRole)
        confirm.setEnabled(False)
        for box in boxes:
            box.toggled.connect(lambda _checked: confirm.setEnabled(all(item.isChecked() for item in boxes)))
        buttons.rejected.connect(dialog.reject)
        buttons.accepted.connect(dialog.accept)
        layout.addWidget(buttons)
        return dialog.exec_() == QDialog.Accepted

    def _execute_path(self):
        if getattr(self, "robot_tcp_busy", False):
            self._log("Execution blocked: TCP 参数读取/写入中")
            return
        if not self._final_execution_gate_ok():
            self._update_trajectory_gate()
            self._log("Execution blocked: FINAL EXECUTION CHECK 未通过")
            return
        if not self._confirm_execute():
            return
        self.execution_trajectory = self.loaded_trajectory
        self.execution_started_monotonic = time.monotonic()
        self.execution_log = []
        self._log("[ARMED] 二次确认完成，发送低速轨迹请求")
        try:
            self.worker.request_trajectory.emit(trajectory_to_ros_message(self.execution_trajectory))
        except Exception as exc:
            self._log(f"Trajectory conversion failed: {exc}")
            self.execution_started_monotonic = None
            self.execution_trajectory = None

    def _preview_speed(self) -> float:
        return float(self.preview_speed_combo.currentText().rstrip("x"))

    # ------------------------------------------------------------------
    # Offline gun-installation adjustment (display/calibration only)
    # ------------------------------------------------------------------
    def _gun_mount_values_from_ui(self) -> tuple[float, list[float]]:
        roll = float(self.gun_roll_spin.value())
        translation = [float(spin.value()) for spin in self.gun_translation_spins]
        return roll, translation

    def _on_gun_mount_adjustment_changed(self, _value=None) -> None:
        if not hasattr(self, "gun_roll_spin"):
            return
        try:
            roll, translation = self._gun_mount_values_from_ui()
            setter = getattr(self.robot_view, "set_gun_mount_adjustment", None)
            if setter is None:
                raise RuntimeError("当前3D视图不支持焊枪安装调整")
            setter(roll, translation, render=True)
            self.gun_mount_adjustment = {
                "roll_deg": roll,
                "translation_mm": translation,
                "status": "PREVIEW",
            }
            self._update_gun_mount_panel()
        except Exception as exc:
            self._log(f"焊枪安装预览失败：{exc}")

    def _nudge_gun_roll(self, delta_deg: float) -> None:
        self.gun_roll_spin.setValue(float(np.clip(self.gun_roll_spin.value() + float(delta_deg), -180.0, 180.0)))

    def _on_gun_visibility_changed(self, _state=None) -> None:
        flags = {}
        current = getattr(self.robot_view, "visibility", lambda: {})()
        for key, box in getattr(self.gun_visibility_boxes, "items", lambda: [])():
            flags[key] = box.isChecked()
        # Preserve seam/workpiece/path/collision layers controlled by the main
        # scene switches.  This local page only changes robot/gun/wire/LIVE.
        for key in ("workpiece", "seam", "tcp_path", "preview_tcp", "collision"):
            if key in current:
                flags[key] = current[key]
        try:
            self.robot_view.set_visibility(**flags)
        except Exception as exc:
            self._log(f"焊枪显示开关更新失败：{exc}")
        for key in ("robot", "gun", "wire", "current_tcp"):
            top_box = getattr(self, "scene_visibility_boxes", {}).get(key)
            if top_box is not None and key in flags:
                top_box.blockSignals(True)
                top_box.setChecked(bool(flags[key]))
                top_box.blockSignals(False)

    def _set_gun_view(self, name: str) -> None:
        if name == "gun":
            # Hiding the robot is an explicit view convenience only; it does
            # not alter the robot state or any planning/collision data.
            box = self.gun_visibility_boxes.get("robot")
            if box is not None:
                box.setChecked(False)
            setter = getattr(self.robot_view, "fit_gun_view", None)
            if setter is not None:
                setter("iso")
            return
        setter = getattr(self.robot_view, "fit_gun_view", None)
        if setter is not None:
            setter(name)
        else:
            self._set_robot_view(name)

    def _update_gun_mount_panel(self) -> None:
        if not hasattr(self, "gun_roll_spin"):
            return
        info = getattr(self.robot_view, "gun_mount_adjustment", lambda: {})()
        axis = info.get("rotation_axis_flange")
        if axis is None:
            self.gun_axis_label.setText("不可用")
        else:
            self.gun_axis_label.setText("[" + ", ".join(f"{float(v):+.4f}" for v in axis) + "]")
        roll = float(self.gun_roll_spin.value())
        translation = [float(spin.value()) for spin in self.gun_translation_spins]
        self.gun_roll_label.setText(f"{roll:+.1f}°")
        status = str(info.get("status", "PREVIEW"))
        self.gun_mount_status_label.setText(
            f"{status}｜flange局部：roll {roll:+.1f}°，Δxyz "
            f"[{translation[0]:+.1f}, {translation[1]:+.1f}, {translation[2]:+.1f}] mm"
        )
        diag = getattr(self.robot_view, "gun_mount_wire_diagnostics", lambda: {"available": False, "reason": "不可用"})()
        if diag.get("available"):
            delta = diag.get("delta_xyz_mm", [np.nan, np.nan, np.nan])
            self.gun_delta_labels["dx"].setText(f"{float(delta[0]):+.3f} mm")
            self.gun_delta_labels["dy"].setText(f"{float(delta[1]):+.3f} mm")
            self.gun_delta_labels["dz"].setText(f"{float(delta[2]):+.3f} mm")
            self.gun_delta_labels["distance"].setText(f"{float(diag.get('distance_mm', np.nan)):.3f} mm")
            self.gun_delta_labels["source"].setText(str(diag.get("live_tcp_source", "LIVE TCP")))
        else:
            reason = str(diag.get("reason", "等待实时 TCP"))
            for key in ("dx", "dy", "dz", "distance"):
                self.gun_delta_labels[key].setText("—")
            self.gun_delta_labels["source"].setText(reason)

    def _gun_mount_payload(self, status: str) -> dict:
        info = getattr(self.robot_view, "gun_mount_adjustment", lambda: {})()
        diag = getattr(self.robot_view, "gun_mount_wire_diagnostics", lambda: {})()
        axis = np.asarray(info.get("rotation_axis_flange", [1.0, 0.0, 0.0]), dtype=float).reshape(-1)
        axis_valid = axis.size == 3 and np.all(np.isfinite(axis)) and float(np.linalg.norm(axis)) > 1e-12
        if axis_valid:
            axis = axis / float(np.linalg.norm(axis))
        else:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        theta = math.radians(float(info.get("roll_deg", self.gun_roll_spin.value())))
        x, y, z = axis
        c, s, one_c = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
        R = np.array([
            [x*x*one_c+c, x*y*one_c-z*s, x*z*one_c+y*s],
            [y*x*one_c+z*s, y*y*one_c+c, y*z*one_c-x*s],
            [z*x*one_c-y*s, z*y*one_c+x*s, z*z*one_c+c],
        ], dtype=float)
        translation = np.asarray(info.get("translation_mm", [spin.value() for spin in self.gun_translation_spins]), dtype=float).reshape(3) * 1e-3
        T_adjust = np.eye(4, dtype=float)
        T_adjust[:3, :3] = R
        T_adjust[:3, 3] = translation
        return {
            "schema_version": 1,
            "calibration_type": "real_gun_mount_adjustment",
            "coordinate_frame": "URDF flange",
            "rotation_axis_flange": info.get("rotation_axis_flange"),
            "rotation_axis_source": "URDF wrist_3_joint / J6 local axis",
            "roll_deg": float(info.get("roll_deg", self.gun_roll_spin.value())),
            "translation_flange_mm": [float(v) for v in info.get("translation_mm", [spin.value() for spin in self.gun_translation_spins])],
            "T_flange_adjustment": T_adjust.tolist(),
            "status": status,
            "valid": bool(axis_valid),
            "confirmed_by_operator": status == "CONFIRMED",
            "applied_to_matlab": False,
            "requires_replan": True,
            "requires_collision_revalidation": True,
            "wire_tcp_error_mm": diag,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "version": "UR5-HMI-GUN-MOUNT-1",
        }

    def _write_gun_mount_record(self, status: str) -> bool:
        payload = self._gun_mount_payload(status)
        if not payload["valid"]:
            QMessageBox.warning(self, "焊枪安装标定", "未能读取 URDF flange/J6 局部旋转轴，不能保存。")
            return False
        try:
            self.gun_mount_calibration_file.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            status_setter = getattr(self.robot_view, "set_gun_mount_calibration_status", None)
            if status_setter is not None:
                status_setter(status)
            self.gun_mount_adjustment["status"] = status
            self._log(f"焊枪安装标定已保存：{status}（独立文件，不修改 MATLAB 场景）")
            return True
        except Exception as exc:
            QMessageBox.critical(self, "保存焊枪安装标定失败", str(exc))
            return False

    def _save_gun_mount_draft(self) -> None:
        self._write_gun_mount_record("DRAFT")

    def _confirm_gun_mount_calibration(self) -> None:
        answer = QMessageBox.question(
            self,
            "确认焊枪安装标定",
            "请确认已用实物测量核对焊枪安装姿态、焊丝尖端与 LIVE TCP。\n"
            "确认后只保存独立版本并使已加载旧轨迹 STALE；不会写入示教器 TCP、MATLAB 场景或发送运动。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        if self._write_gun_mount_record("CONFIRMED"):
            self._mark_trajectory_stale("焊枪安装标定版本已改变，需重新碰撞验证/规划")
            self._update_trajectory_gate()
            self._update_gun_mount_panel()

    def _reset_gun_mount_adjustment(self) -> None:
        self.gun_roll_spin.blockSignals(True)
        for spin in self.gun_translation_spins:
            spin.blockSignals(True)
        self.gun_roll_spin.setValue(90.0)
        for spin in self.gun_translation_spins:
            spin.setValue(0.0)
        self.gun_roll_spin.blockSignals(False)
        for spin in self.gun_translation_spins:
            spin.blockSignals(False)
        self._on_gun_mount_adjustment_changed()

    def _load_gun_mount_calibration(self, _checked=False, silent: bool = False) -> None:
        path = self.gun_mount_calibration_file
        if not path.is_file():
            if not silent:
                QMessageBox.information(self, "加载焊枪安装标定", "尚未保存 real_gun_mount_calibration.json。")
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            roll = float(payload.get("roll_deg"))
            translation = np.asarray(payload.get("translation_flange_mm", [0, 0, 0]), dtype=float).reshape(-1)
            axis = np.asarray(payload.get("rotation_axis_flange"), dtype=float).reshape(-1)
            if not (-180.0 <= roll <= 180.0 and translation.size == 3 and axis.size == 3 and np.all(np.isfinite(translation)) and np.all(np.isfinite(axis))):
                raise ValueError("标定记录角度/轴/平移无效")
            self.gun_roll_spin.blockSignals(True)
            self.gun_roll_spin.setValue(roll)
            for spin, value in zip(self.gun_translation_spins, translation):
                spin.blockSignals(True); spin.setValue(float(value)); spin.blockSignals(False)
            self.gun_roll_spin.blockSignals(False)
            self._on_gun_mount_adjustment_changed()
            status = str(payload.get("status", "DRAFT"))
            status_setter = getattr(self.robot_view, "set_gun_mount_calibration_status", None)
            if status_setter is not None:
                status_setter(status)
            self.gun_mount_adjustment["status"] = status
            self._update_gun_mount_panel()
            if not silent:
                self._log(f"已加载焊枪安装标定：{status}（仅3D预览）")
        except Exception as exc:
            if not silent:
                QMessageBox.warning(self, "加载焊枪安装标定失败", str(exc))
            self._log(f"焊枪安装标定加载失败：{exc}")

    def _set_robot_view(self, name: str) -> None:
        """Apply a read-only camera preset without changing any model pose."""
        if name == "robot":
            setter = getattr(self.robot_view, "fit_robot_view", None)
            if setter is not None:
                setter("iso")
            else:
                self.robot_view.set_view("iso")
            return
        if name == "robot_scene":
            setter = getattr(self.robot_view, "fit_scene_view", None)
            if setter is not None:
                setter("iso")
            elif hasattr(self.robot_view, "set_view_with_scene"):
                self.robot_view.set_view_with_scene("iso")
            else:
                self.robot_view.set_view("iso")
            return

        # Direction buttons retain the current default framing mode.  The
        # default is robot+workpiece, and the robot-only checkbox is still
        # respected when the workpiece is hidden.
        workpiece_box = getattr(self, "scene_visibility_boxes", {}).get("workpiece")
        include_scene = True if workpiece_box is None else workpiece_box.isChecked()
        if include_scene and hasattr(self.robot_view, "fit_scene_view"):
            self.robot_view.fit_scene_view(name)
        elif hasattr(self.robot_view, "fit_robot_view"):
            self.robot_view.fit_robot_view(name)
        elif hasattr(self.robot_view, "set_view_with_scene"):
            self.robot_view.set_view_with_scene(name)
        else:
            self.robot_view.set_view(name)

    def _refresh_scene_status_label(self) -> None:
        if not hasattr(self, "scene_status_label"):
            return
        try:
            status = self.robot_view.scene_status()
        except Exception:
            status = "unavailable"
        self.scene_status_label.setText(
            f"3D 场景：{status}｜工件 UR5_base→base；TCP 路径 base_link→base(Rz180°)"
        )

    def _on_scene_visibility_changed(self, _state=None) -> None:
        flags = {
            key: box.isChecked() for key, box in getattr(self, "scene_visibility_boxes", {}).items()
        }
        try:
            self.robot_view.set_visibility(**flags)
        except Exception as exc:
            self._log(f"Scene visibility update failed: {exc}")
        for key in ("robot", "gun", "wire", "current_tcp"):
            local_box = getattr(self, "gun_visibility_boxes", {}).get(key)
            if local_box is not None and key in flags:
                local_box.blockSignals(True)
                local_box.setChecked(bool(flags[key]))
                local_box.blockSignals(False)
        # The planned TCP path line is hidden until a trajectory provides one.
        if not flags.get("tcp_path", False) and self.loaded_trajectory is not None:
            pass
        # Re-frame using the default combined mode when the workpiece layer
        # changes; the camera follows visibility, not geometry coordinates.
        self._set_robot_view("robot_scene" if flags.get("workpiece", False) else "robot")

    def _apply_planned_tcp_path(self) -> None:
        """Feed the loaded trajectory's tcpPath into the 3D view.

        tcpPath is exported by the MATLAB planner in the ``base_link`` frame and
        is converted inside the view with the verified Rz(180 deg) transform.
        """
        traj = self.loaded_trajectory
        setter = getattr(self.robot_view, "set_planned_tcp_path", None)
        if setter is None:
            return
        if traj is None or traj.tcp_path is None or len(traj.tcp_path) < 2:
            setter(None)
            return
        setter(np.asarray(traj.tcp_path, dtype=float), frame="base_link")
        box = getattr(self, "scene_visibility_boxes", {}).get("tcp_path")
        if box is not None and not box.isChecked():
            box.setChecked(True)
        self._set_preview_pose(getattr(self, "preview_index", 0))

    def _set_preview_pose(self, index: int):
        traj = self.loaded_trajectory
        if traj is None or traj.point_count == 0:
            return
        index = max(0, min(int(index), traj.point_count - 1))
        self.preview_index = index
        if hasattr(self.robot_view, "set_preview_state"):
            # Preview q/TCP is a separate state.  The live red marker continues
            # to use the latest ROS feedback while the blue marker follows qPath.
            self.robot_view.set_preview_state(traj.positions[index].tolist(), index)
        else:
            self.robot_view.set_state(traj.positions[index].tolist(), None, "")
        self.preview_slider.blockSignals(True)
        self.preview_slider.setValue(index)
        self.preview_slider.blockSignals(False)
        if traj.time_from_start is None:
            self.preview_status_label.setText(f"预演点 {index + 1}/{traj.point_count}（无时间参数）")
        else:
            self.preview_status_label.setText(
                f"预演点 {index + 1}/{traj.point_count}，t={traj.time_from_start[index]:.3f}s"
            )

    def _start_preview(self):
        traj = self.loaded_trajectory
        if traj is None:
            self._log("请先加载轨迹")
            return
        if self.goal_active:
            self._log("Preview blocked: action 正在执行")
            return
        if traj.time_from_start is None:
            self._set_preview_pose(self.preview_index)
            self.preview_status_label.setText("TIME PARAMETERIZATION REQUIRED（仅可拖动查看点）")
            return
        self.preview_active = True
        factor = self._preview_speed()
        if self.preview_start_monotonic is None or self.preview_index >= traj.point_count - 1:
            self.preview_index = 0
            self.preview_start_monotonic = time.monotonic()
        else:
            self.preview_start_monotonic = time.monotonic() - float(traj.time_from_start[self.preview_index]) / factor
        self.preview_timer.start()
        self._set_preview_pose(self.preview_index)

    def _pause_preview(self):
        self.preview_timer.stop()
        self.preview_status_label.setText("预演已暂停")

    def _stop_preview(self):
        self.preview_timer.stop()
        self.preview_active = False
        diagnostics = getattr(self.robot_view, "tcp_preview_diagnostics", None)
        if callable(diagnostics):
            try:
                report = diagnostics()
                if report.get("frame_count"):
                    path_error = report.get("max_tcp_path_error_mm")
                    relative_error = report.get("max_flange_tcp_relative_error_mm")
                    relative_angle = report.get("max_flange_tcp_relative_orientation_error_deg")
                    message = (
                        "Preview TCP 校核："
                        f"frames={report['frame_count']}，"
                        f"相对变换最大误差={relative_error:.6f} mm，"
                        f"姿态误差={relative_angle:.6f}°，"
                        f"与 MATLAB tcpPath 最大位置偏差="
                        + (f"{path_error:.3f} mm" if path_error is not None else "不可用")
                    )
                    self._log(message)
            except Exception as exc:
                self._log(f"Preview TCP 校核失败：{exc}")
        self.preview_start_monotonic = None
        self.preview_index = 0
        if hasattr(self, "preview_slider"):
            self.preview_slider.blockSignals(True)
            self.preview_slider.setValue(0)
            self.preview_slider.blockSignals(False)
        clear_preview = getattr(self.robot_view, "clear_preview_state", None)
        if clear_preview is not None:
            clear_preview()
        setter = getattr(self.robot_view, "set_live_state", self.robot_view.set_state)
        if self._feedback_valid(self.state):
            setter(self.state.positions, self.state.tcp_position, self.state.tcp_frame_id, self.state.tcp_stamp)
        else:
            setter(None, None, "", 0.0)
        if hasattr(self, "preview_status_label"):
            self.preview_status_label.setText("预演已停止")

    def _preview_slider_changed(self, value: int):
        if self.loaded_trajectory is None:
            return
        self._set_preview_pose(value)
        if self.preview_active and self.loaded_trajectory.time_from_start is not None:
            self.preview_start_monotonic = time.monotonic() - float(self.loaded_trajectory.time_from_start[value]) / self._preview_speed()

    def _preview_tick(self):
        traj = self.loaded_trajectory
        if not self.preview_active or traj is None or traj.time_from_start is None:
            self.preview_timer.stop()
            return
        elapsed = (time.monotonic() - self.preview_start_monotonic) * self._preview_speed()
        t = np.asarray(traj.time_from_start, dtype=float)
        index = int(np.searchsorted(t, elapsed, side="right") - 1)
        index = max(0, min(index, traj.point_count - 1))
        self._set_preview_pose(index)
        if elapsed >= float(t[-1]):
            self.preview_timer.stop()
            self.preview_active = False
            self.preview_status_label.setText(f"预演完成：{traj.point_count}/{traj.point_count}")

    # ------------------------------------------------------------------
    # Workpiece calibration (read-only TCP teaching)
    # ------------------------------------------------------------------
    def _capture_calibration_point(self, key: str) -> None:
        """Capture one calibration point from the latest valid TCP sample."""
        if key not in self.calibration_points:
            return
        if not self._feedback_valid(self.state) or self.state.tcp_position is None:
            message = "标定点读取失败：没有最新且有效的实时 TCP 反馈"
            self._log(message)
            label = self.calibration_point_labels.get(key)
            if label is not None:
                label.setText("读取失败：TCP 无效或已过期")
                label.setStyleSheet("background:#ffe5e5;color:#9b1c1c;padding:6px 8px;font-family:monospace;")
            return
        point = np.asarray(self.state.tcp_position, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            self._log(f"标定点 {key} 读取失败：TCP 数值无效")
            return
        self.calibration_points[key] = point.copy()
        self.calibration_point_timestamps[key] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.calibration_point_frames[key] = str(self.state.tcp_frame_id or "base")
        label = self.calibration_point_labels.get(key)
        if label is not None:
            label.setText("X: %.3f   Y: %.3f   Z: %.3f mm" % tuple(point * 1000.0))
            label.setStyleSheet("background:#e8f5e9;color:#146c2e;padding:6px 8px;font-family:monospace;")
            label.setToolTip(
                f"frame={self.state.tcp_frame_id or 'base'}; captured={time.strftime('%H:%M:%S')}"
            )
        self.calibration_result = None
        self.calibration_save_button.setEnabled(False)
        self.calibration_display_button.setEnabled(False)
        self._log(f"已读取标定点 {key}：" + ", ".join(f"{v * 1000.0:.3f} mm" for v in point))

    @staticmethod
    def _rpy_zyx_from_rotation(rotation: np.ndarray) -> np.ndarray:
        """Return roll, pitch, yaw (radians) for R = Rz(yaw)Ry(pitch)Rx(roll)."""
        value = float(np.clip(-rotation[2, 0], -1.0, 1.0))
        pitch = math.asin(value)
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        return np.array([roll, pitch, yaw], dtype=float)

    @staticmethod
    def _workpiece_quality_band(residual_mm: float | None) -> tuple[str, str, str]:
        """Return the non-blocking workpiece calibration quality hint."""
        if residual_mm is None or not np.isfinite(residual_mm):
            return "UNVERIFIED", "尚未采集 P_CHECK（建议验证）", "color:#8a6d00;font-weight:bold;"
        value = float(residual_mm)
        if value <= 2.0:
            return "GOOD", "≤2 mm（较好）", "color:#146c2e;font-weight:bold;"
        if value <= 5.0:
            return "WARNING", "2–5 mm（一般，建议重新标定）", "color:#8a6d00;font-weight:bold;"
        if value <= 6.0:
            return "POOR", "5–6 mm（较差；应用需人工确认）", "color:#9b1c1c;font-weight:bold;"
        if value <= 10.0:
            return "POOR", "6–10 mm（较差；可保存/3D预览，禁止真实规划）", "color:#9b1c1c;font-weight:bold;"
        return "POOR", ">10 mm（超出3D预览建议上限，建议重新标定）", "color:#9b1c1c;font-weight:bold;"

    def _workpiece_application_reasons(self, result=None) -> list[str]:
        """Hard gate for applying a real workpiece frame to the scene/planner."""
        result = result or self.calibration_result
        reasons: list[str] = []
        if not result or not result.get("geometry_valid", result.get("valid", False)):
            reasons.append("三点几何结果不可用")
            return reasons
        transform = np.asarray(result.get("T_base_workpiece_real"), dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            reasons.append("工件变换不是有限 4×4 矩阵")
        else:
            if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
                reasons.append("工件变换齐次行无效")
            rotation = transform[:3, :3]
            if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5)
                    or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-5)):
                reasons.append("工件坐标系旋转矩阵无效")
        # P_CHECK is intentionally not needed to save a draft, but it is
        # required before displaying/applying the real frame for planning.
        pcheck = self.calibration_points.get("P_CHECK")
        residual = result.get("residual_mm")
        if pcheck is None or residual is None:
            reasons.append("需要 P_CHECK 独立几何验证")
        elif not np.isfinite(float(residual)) or float(residual) > self.calibration_residual_limit_mm:
            reasons.append(f"P_CHECK 残差超过应用精度 {self.calibration_residual_limit_mm:.1f} mm")
        frames = [self.calibration_point_frames.get(key) for key in ("P0", "PX", "PY")]
        if any(frame not in (None, "base", "base_link") for frame in frames):
            reasons.append("标定点不在统一 base 坐标系")
        for key in ("P0", "PX", "PY", "P_CHECK"):
            point = self.calibration_points.get(key)
            if point is not None:
                value = np.asarray(point, dtype=float)
                if value.shape != (3,) or not np.all(np.isfinite(value)):
                    reasons.append(f"{key} 坐标数据无效")
        return reasons

    def _calculate_calibration(self) -> None:
        """Compute the workpiece frame from P0/PX/PY; P_CHECK is optional."""
        points = self.calibration_points
        if any(points[name] is None for name in ("P0", "PX", "PY")):
            self._log("标定计算失败：请先读取 P0、PX、PY")
            self.calibration_result = None
            self.calibration_save_button.setEnabled(False)
            self.calibration_display_button.setEnabled(False)
            self.calibration_result_labels["status"].setText("INCOMPLETE：缺少 P0/PX/PY")
            self.calibration_result_labels["status"].setStyleSheet("color:#8a6d00;font-weight:bold;")
            return
        # Reject malformed or non-finite samples before any vector operation.
        # P_CHECK remains optional, but if it was captured it must also be a
        # valid 3-vector; otherwise the operator must recapture it instead of
        # silently treating corrupted data as an unverified draft.
        for name in ("P0", "PX", "PY", "P_CHECK"):
            value = points.get(name)
            if value is None:
                continue
            array = np.asarray(value, dtype=float)
            if array.shape != (3,) or not np.all(np.isfinite(array)):
                self._log(f"标定计算失败：{name} 坐标数据无效（需有限 3 维数值）")
                self.calibration_result = None
                self.calibration_save_button.setEnabled(False)
                self.calibration_display_button.setEnabled(False)
                self.calibration_result_labels["status"].setText(f"INVALID：{name} 数据无效")
                self.calibration_result_labels["status"].setStyleSheet("color:#9b1c1c;font-weight:bold;")
                return
        p0 = np.asarray(points["P0"], dtype=float)
        px = np.asarray(points["PX"], dtype=float)
        py = np.asarray(points["PY"], dtype=float)
        x_vector = px - p0
        y_temp = py - p0
        x_norm = float(np.linalg.norm(x_vector))
        y_norm = float(np.linalg.norm(y_temp))
        cross = np.cross(x_vector, y_temp)
        cross_norm = float(np.linalg.norm(cross))
        if min(x_norm, y_norm, cross_norm) <= 1e-5:
            self._log("标定计算失败：示教点距离过小或 P0/PX/PY 共线")
            self.calibration_result = None
            self.calibration_save_button.setEnabled(False)
            self.calibration_display_button.setEnabled(False)
            self.calibration_result_labels["status"].setText("INVALID：点距离/夹角不满足要求")
            self.calibration_result_labels["status"].setStyleSheet("color:#9b1c1c;font-weight:bold;")
            return

        x_axis = x_vector / x_norm
        z_axis = cross / cross_norm
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-12)
        rotation = np.column_stack((x_axis, y_axis, z_axis))
        if np.linalg.det(rotation) <= 0.99:
            self._log("标定计算失败：生成的旋转矩阵不是右手系")
            self.calibration_result = None
            self.calibration_save_button.setEnabled(False)
            self.calibration_display_button.setEnabled(False)
            self.calibration_result_labels["status"].setText("INVALID：旋转矩阵不是右手系")
            self.calibration_result_labels["status"].setStyleSheet("color:#9b1c1c;font-weight:bold;")
            return

        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rotation
        transform[:3, 3] = p0
        rpy = self._rpy_zyx_from_rotation(rotation)
        pcheck = points.get("P_CHECK")
        residual_mm = None if pcheck is None else float(np.linalg.norm(np.asarray(pcheck) - p0) * 1000.0)
        quality_band, quality_text, quality_style = self._workpiece_quality_band(residual_mm)
        # ``valid`` now means the three-point geometry is numerically valid;
        # P_CHECK is an optional review measurement and is not a save gate.
        valid = True
        self.calibration_result = {
            "T_base_workpiece_real": transform,
            "position": p0.copy(),
            "rotation_matrix": rotation.copy(),
            "rpy": rpy.copy(),
            "residual_mm": residual_mm,
            "valid": bool(valid),
            "geometry_valid": True,
            "quality_band": quality_band,
            "quality_text": quality_text,
            "review_status": "VERIFIED" if residual_mm is not None and residual_mm <= 2.0 else "DRAFT",
            "source": "REAL CALIBRATION",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        self.calibration_result_labels["position"].setText(
            "[%.6f, %.6f, %.6f]" % tuple(p0)
        )
        self.calibration_result_labels["rpy"].setText(
            "[%.3f°, %.3f°, %.3f°]" % tuple(np.degrees(rpy))
        )
        self.calibration_result_labels["residual"].setText(
            "未读取 P_CHECK（可保存草稿）" if residual_mm is None else f"{residual_mm:.3f} mm"
        )
        self.calibration_result_labels["residual"].setStyleSheet(quality_style)
        self.calibration_result_labels["quality"].setText(
            f"{quality_band}：{quality_text}（仅初步提示，非官方精度标准）"
        )
        self.calibration_result_labels["quality"].setStyleSheet(quality_style)
        if pcheck is None:
            self.calibration_result_labels["check"].setText("未读取 P_CHECK（建议验证；不影响保存）")
        else:
            measured = np.asarray(pcheck, dtype=float)
            self.calibration_result_labels["check"].setText(
                "预测 [%s] mm；实测 [%s] mm"
                % (
                    ", ".join(f"{v * 1000.0:.3f}" for v in p0),
                    ", ".join(f"{v * 1000.0:.3f}" for v in measured),
                )
            )
        self.calibration_matrix_label.setText(
            "\n".join("[" + ", ".join(f"{v:.6f}" for v in row) + "]" for row in rotation)
        )
        status_text = (
            f"{self.calibration_result['review_status']} | {quality_band}"
            + ("（待 P_CHECK 验证）" if residual_mm is None else "（可保存；应用仍需独立验证）")
        )
        status_style = quality_style
        self.calibration_result_labels["status"].setText(status_text)
        self.calibration_result_labels["status"].setStyleSheet(status_style)
        self.calibration_save_button.setEnabled(bool(valid))
        self.calibration_display_button.setEnabled(bool(valid))
        self._log(
            f"工件三点几何计算成功：{quality_band}；"
            + ("P_CHECK 未采集，结果为 DRAFT" if residual_mm is None else f"P_CHECK={residual_mm:.3f} mm")
            + "；未自动应用 REAL CALIBRATION"
        )

    def _save_calibration(self) -> None:
        result = self.calibration_result
        if not result or not result.get("geometry_valid", result.get("valid")):
            self._log("保存标定被拒绝：三点几何结果不可用")
            return
        rotation = np.asarray(result["rotation_matrix"], dtype=float)
        payload = {
            "frame_id": "base",
            "position": np.asarray(result["position"], dtype=float).tolist(),
            "position_unit": "m",
            "rotation_matrix": rotation.tolist(),
            "rpy": np.asarray(result["rpy"], dtype=float).tolist(),
            "rpy_deg": np.degrees(np.asarray(result["rpy"], dtype=float)).tolist(),
            "timestamp": result["timestamp"],
            "residual_mm": None if result["residual_mm"] is None else float(result["residual_mm"]),
            "valid": True,
            "geometry_valid": True,
            "review_status": str(result.get("review_status", "DRAFT")),
            "quality_band": str(result.get("quality_band", "UNVERIFIED")),
            "quality_text": str(result.get("quality_text", "")),
            "quality_note": "HMI 初步质量提示，不是官方精度标准；P_CHECK 为建议验证",
            "source": "REAL CALIBRATION",
            "points": {
                key: None if value is None else np.asarray(value, dtype=float).tolist()
                for key, value in self.calibration_points.items()
            },
            "point_samples": {
                key: {
                    "position_m": None if self.calibration_points.get(key) is None else np.asarray(self.calibration_points[key], dtype=float).tolist(),
                    "timestamp": self.calibration_point_timestamps.get(key),
                    "frame_id": self.calibration_point_frames.get(key),
                }
                for key in self.calibration_points
            },
        }
        try:
            if self.calibration_file.exists():
                history_dir = self.calibration_file.parent / "workpiece_calibration_history"
                history_dir.mkdir(parents=True, exist_ok=True)
                backup = history_dir / f"workpiece_pose_real_{time.strftime('%Y%m%d_%H%M%S')}.json"
                suffix = 1
                while backup.exists():
                    backup = history_dir / f"workpiece_pose_real_{time.strftime('%Y%m%d_%H%M%S')}_{suffix}.json"
                    suffix += 1
                shutil.copy2(self.calibration_file, backup)
            self.calibration_file.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        except Exception as exc:
            self._log(f"保存工件标定失败：{exc}")
            return
        self._log(
            f"工件标定已保存为 {payload['review_status']}：{self.calibration_file}；"
            "保存不会自动应用真实工件位姿"
        )

    def _reset_calibration(self) -> None:
        self.calibration_points = {"P0": None, "PX": None, "PY": None, "P_CHECK": None}
        self.calibration_point_timestamps = {"P0": None, "PX": None, "PY": None, "P_CHECK": None}
        self.calibration_point_frames = {"P0": None, "PX": None, "PY": None, "P_CHECK": None}
        self.calibration_result = None
        for label in self.calibration_point_labels.values():
            label.setText("X: —   Y: —   Z: — mm")
            label.setStyleSheet("background:#f3f4f6;color:#374151;padding:6px 8px;font-family:monospace;")
            label.setToolTip("等待有效、最新的实时 TCP")
        for key, label in self.calibration_result_labels.items():
            label.setText("—")
            label.setStyleSheet("")
        self.calibration_matrix_label.setText("—")
        self.calibration_save_button.setEnabled(False)
        self.calibration_display_button.setEnabled(False)
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.blockSignals(False)
        self._restore_simulation_calibration(log=False)
        self._log("已清空本次工件标定点，可重新示教")

    def _restore_simulation_calibration(self, log: bool = True) -> None:
        try:
            self.robot_view.clear_workpiece_calibration()
            self._refresh_scene_status_label()
            self._mark_trajectory_stale("工件位姿源切换为 SIMULATION")
        except Exception as exc:
            self._log(f"恢复仿真工件位姿失败：{exc}")
            return
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.blockSignals(False)
        if log:
            self._log("3D 工件显示已恢复为 SIMULATION；未修改保存的标定文件")

    def _load_saved_calibration_action(self) -> None:
        """Restore the saved workpiece calibration into the HMI only.

        Loading never changes the displayed scene source automatically.  The
        operator must explicitly request the separate 3D-display action, which
        reuses the existing validation and confirmation gate.
        """
        if not self._load_saved_calibration():
            self._log(f"恢复工件标定失败：未找到或无法读取 {self.calibration_file}")
            return
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.blockSignals(False)
        self._log(
            f"已恢复保存的工件标定数据：{self.calibration_file}；"
            "当前仍显示 SIMULATION，可点击‘显示到3D（预览）’"
        )

    def _display_saved_calibration(self) -> None:
        """Display the restored/calculated real frame in the 3D scene.

        Approved results are routed through the existing REAL CALIBRATION
        application gate.  Draft/poor results may be shown only as an
        explicitly labelled diagnostic preview; they never enable planning.
        """
        result = self.calibration_result
        if not (result and result.get("geometry_valid", result.get("valid"))):
            if not self._load_saved_calibration():
                self._log("显示真实工件位置被阻止：没有可恢复的有效标定结果")
                return
            result = self.calibration_result
        reasons = self._workpiece_application_reasons(result)
        if reasons:
            residual = result.get("residual_mm")
            if residual is not None and (
                    not np.isfinite(float(residual))
                    or float(residual) > self.calibration_preview_limit_mm
            ):
                self._log(
                    f"真实工件 3D 预览被阻止：P_CHECK 残差超过 "
                    f"{self.calibration_preview_limit_mm:.1f} mm 预览上限"
                )
                return
            answer = QMessageBox.question(
                self, "仅显示真实工件预览",
                "当前标定尚未满足真实规划应用门：\n"
                + "；".join(reasons)
                + "\n\n是否仅在 3D 中显示该实测位姿？\n"
                  "该预览不启用 REAL CALIBRATION，不用于规划或真机执行。",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if answer != QMessageBox.Yes:
                return
            try:
                self.robot_view.set_workpiece_calibration_preview(result["T_base_workpiece_real"])
                self._refresh_scene_status_label()
                self._log(
                    "已显示 REAL CALIBRATION PREVIEW；结果未通过真实规划应用门，"
                    "规划/执行仍保持 BLOCKED"
                )
            except Exception as exc:
                self._log(f"显示真实工件预览失败：{exc}")
            return
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        self.calibration_source_combo.blockSignals(False)
        self._calibration_source_changed("REAL CALIBRATION")

    def _load_saved_calibration(self) -> bool:
        """Load a saved geometric result; applying it still needs P_CHECK and confirmation."""
        if not self.calibration_file.exists():
            return False
        try:
            payload = json.loads(self.calibration_file.read_text(encoding="utf-8"))
            if payload.get("frame_id", "base") not in ("base", "base_link"):
                self._log("读取工件标定失败：保存文件 frame_id 不是 base")
                return False
            position = np.asarray(payload["position"], dtype=float)
            rotation = np.asarray(payload["rotation_matrix"], dtype=float)
            residual_raw = payload.get("residual_mm")
            residual = None if residual_raw is None else float(residual_raw)
            if residual is not None and (not np.isfinite(residual) or residual < 0):
                self._log("读取工件标定失败：保存的残差无效")
                return False
            if position.shape != (3,) or rotation.shape != (3, 3):
                return False
            if not bool(payload.get("valid")) or not np.all(np.isfinite(position)) \
                    or not np.all(np.isfinite(rotation)):
                return False
            if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5)
                    or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-5)):
                self._log("读取工件标定失败：保存的旋转矩阵不是有效右手系")
                return False
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = rotation
            transform[:3, 3] = position
            self.calibration_result = {
                "T_base_workpiece_real": transform,
                "position": position,
                "rotation_matrix": rotation,
                "rpy": np.asarray(payload.get("rpy", self._rpy_zyx_from_rotation(rotation)), dtype=float),
                "residual_mm": residual,
                "valid": True,
                "geometry_valid": bool(payload.get("geometry_valid", True)),
                "quality_band": str(payload.get("quality_band", self._workpiece_quality_band(residual)[0])),
                "quality_text": str(payload.get("quality_text", self._workpiece_quality_band(residual)[1])),
                "review_status": str(payload.get("review_status", "DRAFT")),
                "source": "REAL CALIBRATION",
                "timestamp": str(payload.get("timestamp", "")),
            }
            points = payload.get("points", {})
            for key in self.calibration_points:
                value = points.get(key)
                self.calibration_points[key] = None if value is None else np.asarray(value, dtype=float)
            point_samples = payload.get("point_samples", {})
            for key in self.calibration_points:
                sample = point_samples.get(key, {}) if isinstance(point_samples, dict) else {}
                self.calibration_point_timestamps[key] = sample.get("timestamp")
                self.calibration_point_frames[key] = sample.get("frame_id")
            self.calibration_result_labels["position"].setText(
                "[%.6f, %.6f, %.6f]" % tuple(position)
            )
            self.calibration_result_labels["rpy"].setText(
                "[%.3f°, %.3f°, %.3f°]" % tuple(np.degrees(self.calibration_result["rpy"]))
            )
            self.calibration_result_labels["residual"].setText(
                "未读取 P_CHECK（可保存草稿）" if residual is None else f"{residual:.3f} mm"
            )
            quality_band, quality_text, quality_style = self._workpiece_quality_band(residual)
            self.calibration_result_labels["residual"].setStyleSheet(quality_style)
            self.calibration_result_labels["quality"].setText(
                f"{quality_band}：{quality_text}（仅初步提示，非官方精度标准）"
            )
            self.calibration_result_labels["quality"].setStyleSheet(quality_style)
            self.calibration_result_labels["check"].setText(
                "已从保存文件加载；P_CHECK 可选，应用前必须验证" if residual is None
                else "已从保存文件加载；P_CHECK 记录见 JSON"
            )
            self.calibration_matrix_label.setText(
                "\n".join("[" + ", ".join(f"{v:.6f}" for v in row) + "]" for row in rotation)
            )
            review = str(payload.get("review_status", "DRAFT"))
            self.calibration_result_labels["status"].setText(f"{review} | {quality_band}（已加载）")
            self.calibration_result_labels["status"].setStyleSheet(quality_style)
            self.calibration_save_button.setEnabled(True)
            self.calibration_display_button.setEnabled(bool(self.calibration_result.get("geometry_valid")))
            return True
        except Exception as exc:
            self._log(f"读取保存的工件标定失败：{exc}")
            return False

    def _calibration_source_changed(self, text: str) -> None:
        if text == "SIMULATION":
            self._restore_simulation_calibration(log=False)
            return
        result = self.calibration_result
        if not (result and result.get("geometry_valid", result.get("valid"))):
            self._load_saved_calibration()
            result = self.calibration_result
        reasons = self._workpiece_application_reasons(result)
        if result and not reasons:
            answer = QMessageBox.question(
                self, "确认应用真实工件位姿",
                f"P_CHECK 已通过应用精度检查（≤{self.calibration_residual_limit_mm:.1f} mm）。\n"
                "应用后规划场景、焊缝和相关轨迹将使用 REAL CALIBRATION；"
                "依赖旧工件位姿的轨迹必须重新规划并碰撞验证。\n\n确认应用？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if answer != QMessageBox.Yes:
                self.calibration_source_combo.blockSignals(True)
                self.calibration_source_combo.setCurrentText("SIMULATION")
                self.calibration_source_combo.blockSignals(False)
                return
            try:
                self.robot_view.set_workpiece_calibration(result["T_base_workpiece_real"])
                self._refresh_scene_status_label()
                self._mark_trajectory_stale("工件真实标定版本已改变")
                self._log("3D 工件显示已切换为 REAL CALIBRATION")
            except Exception as exc:
                self._log(f"切换真实工件位姿失败：{exc}")
        else:
            self.calibration_source_combo.blockSignals(True)
            self.calibration_source_combo.setCurrentText("SIMULATION")
            self.calibration_source_combo.blockSignals(False)
            self._log("REAL CALIBRATION 应用被阻止：" + "；".join(reasons or ["无有效结果"]))

    # ------------------------------------------------------------------
    # TCP pivot calibration (read-only flange FK)
    # ------------------------------------------------------------------
    def _flange_transform_from_q(self, q) -> np.ndarray:
        """Return calibrated-URDF base->flange FK for a six-joint snapshot."""
        values = np.asarray(q, dtype=float).reshape(-1)
        if values.size != 6 or not np.all(np.isfinite(values)):
            raise PivotCalibrationError("当前关节快照不是有限的 6 维向量")
        model = getattr(self, "tcp_fk_model", None)
        if model is None or list(getattr(model, "revolute_names", [])) != list(JOINT_NAMES):
            model = UrdfKinematicModel()
        frames = model.link_transforms_in_base(values)
        if "flange" not in frames:
            raise PivotCalibrationError("已标定 URDF 中没有 flange frame")
        return np.asarray(frames["flange"], dtype=float)

    def _existing_flange_tcp_offset(self) -> np.ndarray | None:
        """Read the frozen baseline only for comparison; never overwrite it."""
        path = ROOT.parent / "ur5_tool_calibration.mat"
        try:
            from scipy.io import loadmat
            data = loadmat(str(path))
            T_tool0_mount = np.asarray(data["T_tool0_mount"], dtype=float)
            T_mount_tcp = np.asarray(data["T_mount_tcp"], dtype=float)
            model = UrdfKinematicModel()
            frames = model.link_transforms_in_base(np.zeros(6, dtype=float))
            T_flange_tool0 = np.linalg.inv(frames["flange"]) @ frames["tool0"]
            return (T_flange_tool0 @ T_tool0_mount @ T_mount_tcp)[:3, 3]
        except Exception:
            return None

    @staticmethod
    def _tcp_quality_band(result) -> tuple[str, str, str]:
        """Return the simple HMI quality hint for an ordinary four-point fit.

        This is intentionally only an operator hint.  It is not the UR
        accuracy specification and is separate from the stricter real-robot
        write gate in ``TcpRobotPanel``.
        """
        rms = float(result.rms_residual_mm)
        if rms <= 2.0:
            return "GOOD", "RMS ≤ 2 mm（精度较好）", "color:#146c2e;font-weight:bold;"
        if rms <= 5.0:
            return "WARNING", "2–5 mm（误差偏大，建议重新标定）", "color:#8a6d00;font-weight:bold;"
        return "POOR", "> 5 mm（标定质量较差）", "color:#9b1c1c;font-weight:bold;"

    def _tcp_review_status(self, result=None) -> str:
        result = result or self.tcp_calibration_result
        if result is None or not result.valid:
            return "NOT_READY"
        # A numeric error list copied from an old JSON is not proof that a
        # new verification pose was actually measured.  Only freshly captured
        # or reproducibly loaded verification poses can promote DRAFT.
        if not self.tcp_verification_poses and not self.tcp_saved_verification_recomputed:
            return "DRAFT"
        errors = self._tcp_independent_errors()
        if errors and np.all(np.isfinite(errors)) and float(np.max(errors)) <= 2.0:
            return "VERIFIED"
        return "DRAFT"

    def _tcp_application_reasons(self, result=None) -> list[str]:
        """Hard checks for applying a fit to the software planning model.

        RMS remains a save-time quality hint, but a degenerate solve, a bad
        calibrated-URDF joint order, or a poorly conditioned pose set must
        never reach the planning model.
        """
        result = result or self.tcp_calibration_result
        reasons: list[str] = []
        if result is None or not result.valid:
            reasons.append("四点最小二乘结果不可用")
            return reasons
        if result.matrix_rank < 6:
            reasons.append("求解矩阵秩不足")
        if not np.isfinite(result.condition_number) or result.condition_number > 100.0:
            reasons.append("姿态数据退化（condition > 100）")
        if result.orientation_span_deg < 10.0:
            reasons.append("姿态跨度不足")
        if list(getattr(self.tcp_fk_model, "revolute_names", [])) != list(JOINT_NAMES):
            reasons.append("URDF 关节顺序/坐标模型不匹配")
        if not self.tcp_calibration_poses:
            reasons.append("缺少法兰姿态样本")
        else:
            for sample in self.tcp_calibration_poses:
                T = np.asarray(sample.get("T"), dtype=float)
                if T.shape != (4, 4) or not np.all(np.isfinite(T)):
                    reasons.append("法兰样本坐标无效")
                    break
        return reasons

    def _reset_tcp_calibration_ui(self) -> None:
        self.tcp_calibration_poses = []
        self.tcp_verification_poses = []
        self.tcp_calibration_result = None
        self.tcp_saved_verification_mm = []
        self.tcp_saved_verification_recomputed = False
        self.tcp_calibration_applied = False
        self.tcp_pose_table.setRowCount(0)
        self.tcp_sample_count_label.setText("0 / 6（4 组即可）")
        self.tcp_compute_status_label.setText("等待 4 组不同姿态")
        baseline = self._existing_flange_tcp_offset()
        self.tcp_result_labels["before"].setText(
            "[%.3f, %.3f, %.3f] mm（仿真 flange 坐标；不是 UR5 当前 TCP）" % tuple(baseline * 1000.0)
            if baseline is not None else "未能读取冻结基线（不影响采集）"
        )
        for key in ("offset", "pivot", "rank", "diversity", "residual", "quality", "independent"):
            self.tcp_result_labels[key].setText("—")
        self.tcp_result_labels["status"].setText("NOT READY")
        self.tcp_result_labels["status"].setStyleSheet("color:#8a6d00;font-weight:bold;")
        self.tcp_capture_button.setEnabled(True)
        self.tcp_compute_button.setEnabled(False)
        self.tcp_verify_capture_button.setEnabled(False)
        self.tcp_verify_capture_button.setText("可选：采集验证姿态（0）")
        self.tcp_verify_button.setEnabled(False)
        self.tcp_save_button.setEnabled(False)
        self.tcp_apply_button.setEnabled(False)
        self._update_robot_tcp_controls()

    def _render_tcp_pose_table(self) -> None:
        self.tcp_pose_table.setRowCount(len(self.tcp_calibration_poses))
        for row, sample in enumerate(self.tcp_calibration_poses):
            q_deg = ", ".join(f"{math.degrees(v):.2f}" for v in sample["q"])
            origin_mm = ", ".join(f"{v * 1000.0:.2f}" for v in sample["T"][:3, 3])
            values = [str(row + 1), sample["timestamp"], q_deg, origin_mm]
            for col, text in enumerate(values):
                self.tcp_pose_table.setItem(row, col, QTableWidgetItem(text))
        count = len(self.tcp_calibration_poses)
        self.tcp_sample_count_label.setText(f"{count} / 6（4 组即可）")
        self.tcp_compute_button.setEnabled(4 <= count <= 6)
        self.tcp_capture_button.setEnabled(count < 6)

    def _capture_tcp_pose(self) -> None:
        if self.robot_tcp_busy:
            return
        if len(self.tcp_calibration_poses) >= 6:
            self._log("TCP 标定采集已达到 6 组上限")
            return
        if not self._feedback_valid(self.state):
            self._log("TCP 标定采集拒绝：需要最新、上电且 RUNNING 的关节反馈")
            self.tcp_compute_status_label.setText("BLOCKED：没有有效实时关节反馈")
            return
        try:
            q = np.asarray(self.state.positions, dtype=float).copy()
            T = self._flange_transform_from_q(q)
            self.tcp_calibration_poses.append({"q": q, "T": T, "timestamp": time.strftime("%H:%M:%S")})
            self._render_tcp_pose_table()
            self.tcp_calibration_result = None
            self.tcp_verification_poses = []
            self.tcp_saved_verification_mm = []
            self.tcp_saved_verification_recomputed = False
            self.tcp_verify_capture_button.setText("可选：采集验证姿态（0）")
            for key in ("offset", "pivot", "rank", "diversity", "residual", "quality", "independent"):
                self.tcp_result_labels[key].setText("—")
            for button in (self.tcp_save_button, self.tcp_apply_button,
                           self.tcp_verify_capture_button, self.tcp_verify_button):
                button.setEnabled(False)
            self.tcp_result_labels["status"].setText("SAMPLES READY" if len(self.tcp_calibration_poses) >= 4 else "NOT READY")
            self._update_robot_tcp_controls()
            self._log(f"TCP 标定已采集姿态 {len(self.tcp_calibration_poses)}：使用 q_actual → flange FK")
        except Exception as exc:
            self._log(f"TCP 标定采集失败：{exc}")

    def _clear_tcp_calibration(self) -> None:
        if self.robot_tcp_busy:
            return
        self._reset_tcp_calibration_ui()
        self._log("TCP 标定姿态已清空；冻结 MATLAB/CoppeliaSim 标定未修改")

    def _compute_tcp_calibration(self) -> None:
        if self.robot_tcp_busy:
            return
        if len(self.tcp_calibration_poses) < 4:
            self._log("TCP 标定计算需要至少 4 组姿态")
            return
        try:
            result = fit_pivot_calibration([sample["T"] for sample in self.tcp_calibration_poses])
        except Exception as exc:
            self.tcp_calibration_result = None
            self.tcp_result_labels["status"].setText(f"INVALID：{exc}")
            self.tcp_result_labels["status"].setStyleSheet("color:#9b1c1c;font-weight:bold;")
            self.tcp_save_button.setEnabled(False)
            self.tcp_apply_button.setEnabled(False)
            self._log(f"TCP 标定计算失败：{exc}")
            return
        self.tcp_calibration_result = result
        self.tcp_result_labels["offset"].setText("[%.3f, %.3f, %.3f] mm" % tuple(result.tcp_flange_m * 1000.0))
        self.tcp_result_labels["pivot"].setText("[%.3f, %.3f, %.3f] mm" % tuple(result.pivot_base_m * 1000.0))
        self.tcp_result_labels["rank"].setText(f"{result.matrix_rank} / 6；condition={result.condition_number:.3g}")
        self.tcp_result_labels["diversity"].setText(f"{result.orientation_span_deg:.2f}°")
        self.tcp_result_labels["residual"].setText(f"RMS {result.rms_residual_mm:.3f} mm；最大 {result.max_residual_mm:.3f} mm")
        band, band_text, band_style = self._tcp_quality_band(result)
        application_reasons = self._tcp_application_reasons(result)
        self.tcp_result_labels["quality"].setText(
            f"{band}：{band_text}（仅初步提示，非 UR 官方精度标准）"
            + ("；应用禁止：" + "；".join(application_reasons) if application_reasons else "")
        )
        self.tcp_result_labels["quality"].setStyleSheet(band_style)
        self.tcp_result_labels["residual"].setStyleSheet(band_style)
        if result.valid:
            status = f"DRAFT | {band}（可保存；独立验证可选）"
            if application_reasons:
                status += " | APPLY BLOCKED"
            style = band_style
            self.tcp_compute_status_label.setText(
                "求解成功：可保存为草稿；独立验证可选；"
                + ("数据退化，禁止应用" if application_reasons else "应用前需要人工确认")
            )
        else:
            status = "INVALID（姿态/秩/残差不满足要求）"
            style = "color:#9b1c1c;font-weight:bold;"
            self.tcp_compute_status_label.setText("求解失败：请重新采集 4 组姿态")
        self.tcp_result_labels["status"].setText(status)
        self.tcp_result_labels["status"].setStyleSheet(style)
        self.tcp_verify_capture_button.setEnabled(True)
        # Keep this button operable after a successful fit. If no separate
        # verification pose has been captured yet, the handler explains the
        # required order instead of leaving an apparently dead control.
        # A numerically invalid/weak fit must still be independently
        # diagnosable.  Keep verification available whenever a result object
        # exists; save/apply remain gated by the stricter validity checks.
        self.tcp_verify_button.setEnabled(True)
        self.tcp_save_button.setEnabled(bool(result.valid))
        # Independent verification is optional for saving and for applying
        # the software/planning-model result.  Applying still opens an
        # explicit confirmation dialog; the separate real-robot writer keeps
        # its stricter quality, state and stationary gates.
        self.tcp_apply_button.setEnabled(not self._tcp_application_reasons(result))
        self._update_robot_tcp_controls()
        self._log(f"TCP 标定结果：offset={result.tcp_flange_m * 1000.0} mm，RMS={result.rms_residual_mm:.3f} mm，rank={result.matrix_rank}")

    def _capture_tcp_verification_pose(self) -> None:
        if self.robot_tcp_busy:
            return
        if self.tcp_calibration_result is None:
            self._log("请先计算 TCP 标定")
            return
        if not self._feedback_valid(self.state):
            self._log("独立验证采集拒绝：没有有效实时关节反馈")
            return
        try:
            q = np.asarray(self.state.positions, dtype=float).copy()
            if any(np.max(np.abs(q - sample["q"])) < 1e-4 for sample in self.tcp_calibration_poses):
                self._log("独立验证拒绝：需要不同于拟合样本的新姿态")
                return
            T = self._flange_transform_from_q(q)
            self.tcp_saved_verification_mm = []
            self.tcp_verification_poses.append({"q": q, "T": T, "timestamp": time.strftime("%H:%M:%S")})
            self.tcp_verify_capture_button.setText(
                f"可选：采集验证姿态（{len(self.tcp_verification_poses)}）"
            )
            self._verify_tcp_calibration()
            self._log(f"TCP 独立验证姿态已采集：第 {len(self.tcp_verification_poses)} 组")
        except Exception as exc:
            self._log(f"TCP 独立验证采集失败：{exc}")

    def _verify_tcp_calibration(self) -> None:
        result = self.tcp_calibration_result
        if result is None or not self.tcp_verification_poses:
            self.tcp_result_labels["independent"].setText("未验证（可选；不影响保存）")
            self.tcp_result_labels["independent"].setStyleSheet("color:#8a6d00;font-weight:bold;")
            self.tcp_compute_status_label.setText("独立验证未执行：这是可选步骤")
            if result is not None and not self._tcp_application_reasons(result):
                self.tcp_apply_button.setEnabled(True)
            self._log("独立验证尚未执行；普通四点标定仍可保存为 DRAFT")
            return
        errors = [verify_pivot_sample(sample["T"], result) * 1000.0 for sample in self.tcp_verification_poses]
        max_error = max(errors)
        rms = float(np.sqrt(np.mean(np.square(errors))))
        passed = max_error <= 2.0
        self.tcp_result_labels["independent"].setText(f"{'PASS' if passed else 'FAIL'}；RMS {rms:.3f} mm；最大 {max_error:.3f} mm（{len(errors)} 组）")
        self.tcp_result_labels["independent"].setStyleSheet("color:#146c2e;font-weight:bold;" if passed else "color:#9b1c1c;font-weight:bold;")
        self.tcp_verify_button.setEnabled(True)
        self.tcp_apply_button.setEnabled(not self._tcp_application_reasons(result))
        band, band_text, band_style = self._tcp_quality_band(result)
        review = "VERIFIED" if passed else "DRAFT"
        self.tcp_result_labels["status"].setText(f"{review} | {band}")
        self.tcp_result_labels["status"].setStyleSheet(band_style)
        self._update_robot_tcp_controls()

    def _tcp_payload(self, *, applied: bool = False) -> dict:
        result = self.tcp_calibration_result
        urdf_path = ROOT.parent / "real_robot_description_calibrated.urdf"
        band, band_text, _ = self._tcp_quality_band(result)
        review_status = self._tcp_review_status(result)
        return {
            "schema_version": 2,
            "calibration_type": "ordinary_four_point_tcp_position_only",
            "coordinate_frames": {"base": "URDF base", "flange": "URDF flange", "tcp": "real welding TCP position"},
            "position_flange_tcp_m": result.tcp_flange_m.tolist(),
            "position_flange_tcp_mm": (result.tcp_flange_m * 1000.0).tolist(),
            "pivot_base_m": result.pivot_base_m.tolist(),
            "orientation_source": "fresh controller tcp_offset on confirmed write; not solved by pivot calibration",
            "orientation_policy": "preserve current controller RX/RY/RZ; no orientation calibration",
            "fit": result.to_dict(),
            "independent_verification_mm": self._tcp_independent_errors(),
            "verification_samples": [{"timestamp": sample["timestamp"], "q_rad": sample["q"].tolist()} for sample in self.tcp_verification_poses],
            "pose_samples": [{"timestamp": sample["timestamp"], "q_rad": sample["q"].tolist()} for sample in self.tcp_calibration_poses],
            "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest() if urdf_path.exists() else None,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "version": "UR5-HMI-TCP-FOUR-POINT-1",
            "valid": bool(result.valid),
            "review_status": review_status,
            "quality_band": band,
            "quality_text": band_text,
            "quality_note": "HMI 初步质量提示，不是 UR 官方精度标准；独立验证为可选项",
            "independent_verification_optional": True,
            "applied_to_planning_model": bool(applied),
            "requires_replan": bool(applied),
            "requires_collision_revalidation": bool(applied),
        }

    def _save_tcp_calibration(self) -> None:
        result = self.tcp_calibration_result
        if result is None or not result.valid:
            self._log("保存 TCP 标定被拒绝：四点最小二乘结果不可用")
            return
        payload = self._tcp_payload()
        backup_path = None
        # Preserve the previously accepted software calibration before a new
        # draft replaces the canonical JSON.  The frozen MATLAB/CoppeliaSim
        # calibration is never touched; this small history copy also makes a
        # mistaken draft reversible without changing any robot parameter.
        if self.tcp_calibration_file.exists():
            history_dir = self.tcp_calibration_file.parent / "tcp_calibration_history"
            history_dir.mkdir(parents=True, exist_ok=True)
            backup_path = history_dir / (
                f"real_tcp_calibration_{time.strftime('%Y%m%d_%H%M%S')}.json"
            )
            suffix = 1
            while backup_path.exists():
                backup_path = history_dir / (
                    f"real_tcp_calibration_{time.strftime('%Y%m%d_%H%M%S')}_{suffix}.json"
                )
                suffix += 1
            shutil.copy2(self.tcp_calibration_file, backup_path)
        self.tcp_calibration_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._log(
            f"TCP 标定已保存为 {payload['review_status']}：{self.tcp_calibration_file}；"
            f"质量提示={payload['quality_band']}，独立验证可选"
            + (f"；旧文件已归档到 {backup_path}" if backup_path else "")
        )

    def _apply_tcp_calibration(self) -> None:
        if self.robot_tcp_busy or self._motion_busy() or self.planning_busy:
            self._log("软件 TCP 应用被阻止：参数/运动/规划任务正在运行")
            return
        result = self.tcp_calibration_result
        application_reasons = self._tcp_application_reasons(result)
        if application_reasons:
            self._log("应用 TCP 标定被拒绝：" + "；".join(application_reasons))
            return
        band, band_text, _ = self._tcp_quality_band(result)
        review = self._tcp_review_status(result)
        independent = self.tcp_result_labels["independent"].text()
        answer = QMessageBox.question(
            self, "二次确认：应用 TCP 标定到软件规划模型",
            f"当前结果：{review} / {band}\n{band_text}\n"
            f"独立验证：{independent}\n\n"
            "这一步只更新 HMI 与软件规划模型，不修改 UR5 示教器 TCP。\n"
            "应用后旧轨迹会失效，必须重新规划、做完整碰撞验证，并由人工确认焊接精度。\n"
            "黄色/红色或未独立验证的结果仍可保存，但不应直接用于正式焊接。\n\n"
            "确认继续？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return
        payload = self._tcp_payload(applied=True)
        # Keep the canonical explicit MATLAB input in sync with the HMI only
        # after this manual confirmation.  The frozen .mat simulation
        # calibration remains untouched.
        self.tcp_calibration_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self.tcp_calibration_applied_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self.tcp_calibration_applied = True
        if hasattr(self.robot_view, "set_calibrated_tcp_offset"):
            self.robot_view.set_calibrated_tcp_offset(result.tcp_flange_m)
        self._stop_preview()
        # Keep the old path available for audit/preview, but mark it STALE so
        # a TCP change can never be mistaken for a valid executable path.
        self._mark_trajectory_stale("TCP 标定版本已改变")
        self.trajectory_validation = None
        self.loaded_path_name = "TCP 标定已更新：需重新规划"
        if hasattr(self, "path_label"):
            self.path_label.setText("TCP 标定已更新，旧轨迹 STALE（需重新规划）")
        if hasattr(self, "preview_status_label"):
            self.preview_status_label.setText("需重新规划与碰撞验证")
        if hasattr(self.robot_view, "clear_planned_tcp_path"):
            self.robot_view.clear_planned_tcp_path()
        self._update_trajectory_gate()
        self._update_buttons()
        self._log("TCP 标定已用于 HMI 对比显示并导出；未写入 UR5，未自动更改 MATLAB 规划器。冻结标定未修改，旧轨迹已失效。")

    def _save_home(self):
        if self._motion_busy():
            self._log("RealHome blocked: motion busy (see Motion State / Current Goal)")
            return
        if len(self.state.positions) != 6:
            return
        self.config["real_home"] = [float(v) for v in self.state.positions]
        CONFIG.write_text(yaml.safe_dump(self.config, sort_keys=False, allow_unicode=True))
        self.home_info.setText(
            "已保存 RealHome：\n" +
            "  ".join(
                f"J{i + 1}={math.degrees(value):.3f}°"
                for i, value in enumerate(self.state.positions)
            )
        )
        self._log(f"RealHome saved to {CONFIG}")

    def _open_rviz(self):
        rviz_config = ROOT / "config" / "ur5_hmi.rviz"
        try:
            subprocess.Popen(
                ["rviz2", "-d", str(rviz_config)],
                start_new_session=True,
            )
            self._log(f"RViz started with {rviz_config}")
        except Exception as exc:
            QMessageBox.critical(self, "RViz 启动失败", str(exc))
            self._log(f"RViz start failed: {exc}")

    def _log(self, text: str):
        self.log_view.appendPlainText(time.strftime("%H:%M:%S ") + text)

    def closeEvent(self, event):
        if getattr(self, "robot_tcp_busy", False):
            if self.robot_tcp_mode == "apply" or (self.robot_tcp_mode == "build" and self._robot_tcp_next[0] == "apply"):
                self._log("正在完成 TCP 写入/读回，请等待结果后关闭界面")
                event.ignore()
                return
            self.robot_tcp_process.terminate()
            self.robot_tcp_process.waitForFinished(1000)
        self._stop_preview()
        if self.planning_process is not None:
            self.planning_process.terminate()
            if not self.planning_process.waitForFinished(1000):
                self.planning_process.kill()
        self.worker.quit_requested.emit()
        self.thread.wait(3000)
        event.accept()
