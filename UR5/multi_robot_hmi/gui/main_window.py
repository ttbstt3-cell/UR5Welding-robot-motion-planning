from __future__ import annotations

import math
import os
import csv
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
from gui.robot_profile import RobotProfile, check_trajectory_robot_id, load_robot_profiles
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
from ros.ros_node import UR5RosNode
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


class Cr7SimulationRosWorker(QObject):
    """Read-only CR7 simulation JointState monitor; deliberately has no action client."""

    state = pyqtSignal(object)
    log = pyqtSignal(str)
    quit_requested = pyqtSignal()
    finished = pyqtSignal()

    def __init__(self, joint_names, topic):
        super().__init__()
        self.joint_names = list(joint_names)
        self.topic = str(topic)
        self.node = None
        self.quit_requested.connect(self._quit)

    @pyqtSlot()
    def run(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        rclpy.init()
        outer = self

        class StateOnlyNode(Node):
            def __init__(self):
                super().__init__("cr7_hmi_read_only_state")
                self.create_subscription(JointState, outer.topic, self.joint_cb, 10)

            def joint_cb(self, msg):
                values = dict(zip(msg.name, msg.position))
                if not all(name in values for name in outer.joint_names):
                    return
                state = RobotState(
                    joint_names=list(outer.joint_names),
                    positions=[float(values[name]) for name in outer.joint_names],
                    joint_stamp=time.monotonic(),
                )
                outer.state.emit(state)

        self.node = StateOnlyNode()
        self.log.emit(f"CR7 read-only ROS monitor subscribed: {self.topic}; no action client")
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
        import rclpy
        if hasattr(self, "spin_timer"):
            self.spin_timer.stop()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()
        self.finished.emit()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Robot Welding Planning & Control HMI")
        self.setMinimumSize(1440, 850)
        self.resize(1600, 1000)
        self.state = RobotState()
        self.goal_active = False
        self.robot_profiles = load_robot_profiles()
        self.current_profile: RobotProfile = self.robot_profiles["UR5"]
        self.robot_switch_block_reason = ""
        self.trajectory_block_reason = ""
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
        self.calibration_capture_buttons = []
        self.calibration_residual_limit_mm = 2.0
        self.config = self._load_config()
        self.calibration_file = self.current_profile.workpiece_pose_file
        self.ui_scale = float(self.config.get("ui", {}).get("scale", 1.0))
        self.metrics = UiMetrics(self.ui_scale)
        jog_config = self.config.get("jog", {})
        self.max_hmi_speed_percent = min(20, max(1, int(jog_config.get("max_hmi_speed_percent", 20))))
        trajectory_config = self.config.get("trajectory", {})
        self.start_tolerance_rad = math.radians(float(trajectory_config.get("start_tolerance_deg", 1.0)))
        self._build_ui()
        self._apply_robot_profile(initial=True)
        self._start_ros_worker()
        self._log("界面启动：不会自动发送运动")
        self.showMaximized()

    def _load_config(self):
        try:
            return yaml.safe_load(CONFIG.read_text()) or {}
        except Exception:
            return {}

    def _start_ros_worker(self) -> None:
        """Start the UR5 client or CR7 read-only topic monitor for its profile."""
        if self.current_profile.supports_ros_control:
            ros = self.current_profile.ros
            robot_ip = str(ros.get("robot_ip") or os.environ.get("UR5_ROBOT_IP", "192.168.0.10"))
            self.worker = RosWorker(self.current_profile.controller_action, robot_ip)
        elif self.current_profile.robot_id == "CR7":
            topic = self.current_profile.ros.get("joint_states_topic", "/cr7/joint_states")
            self.worker = Cr7SimulationRosWorker(self.current_profile.joint_names, topic)
        else:
            self.worker = None
            self.thread = None
            self.trajectory_client = None
            return
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.state.connect(self._on_state)
        self.worker.log.connect(self._log)
        if self.current_profile.supports_ros_control:
            self.worker.goal_active.connect(self._on_goal_active)
            self.worker.goal_done.connect(self._on_goal_done)
            self.worker.motion.connect(self._on_motion)
            self.worker.client_ready.connect(self._on_client_ready)
        self.worker.finished.connect(self.thread.quit)
        self.thread.start()

    def _stop_ros_worker(self) -> bool:
        worker = getattr(self, "worker", None)
        thread = getattr(self, "thread", None)
        if worker is None or thread is None:
            self.worker = None
            self.thread = None
            self.trajectory_client = None
            return True
        worker.quit_requested.emit()
        stopped = bool(thread.wait(3000))
        if stopped:
            self.worker = None
            self.thread = None
            self.trajectory_client = None
        return stopped

    def _apply_robot_profile(self, initial=False) -> None:
        profile = self.current_profile
        self.setWindowTitle(f"Multi-Robot Welding Planning & Control HMI — {profile.display_name}")
        self.calibration_file = profile.workpiece_pose_file
        self._description_loaded = False
        if hasattr(self, "robot_view"):
            self.robot_view.set_robot_profile(
                profile.robot_description, profile.joint_names, profile.scene_geometry_file
            )
        if hasattr(self, "model_box"):
            self.model_box.setTitle(f"{profile.display_name} 3D 模型")
        if hasattr(self, "joint_table"):
            for index, joint_name in enumerate(profile.joint_names):
                label = profile.joint_display_names[index]
                row_item = self.joint_table.item(index, 0)
                row_item.setText(f"J{index + 1}  {label}")
                lo, hi = profile.joint_limits[index]
                row_item.setToolTip(f"{joint_name}\nsoftware limits: [{lo:.4f}, {hi:.4f}] rad")
                self.joint_table.item(index, 1).setText("—")
                self.joint_table.item(index, 2).setText("—")
            for index in range(len(profile.joint_names), self.joint_table.rowCount()):
                self.joint_table.item(index, 0).setText("—")
                self.jog_widgets[index].setEnabled(False)
        self.state = RobotState(joint_names=list(profile.joint_names), positions=[])
        self.goal_active = False
        self._jog_pending = False
        self.motion_state_text = MotionState.IDLE.value
        self.motion_goal_label = ""
        self.trajectory_client = None
        self.loaded_path = None
        self.loaded_trajectory = None
        self.trajectory_validation = None
        self.loaded_path_name = "未加载"
        self.trajectory_block_reason = ""
        self.execution_trajectory = None
        self.execution_started_monotonic = None
        self.execution_log = []
        if hasattr(self, "path_label"):
            self.path_label.setText("未加载轨迹")
            self.preview_slider.setRange(0, 0)
            self.preview_status_label.setText("预演已停止")
        if hasattr(self, "calibration_result_labels"):
            for label in self.calibration_result_labels.values():
                label.setText("—")
            self.calibration_matrix_label.setText("—")
        self.calibration_points = {key: None for key in ("P0", "PX", "PY", "P_CHECK")}
        self.calibration_result = None
        if hasattr(self, "calibration_source_combo"):
            self.calibration_source_combo.blockSignals(True)
            self.calibration_source_combo.setCurrentText("SIMULATION")
            self.calibration_source_combo.blockSignals(False)
            has_scene = profile.scene_geometry_file is not None and profile.scene_geometry_file.is_file()
            for widget in (self.calibration_source_combo, self.calibration_compute_button,
                           self.calibration_save_button, self.calibration_reset_button,
                           self.calibration_restore_button):
                widget.setEnabled(has_scene and profile.supports_real_robot)
            for widget in self.calibration_capture_buttons:
                widget.setEnabled(has_scene and profile.supports_real_robot)
            self.calibration_note.setText(
                f"{profile.display_name}: 工件标定文件 {profile.workpiece_pose_file}；"
                + ("三点 TCP 示教标定。" if has_scene else "本 profile 暂无可校准的 3D 工件场景。")
            )
            for key, label in self.calibration_point_labels.items():
                label.setText("X: —   Y: —   Z: — mm")
        if hasattr(self, "calibration_scene_source_label"):
            scene_path = profile.scene_geometry_file or profile.collision_scene_file
            self.calibration_scene_source_label.setText(
                f"{profile.display_name} 场景：{scene_path or '未配置'}"
            )
        if hasattr(self, "robot_profile_status"):
            ros = profile.ros
            action = profile.controller_action or "未配置"
            self.robot_profile_status.setText(
                f"ROS {ros.get('namespace', '')} · {ros.get('joint_states_topic', '未配置')}  |  "
                f"Action: {action}  |  {'REAL CONTROL CONFIGURED' if profile.supports_ros_control else 'SIMULATION / NO REAL CONTROL'}"
            )
            self.robot_profile_status.setToolTip(
                f"Planning model: {profile.planning_model}\nTCP calibration: {profile.tcp_calibration}\n"
                f"Workpiece calibration: {profile.workpiece_pose_file}\n"
                f"Logs: {profile.log_dir}"
            )
        if hasattr(self, "online_start_combo"):
            topic = profile.ros.get("joint_states_topic", "未配置")
            self.online_start_combo.setItemText(0, f"Current Robot（{topic}）")
            self.online_start_combo.setItemText(1, f"Saved RealHome（{profile.real_home.name}）")
        if hasattr(self, "scene_visibility_boxes"):
            scene_available = profile.scene_geometry_file is not None
            for key, box in self.scene_visibility_boxes.items():
                enabled = key == "robot" or (scene_available and key != "tcp_path")
                box.setEnabled(enabled)
                box.setChecked(key == "robot" or (scene_available and key in ("workpiece", "seam", "collision", "current_tcp")))
            self._on_scene_visibility_changed()
        if hasattr(self, "online_status_label"):
            if profile.supports_online_planning:
                self.online_status_label.setText("未开始")
                self._set_online_stage("IDLE")
            else:
                self.online_status_label.setText(
                    f"{profile.display_name}: 离线模型已选择；HMI 焊接规划接口尚未验收。"
                )
                self._set_online_stage("UNAVAILABLE")
            if hasattr(self, "online_output_view"):
                self.online_output_view.clear()
                self.online_output_view.appendPlainText(f"robot_id: {profile.robot_id}")
                self.online_output_view.appendPlainText(f"planning model: {profile.planning_model}")
        if not initial:
            self._log(f"Robot profile loaded: {profile.robot_id}; no motion command sent")
        self._refresh_scene_status_label()
        if hasattr(self, "home_info"):
            home = self._saved_real_home()
            self.home_info.setText(
                f"{profile.display_name} RealHome: " +
                (self._format_joint_row(home) if home is not None else "尚未标定 / 保存")
            )
        if hasattr(self, "final_gate_label"):
            self._update_trajectory_gate()
            self._update_buttons()

    def _on_robot_selected(self, index: int) -> None:
        robot_id = self.robot_select_combo.itemData(index)
        if not robot_id or robot_id == self.current_profile.robot_id:
            return
        if self._motion_busy() or self.planning_busy or self.goal_active or self.state.goal_active:
            self.robot_switch_block_reason = "机器人切换被阻止：motion_busy / planning_busy / goal_active"
            self._log(self.robot_switch_block_reason)
            self.robot_select_combo.blockSignals(True)
            self.robot_select_combo.setCurrentIndex(
                self.robot_select_combo.findData(self.current_profile.robot_id)
            )
            self.robot_select_combo.blockSignals(False)
            return
        self._stop_preview()
        self.preview_timer.stop()
        if not self._stop_ros_worker():
            self._log("机器人切换被阻止：ROS monitor thread 未能安全退出")
            self.robot_select_combo.blockSignals(True)
            self.robot_select_combo.setCurrentIndex(
                self.robot_select_combo.findData(self.current_profile.robot_id)
            )
            self.robot_select_combo.blockSignals(False)
            return
        profile = self.robot_profiles.get(str(robot_id))
        if profile is None:
            return
        self.current_profile = profile
        self._apply_robot_profile()
        self._start_ros_worker()
        self._update_online_start_display()
        self._update_status_widgets()
        self._update_trajectory_gate()
        self._update_buttons()

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

        device_bar = QHBoxLayout()
        device_bar.addStretch(1)
        device_bar.addWidget(QLabel("当前设备："))
        self.robot_select_combo = QComboBox()
        for robot_id in ("UR5", "CR7"):
            profile = self.robot_profiles.get(robot_id)
            if profile is not None:
                self.robot_select_combo.addItem(profile.display_name, robot_id)
        self.robot_select_combo.setCurrentIndex(self.robot_select_combo.findData(self.current_profile.robot_id))
        self.robot_select_combo.currentIndexChanged.connect(self._on_robot_selected)
        self.robot_select_combo.setMinimumWidth(self._u(150))
        self.robot_select_combo.setMinimumHeight(self._u(38))
        device_bar.addWidget(self.robot_select_combo)
        self.robot_profile_status = QLabel("")
        self.robot_profile_status.setMinimumWidth(self._u(650))
        self.robot_profile_status.setWordWrap(True)
        self.robot_profile_status.setStyleSheet("color:#475569;padding:4px 8px;")
        device_bar.addWidget(self.robot_profile_status)
        root.addLayout(device_bar)

        # Main layout: the complete left half is the 3D scene and the
        # complete right half is safety + functional tabs.  This splitter is
        # deliberately horizontal and user-adjustable; no vertical splitter
        # is used to squeeze the model when a tab is selected.
        top_splitter = QSplitter(Qt.Horizontal)
        self.top_splitter = top_splitter
        self.main_splitter = top_splitter
        top_splitter.setChildrenCollapsible(False)
        top_splitter.setMinimumHeight(self._u(620))
        model_box = QGroupBox(f"{self.current_profile.display_name} 3D 模型")
        self.model_box = model_box
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
        # all six switches on a single row below the view buttons.  The
        # left pane is deliberately wide (about 70%), so the labels remain
        # readable without consuming VTK height or overlapping the buttons.
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
            ("collision", "Collision Geometry"),
        ]):
            box = QCheckBox(text)
            box.setMinimumHeight(self._u(24))
            box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            box.setChecked(key in ("robot", "workpiece", "seam", "current_tcp", "collision"))
            box.setToolTip(f"显示/隐藏 {text}（仅显示，不影响控制）")
            box.stateChanged.connect(self._on_scene_visibility_changed)
            scene_toggles.addWidget(box, 0, index)
            self.scene_visibility_boxes[key] = box
        for col in range(6):
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
        self.refresh_button.clicked.connect(self._refresh_ros_state)
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
        for i, _name in enumerate(self.current_profile.joint_names):
            joint_label = self.current_profile.joint_display_names[i]
            self.joint_table.setItem(i, 0, QTableWidgetItem(f"J{i + 1}  {joint_label}"))
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
        self.online_algorithm_combo.addItem("CBi-RRT*", "V1 FastConnectAnytimeRRTStar")
        configured_algorithm = str(self.config.get("online_planning", {}).get(
            "default_algorithm", "V1 FastConnectAnytimeRRTStar"))
        match = self.online_algorithm_combo.findData(configured_algorithm)
        self.online_algorithm_combo.setCurrentIndex(match if match >= 0 else 3)
        self.online_algorithm_combo.setToolTip("自由空间段使用 CBi-RRT*；焊缝段固定使用 ACCP。")
        setup_grid.addWidget(self.online_algorithm_combo, 2, 1)
        self.online_field_widgets["weld_algorithm_label"] = QLabel("焊缝算法")
        setup_grid.addWidget(self.online_field_widgets["weld_algorithm_label"], 2, 2)
        self.online_weld_algorithm_label = QLabel("ACCP（固定）")
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
            ("seam_result", "ACCP"),
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
        self.calibration_scene_source_label = source_label
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
            self.calibration_capture_buttons.append(button)
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
        add_calibration_point_row(3, "P_CHECK", "P_CHECK 验证点", "默认重复测量 P0，检查残差")
        calibration_layout.addWidget(points_box)

        action_box = QGroupBox("3. 计算与保存")
        action_layout = QHBoxLayout(action_box)
        action_layout.setContentsMargins(self._u(12), self._u(8), self._u(12), self._u(8))
        action_layout.setSpacing(self._u(8))
        self.calibration_compute_button = QPushButton("计算标定")
        self.calibration_save_button = QPushButton("保存标定")
        self.calibration_reset_button = QPushButton("重新标定")
        self.calibration_restore_button = QPushButton("恢复仿真位姿")
        for button in (self.calibration_compute_button, self.calibration_save_button,
                       self.calibration_reset_button, self.calibration_restore_button):
            button.setMinimumHeight(self._u(40))
        self.calibration_compute_button.clicked.connect(self._calculate_calibration)
        self.calibration_save_button.clicked.connect(self._save_calibration)
        self.calibration_reset_button.clicked.connect(self._reset_calibration)
        self.calibration_restore_button.clicked.connect(self._restore_simulation_calibration)
        self.calibration_save_button.setEnabled(False)
        action_layout.addWidget(self.calibration_compute_button)
        action_layout.addWidget(self.calibration_save_button)
        action_layout.addWidget(self.calibration_reset_button)
        action_layout.addWidget(self.calibration_restore_button)
        action_layout.addStretch(1)
        calibration_layout.addWidget(action_box)

        result_box = QGroupBox("4. 标定结果")
        result_grid = QGridLayout(result_box)
        result_grid.setContentsMargins(self._u(12), self._u(10), self._u(12), self._u(10))
        result_grid.setHorizontalSpacing(self._u(12))
        result_grid.setVerticalSpacing(self._u(5))
        self.calibration_result_labels = {}
        result_items = [
            ("position", "Position (m)"), ("rpy", "RPY (ZYX, deg)"),
            ("residual", "Residual"), ("status", "Calibration Status"),
            ("check", "P_CHECK 预测 / 实测"),
        ]
        for row, (key, title) in enumerate(result_items):
            result_grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            result_grid.addWidget(value, row, 1, 1, 3)
            self.calibration_result_labels[key] = value
        result_grid.addWidget(QLabel("Rotation Matrix"), 5, 0, Qt.AlignTop)
        self.calibration_matrix_label = QLabel("—")
        self.calibration_matrix_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.calibration_matrix_label.setStyleSheet("font-family:monospace;background:#f3f4f6;padding:6px;")
        self.calibration_matrix_label.setWordWrap(True)
        result_grid.addWidget(self.calibration_matrix_label, 5, 1, 1, 3)
        result_grid.setColumnStretch(1, 1)
        result_grid.setColumnStretch(3, 1)
        calibration_layout.addWidget(result_box)

        self.calibration_note = QLabel(
            "只读取实时 TCP，不自动移动机器人。P_CHECK 默认按重复测量 P0 计算残差；" 
            f"残差阈值为 {self.calibration_residual_limit_mm:.1f} mm。保存文件：{self.calibration_file}"
        )
        self.calibration_note.setWordWrap(True)
        self.calibration_note.setStyleSheet("color:#6b7280;")
        calibration_layout.addWidget(self.calibration_note)
        calibration_layout.addStretch(1)
        tabs.addTab(calibration_scroll, "工件标定")

        # Tab 5: RealHome information and save action.
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
        if not self.current_profile.supports_ros_control:
            return
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
        if not self.current_profile.supports_ros_control:
            if self.current_profile.robot_id != "CR7":
                return
            self.state = state
            self._update_online_start_display()
            if len(state.positions) == len(self.current_profile.joint_names) \
                    and time.monotonic() - state.joint_stamp < 1.0:
                self.robot_view.set_preview_state(state.positions)
            else:
                self.robot_view.set_preview_state([0.0] * len(self.current_profile.joint_names))
            self._update_status_widgets()
            self._update_trajectory_gate()
            self._update_buttons()
            return
        self.state = state
        self._update_online_start_display()
        if state.robot_description and not self._description_loaded:
            self.robot_view.set_urdf_text(state.robot_description)
            self._description_loaded = True
        if self.preview_active:
            # Keep the preview pose on screen while still updating all
            # read-only status widgets from ROS2.
            pass
        elif self._feedback_valid(state):
            self.robot_view.set_state(state.positions, state.tcp_position, state.tcp_frame_id)
        else:
            # Keep the reference/frozen robot mesh visible, but do not present
            # the last powered-on pose or TCP as live data.
            self.robot_view.set_state(None, None, "")
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
            "robot_id": self.current_profile.robot_id,
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
        log_dir = self.current_profile.log_dir
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
        simulation_feedback = (
            self.current_profile.robot_id == "CR7"
            and fresh
            and len(self.state.positions) == len(self.current_profile.joint_names)
        )
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
            if feedback_valid or simulation_feedback:
                self.joint_table.item(i, 1).setText(f"{math.degrees(q):.3f}°")
                self.joint_table.item(i, 2).setText(f"{math.degrees(q):.3f}°")
                self.joint_table.item(i, 1).setToolTip(f"{q:.6f} rad")
                self.joint_table.item(i, 2).setToolTip(
                    f"{q:.6f} rad" if feedback_valid else
                    "Simulation /cr7/joint_states only; not real robot feedback"
                )
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
        if not self.current_profile.supports_ros_control:
            status("driver", "Simulation topic updating" if fresh else "No simulation JointState", "ok" if fresh else "warn")
            status("program", "N/A (simulation)", "unknown")
            status("mode", "SIMULATION", "unknown")
            status("safety", "NOT CONFIGURED", "warn")
            status("speed", "—", "unknown")
            status("controller", "NOT CONFIGURED", "warn")
            status("joint", "simulation updating" if fresh else "stale", "ok" if fresh else "warn")

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
            self.current_profile.supports_ros_control
            and check_motion_ready(self.state).ok
            and not self.goal_active
            and not self._motion_busy()
        )

    def _update_buttons(self):
        enabled = self._motion_ready() and self.current_profile.supports_real_robot
        for jog in self.jog_widgets:
            jog.set_enabled(enabled)
        # `trajectory_client.busy` blocks new motion and new work: it covers
        # SENDING, EXECUTING and CANCELING, i.e. every moment in which a goal
        # exists but its outcome is not yet known.
        blocked = self._motion_busy()
        if hasattr(self, "load_button"):
            self.load_button.setEnabled(not blocked and not self.planning_busy)
        if hasattr(self, "validate_button"):
            self.validate_button.setEnabled(not blocked and not self.planning_busy and self.loaded_trajectory is not None)
        self.execute_button.setEnabled(
            self.current_profile.supports_ros_control
            and not blocked and self._final_execution_gate_ok()
        )
        if hasattr(self, "online_start_button"):
            self.online_start_button.setEnabled(
                self.current_profile.supports_online_planning
                and not blocked and not self.planning_busy
            )
        if hasattr(self, "online_cancel_button"):
            self.online_cancel_button.setEnabled(self.planning_busy)
        # STOP MOTION is never disabled by the busy gate.
        self.stop_button.setEnabled(blocked or self.goal_active)
        self.home_button.setEnabled(
            self.current_profile.supports_real_robot
            and self._feedback_valid(self.state) and not blocked
        )
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
        if not self.current_profile.supports_ros_control or self.worker is None:
            self._log(f"Jog BLOCKED: {self.current_profile.display_name} has no configured HMI motion controller")
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
        if self.worker is None:
            self._log("STOP MOTION unavailable: no motion controller is configured for this profile")
            return
        self.worker.request_stop.emit()
        self._log("STOP MOTION: cancel requested; this is not a hardware emergency stop")

    def _refresh_ros_state(self):
        worker = getattr(self, "worker", None)
        if worker is not None and hasattr(worker, "request_refresh"):
            worker.request_refresh.emit()
        elif worker is not None:
            self._log("CR7 state refresh is read-only and updates from /cr7/joint_states")
        else:
            self._log("No ROS monitor is configured for the selected profile")

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
        if not self.current_profile.supports_ros_control:
            reasons.append(f"{self.current_profile.display_name} profile has no configured real motion controller")
        if self.trajectory_block_reason:
            reasons.append(self.trajectory_block_reason)
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
        try:
            check_trajectory_robot_id(traj, self.current_profile.robot_id)
        except ValueError as exc:
            reasons.append(str(exc))
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
            joint_names=self.current_profile.joint_names,
            joint_limits=self.current_profile.joint_limits,
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
            text = self.trajectory_block_reason or "未加载轨迹"
            self.final_gate_label.setText(f"FINAL EXECUTION CHECK：BLOCKED（{text}）")
            self.final_gate_label.setStyleSheet("background:#fff2cc;color:#8a5a00;font-weight:bold;padding:8px;")
            self.final_gate_details_label.setText(text)
            return
        current_report = validate_trajectory(
            self.loaded_trajectory,
            q_actual=self.state.positions if self._feedback_valid(self.state) else None,
            joint_names=self.current_profile.joint_names,
            joint_limits=self.current_profile.joint_limits,
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
        if not self.current_profile.supports_online_planning:
            return self.current_profile.planning_runs_dir / "report.json"
        configured = self.config.get("online_planning", {}).get("v1_report_file", "")
        if configured:
            return Path(str(configured)).expanduser()
        return ROOT.parent / "轨迹文件" / "realhome_to_preweld_v1_report.json"

    def _online_task(self) -> str:
        if not hasattr(self, "online_task_combo"):
            return "Weld_Seam"
        return str(self.online_task_combo.currentData() or "Weld_Seam")

    def _weld_anchor_path(self) -> Path:
        if self.current_profile.matlab_anchor_file is not None:
            return self.current_profile.matlab_anchor_file
        if not self.current_profile.supports_online_planning:
            return self.current_profile.planning_runs_dir / "anchors.json"
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
                "Full_Welding_Task": "READY · Approach CBi-RRT* / Weld ACCP / Exit CBi-RRT*",
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
        """Read only the active profile's isolated RealHome file."""
        try:
            data = yaml.safe_load(self.current_profile.real_home.read_text(encoding="utf-8")) or {}
            if str(data.get("robot_id", "")) != self.current_profile.robot_id:
                return None
            q = np.asarray(data.get("positions_rad", []), dtype=float).reshape(-1)
            if q.size != len(self.current_profile.joint_names) or not np.isfinite(q).all():
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
                f"等待新鲜 {self.current_profile.ros.get('joint_states_topic', '/joint_states')}"
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
        "WELD": "PLANNING WELD（ACCP 焊缝）",
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
            self.online_status_label.setText("Forward 焊缝失败，正在尝试 Reverse（ACCP）")
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
        algorithm_text = str(payload.get("algorithm", "—"))
        if "V1 FastConnectAnytimeRRTStar" in algorithm_text or "CBi-RRT*" in algorithm_text:
            algorithm_text = "CBi-RRT*"
        labels["algorithm"].setText(algorithm_text)
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
            seam_planner_text = str(payload.get("seam_planner", "ACCP"))
            if "A-ARC-IK" in seam_planner_text or "ARC-IK" in seam_planner_text:
                seam_planner_text = "ACCP"
            labels["seam_result"].setText(
                f"{seam_planner_text[:44]}\n"
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

        archive_dir = self.current_profile.trajectory_dir
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
            "CBi-RRT*": "cbi_rrt_star",
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
        if not self.current_profile.supports_online_planning:
            message = f"{self.current_profile.display_name} 的在线焊接规划接口尚未验收；请求已阻止。"
            self.online_status_label.setText(message)
            self._log(message)
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
        run_dir = self.current_profile.planning_runs_dir / f"hmi_{request_id}"
        run_dir.mkdir(parents=True, exist_ok=True)
        output_path = run_dir / "trajectory.mat"
        report_path = run_dir / "report.json"
        request_path = run_dir / "request.json"
        max_time = float(self.online_budget_spin.value())

        # The weld wrapper handles all four tasks.  Free-space segments use the
        # selected comparison algorithm; the weld segment is always ACCP
        # with branchReferenceQ and the Forward -> Reverse cascade.
        request_payload = {
            "robot_id": self.current_profile.robot_id,
            "request_id": request_id,
            "task": task,
            "qStart": np.asarray(q_start, dtype=float).tolist(),
            "qGoal": np.asarray(q_goal, dtype=float).tolist(),
            "algorithm": algorithm,
            "seed": int(self.online_seed_spin.value()),
            "maxPlanningTimeSec": max_time,
            "start_source": str(self.online_start_combo.currentData()),
            "endpoint_source": endpoints_text,
            "weld_algorithm": "ACCP (replan_weld_actual_axis_adaptive_feasibility.m)",
        }
        request_path.write_text(
            json.dumps(request_payload, ensure_ascii=False, indent=1), encoding="utf-8"
        )

        matlab_dir = ROOT / "matlab"
        benchmark_dir = self.current_profile.matlab_planning_entry.parent
        wrapper = matlab_dir / "plan_hmi_robot_request.m"
        if not wrapper.is_file():
            self.online_status_label.setText(f"找不到 MATLAB 入口：{wrapper}")
            self._set_online_stage("FAILED")
            return
        expression = (
            f"addpath({self._matlab_quote(benchmark_dir)}); "
            f"addpath({self._matlab_quote(matlab_dir)}); "
            f"addpath(genpath({self._matlab_quote(self.current_profile.matlab_model_root)})); "
            f"req=jsondecode(fileread({self._matlab_quote(request_path)})); "
            f"req.outputFile={self._matlab_quote(output_path)}; "
            f"req.reportFile={self._matlab_quote(report_path)}; "
            f"req.requestFile={self._matlab_quote(request_path)}; "
            f"req.requestId='{request_id}'; "
            f"req.robotId='{self.current_profile.robot_id}'; "
            f"req.robotModelRoot={self._matlab_quote(self.current_profile.matlab_model_root)}; "
            f"req.matlabPlanningEntry={self._matlab_quote(self.current_profile.matlab_planning_entry)}; "
            f"req.sceneScript={self._matlab_quote(self.current_profile.matlab_scene_script or '')}; "
            f"req.anchorFile={self._matlab_quote(self.current_profile.matlab_anchor_file or '')}; "
            f"req.maxPlanningTimeSec={max_time}; "
            "plan_hmi_robot_request(req);"
        )
        self.planning_request_path = request_path
        self.planning_output_path = output_path
        self.planning_report_path = report_path
        self.online_task_value = task
        self.online_output_view.clear()
        self.online_output_view.appendPlainText(f"task: {task}")
        self.online_output_view.appendPlainText(f"endpoints: {endpoints_text}")
        self.online_output_view.appendPlainText(f"free-space algorithm: {algorithm}")
        self.online_output_view.appendPlainText("weld algorithm: ACCP (fixed)")
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
        self.trajectory_block_reason = ""
        trajectory = load_trajectory(str(path))
        try:
            check_trajectory_robot_id(trajectory, self.current_profile.robot_id)
        except ValueError as exc:
            self.loaded_trajectory = None
            self.loaded_path = None
            self.trajectory_validation = None
            self.loaded_path_name = str(path)
            self.trajectory_block_reason = str(exc)
            self._update_trajectory_gate()
            self._update_buttons()
            raise
        if trajectory.joint_names != list(self.current_profile.joint_names):
            self.loaded_trajectory = None
            self.loaded_path = None
            self.trajectory_validation = None
            self.trajectory_block_reason = (
                f"BLOCKED: trajectory joint_names do not match {self.current_profile.robot_id} profile"
            )
            self._update_trajectory_gate()
            self._update_buttons()
            raise ValueError(self.trajectory_block_reason)
        self.loaded_trajectory = trajectory
        self._apply_planned_tcp_path()
        self.loaded_path = trajectory.positions
        self.loaded_path_name = str(path)
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
        default_dir = self.current_profile.trajectory_dir
        if not default_dir.is_dir():
            default_dir = self.current_profile.planning_runs_dir
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
            QMessageBox.critical(self, "轨迹加载失败", str(exc))
            self._log(f"Trajectory load failed: {exc}")

    def _validate_path(self):
        if self.loaded_trajectory is None:
            self._log("请先加载轨迹")
            return
        self.trajectory_validation = validate_trajectory(
            self.loaded_trajectory,
            joint_names=self.current_profile.joint_names,
            joint_limits=self.current_profile.joint_limits,
        )
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
        report = validate_trajectory(
            traj, q_actual=self.state.positions,
            joint_names=self.current_profile.joint_names,
            joint_limits=self.current_profile.joint_limits,
        )
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
        if not self.current_profile.supports_ros_control or self.worker is None:
            self._log(f"Execution BLOCKED: {self.current_profile.display_name} has no configured motion controller")
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
        if self.current_profile.robot_id == "UR5":
            frames = "工件 UR5_base→base；TCP 路径 base_link→base(Rz180°)"
        else:
            frames = "CR7 URDF base frame；未套用 UR5 工件位姿或 TCP 变换"
        self.scene_status_label.setText(f"3D 场景：{status}｜{frames}")

    def _on_scene_visibility_changed(self, _state=None) -> None:
        flags = {
            key: box.isChecked() for key, box in getattr(self, "scene_visibility_boxes", {}).items()
        }
        try:
            self.robot_view.set_visibility(**flags)
        except Exception as exc:
            self._log(f"Scene visibility update failed: {exc}")
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
        if self.current_profile.robot_id != "UR5":
            # CR7 currently has no HMI-validated TCP frame adapter. A path
            # authored in a CR7 frame must not inherit the UR5 Rz(180) map.
            setter(None)
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
            # tcp_index keeps the red TCP marker on the planned path sample so
            # the slider position and the TCP position always correspond.
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
        self.preview_start_monotonic = None
        self.preview_index = 0
        if hasattr(self, "preview_slider"):
            self.preview_slider.blockSignals(True)
            self.preview_slider.setValue(0)
            self.preview_slider.blockSignals(False)
        if self._feedback_valid(self.state):
            self.robot_view.set_state(self.state.positions, self.state.tcp_position, self.state.tcp_frame_id)
        else:
            self.robot_view.set_state(None, None, "")
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
        label = self.calibration_point_labels.get(key)
        if label is not None:
            label.setText("X: %.3f   Y: %.3f   Z: %.3f mm" % tuple(point * 1000.0))
            label.setStyleSheet("background:#e8f5e9;color:#146c2e;padding:6px 8px;font-family:monospace;")
            label.setToolTip(
                f"frame={self.state.tcp_frame_id or 'base'}; captured={time.strftime('%H:%M:%S')}"
            )
        self.calibration_result = None
        self.calibration_save_button.setEnabled(False)
        self._log(f"已读取标定点 {key}：" + ", ".join(f"{v * 1000.0:.3f} mm" for v in point))

    @staticmethod
    def _rpy_zyx_from_rotation(rotation: np.ndarray) -> np.ndarray:
        """Return roll, pitch, yaw (radians) for R = Rz(yaw)Ry(pitch)Rx(roll)."""
        value = float(np.clip(-rotation[2, 0], -1.0, 1.0))
        pitch = math.asin(value)
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        return np.array([roll, pitch, yaw], dtype=float)

    def _calculate_calibration(self) -> None:
        """Compute the workpiece frame from P0/PX/PY and check P_CHECK."""
        points = self.calibration_points
        if any(points[name] is None for name in ("P0", "PX", "PY")):
            self._log("标定计算失败：请先读取 P0、PX、PY")
            self.calibration_result_labels["status"].setText("INCOMPLETE：缺少 P0/PX/PY")
            self.calibration_result_labels["status"].setStyleSheet("color:#8a6d00;font-weight:bold;")
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
            self.calibration_result_labels["status"].setText("INVALID：旋转矩阵不是右手系")
            self.calibration_result_labels["status"].setStyleSheet("color:#9b1c1c;font-weight:bold;")
            return

        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rotation
        transform[:3, 3] = p0
        rpy = self._rpy_zyx_from_rotation(rotation)
        pcheck = points.get("P_CHECK")
        residual_mm = None if pcheck is None else float(np.linalg.norm(np.asarray(pcheck) - p0) * 1000.0)
        valid = residual_mm is not None and residual_mm <= self.calibration_residual_limit_mm
        self.calibration_result = {
            "T_base_workpiece_real": transform,
            "position": p0.copy(),
            "rotation_matrix": rotation.copy(),
            "rpy": rpy.copy(),
            "residual_mm": residual_mm,
            "valid": bool(valid),
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
            "未读取 P_CHECK" if residual_mm is None else f"{residual_mm:.3f} mm"
        )
        if pcheck is None:
            self.calibration_result_labels["check"].setText("未读取 P_CHECK（预测为 P0）")
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
        if residual_mm is None:
            status_text = "INCOMPLETE：请重复读取 P0 作为 P_CHECK"
            status_style = "color:#8a6d00;font-weight:bold;"
        elif valid:
            status_text = "VALID"
            status_style = "color:#146c2e;font-weight:bold;"
        else:
            status_text = f"INVALID：残差超过 {self.calibration_residual_limit_mm:.1f} mm"
            status_style = "color:#9b1c1c;font-weight:bold;"
        self.calibration_result_labels["status"].setText(status_text)
        self.calibration_result_labels["status"].setStyleSheet(status_style)
        self.calibration_save_button.setEnabled(bool(valid))
        if valid:
            try:
                self.robot_view.set_workpiece_calibration(transform)
                self._refresh_scene_status_label()
                self._log("工件三点标定 VALID：3D 显示已切换为 REAL CALIBRATION")
            except Exception as exc:
                self._log(f"工件位姿显示刷新失败：{exc}")
        else:
            self._log("工件标定几何有效，但尚未通过 P_CHECK 残差验证；未切换显示位姿")

    def _save_calibration(self) -> None:
        result = self.calibration_result
        if not result or not result.get("valid"):
            self._log("保存标定被拒绝：标定状态不是 VALID")
            return
        rotation = np.asarray(result["rotation_matrix"], dtype=float)
        payload = {
            "robot_id": self.current_profile.robot_id,
            "frame_id": "base",
            "position": np.asarray(result["position"], dtype=float).tolist(),
            "position_unit": "m",
            "rotation_matrix": rotation.tolist(),
            "rpy": np.asarray(result["rpy"], dtype=float).tolist(),
            "rpy_deg": np.degrees(np.asarray(result["rpy"], dtype=float)).tolist(),
            "timestamp": result["timestamp"],
            "residual_mm": float(result["residual_mm"]),
            "valid": True,
            "source": "REAL CALIBRATION",
            "points": {
                key: None if value is None else np.asarray(value, dtype=float).tolist()
                for key, value in self.calibration_points.items()
            },
        }
        try:
            self.calibration_file.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        except Exception as exc:
            self._log(f"保存工件标定失败：{exc}")
            return
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("REAL CALIBRATION")
        self.calibration_source_combo.blockSignals(False)
        self._log(f"工件标定已保存：{self.calibration_file}")

    def _reset_calibration(self) -> None:
        self.calibration_points = {"P0": None, "PX": None, "PY": None, "P_CHECK": None}
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
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.blockSignals(False)
        self._restore_simulation_calibration(log=False)
        self._log("已清空本次工件标定点，可重新示教")

    def _restore_simulation_calibration(self, log: bool = True) -> None:
        try:
            self.robot_view.clear_workpiece_calibration()
            self._refresh_scene_status_label()
        except Exception as exc:
            self._log(f"恢复仿真工件位姿失败：{exc}")
            return
        self.calibration_source_combo.blockSignals(True)
        self.calibration_source_combo.setCurrentText("SIMULATION")
        self.calibration_source_combo.blockSignals(False)
        if log:
            self._log("3D 工件显示已恢复为 SIMULATION；未修改保存的标定文件")

    def _load_saved_calibration(self) -> bool:
        """Load a previously saved VALID calibration when explicitly selected."""
        if not self.calibration_file.exists():
            return False
        try:
            payload = json.loads(self.calibration_file.read_text(encoding="utf-8"))
            stored_robot_id = payload.get("robot_id")
            if stored_robot_id and str(stored_robot_id) != self.current_profile.robot_id:
                self._log(
                    f"工件标定加载 BLOCKED：robot_id={stored_robot_id}，"
                    f"current_robot_id={self.current_profile.robot_id}"
                )
                return False
            position = np.asarray(payload["position"], dtype=float)
            rotation = np.asarray(payload["rotation_matrix"], dtype=float)
            residual = float(payload["residual_mm"])
            if position.shape != (3,) or rotation.shape != (3, 3):
                return False
            if not bool(payload.get("valid")) or not np.all(np.isfinite(position)) \
                    or not np.all(np.isfinite(rotation)):
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
                "source": "REAL CALIBRATION",
                "timestamp": str(payload.get("timestamp", "")),
            }
            self.calibration_result_labels["position"].setText(
                "[%.6f, %.6f, %.6f]" % tuple(position)
            )
            self.calibration_result_labels["rpy"].setText(
                "[%.3f°, %.3f°, %.3f°]" % tuple(np.degrees(self.calibration_result["rpy"]))
            )
            self.calibration_result_labels["residual"].setText(f"{residual:.3f} mm")
            self.calibration_result_labels["check"].setText("已从保存文件加载；P_CHECK 记录见 JSON")
            self.calibration_matrix_label.setText(
                "\n".join("[" + ", ".join(f"{v:.6f}" for v in row) + "]" for row in rotation)
            )
            self.calibration_result_labels["status"].setText("VALID（已加载保存结果）")
            self.calibration_result_labels["status"].setStyleSheet("color:#146c2e;font-weight:bold;")
            self.calibration_save_button.setEnabled(True)
            return True
        except Exception as exc:
            self._log(f"读取保存的工件标定失败：{exc}")
            return False

    def _calibration_source_changed(self, text: str) -> None:
        if text == "SIMULATION":
            self._restore_simulation_calibration(log=False)
            return
        result = self.calibration_result
        if not (result and result.get("valid")):
            self._load_saved_calibration()
            result = self.calibration_result
        if result and result.get("valid"):
            try:
                self.robot_view.set_workpiece_calibration(result["T_base_workpiece_real"])
                self._refresh_scene_status_label()
                self._log("3D 工件显示已切换为 REAL CALIBRATION")
            except Exception as exc:
                self._log(f"切换真实工件位姿失败：{exc}")
        else:
            self.calibration_source_combo.blockSignals(True)
            self.calibration_source_combo.setCurrentText("SIMULATION")
            self.calibration_source_combo.blockSignals(False)
            self._log("尚无 VALID 标定结果，继续使用 SIMULATION 工件位姿")


    def _save_home(self):
        if not self.current_profile.supports_real_robot:
            self._log(f"RealHome save BLOCKED: {self.current_profile.display_name} has no real robot feedback")
            return
        if self._motion_busy():
            self._log("RealHome blocked: motion busy (see Motion State / Current Goal)")
            return
        if len(self.state.positions) != 6:
            return
        payload = {
            "robot_id": self.current_profile.robot_id,
            "positions_rad": [float(v) for v in self.state.positions],
        }
        self.current_profile.real_home.parent.mkdir(parents=True, exist_ok=True)
        self.current_profile.real_home.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8"
        )
        self.home_info.setText(
            "已保存 RealHome：\n" +
            "  ".join(
                f"J{i + 1}={math.degrees(value):.3f}°"
                for i, value in enumerate(self.state.positions)
            )
        )
        self._log(f"RealHome saved to {self.current_profile.real_home}")

    def _open_rviz(self):
        if self.current_profile.robot_id != "UR5":
            self._log("RViz preset is UR5-specific; CR7 is displayed in the profile-aware 3D view")
            return
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
        self._stop_preview()
        if self.planning_process is not None:
            self.planning_process.terminate()
            if not self.planning_process.waitForFinished(1000):
                self.planning_process.kill()
        self._stop_ros_worker()
        event.accept()
