"""Operator-applied UR speed slider and measured live animation status."""
import math
import time
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QMessageBox, QPushButton, QSlider, QSpinBox
from ros.speed_slider import speed_request_reasons


class ExecutionLivePanel:
    def _init_execution_live(self):
        self.real_speed_pending = False
        self.last_speed_request_percent = None
        self.execution_feedback = None

    def _build_execution_live(self, layout):
        box = QGroupBox("运行时示教器倍率 / 实时动画（不生成或重新验收轨迹）")
        grid = QGridLayout(box)
        grid.addWidget(QLabel("目标倍率"), 0, 0)
        self.real_speed_slider = QSlider(Qt.Horizontal)
        self.real_speed_slider.setRange(1, 100)
        self.real_speed_slider.setValue(20)
        self.real_speed_slider.setTracking(False)
        self.real_speed_spin = QSpinBox()
        self.real_speed_spin.setRange(1, 100)
        self.real_speed_spin.setSuffix(" %")
        self.real_speed_spin.setValue(20)
        self.real_speed_slider.valueChanged.connect(self.real_speed_spin.setValue)
        self.real_speed_spin.valueChanged.connect(self.real_speed_slider.setValue)
        grid.addWidget(self.real_speed_slider, 0, 1)
        grid.addWidget(self.real_speed_spin, 0, 2)
        self.real_speed_apply_button = QPushButton("应用真机倍率")
        self.real_speed_apply_button.clicked.connect(self._apply_real_speed)
        grid.addWidget(self.real_speed_apply_button, 0, 3)
        self.real_speed_actual_label = QLabel("实际生效倍率：UNKNOWN")
        grid.addWidget(self.real_speed_actual_label, 1, 0, 1, 4)
        self.real_speed_request_label = QLabel("目标尚未应用；拖动不会自动发指令。运行中应用会改变示教器倍率。")
        self.real_speed_request_label.setWordWrap(True)
        grid.addWidget(self.real_speed_request_label, 2, 0, 1, 4)
        note = QLabel("100% = 当前文件原速。拖动不等于应用，也不会解除执行阻止；"
                      "需要生成更慢的已验收文件时，请使用上方“文件调速”。动态调速仍需现场验证。")
        note.setWordWrap(True)
        grid.addWidget(note, 3, 0, 1, 4)
        self.live_execution_label = QLabel("实机执行时，左侧机器人/焊枪/焊丝跟随最新关节反馈，不按预演计时播放。")
        self.live_execution_label.setWordWrap(True)
        grid.addWidget(self.live_execution_label, 4, 0, 1, 4)
        layout.addWidget(box)

    def _speed_ui_reasons(self):
        reasons = speed_request_reasons(self.state)
        if self.planning_busy or getattr(self, "robot_tcp_busy", False):
            reasons.append("请等待规划/TCP操作完成")
        if self.real_speed_pending:
            reasons.append("等待前一个倍率请求返回")
        if self._motion_busy() or self.goal_active:
            if (self.execution_trajectory is None or not self.goal_active
                    or self.motion_state_text != "EXECUTING"):
                reasons.append("仅允许本界面轨迹EXECUTING时调速；点动/取消/未知状态不可调速")
        return reasons

    def _apply_real_speed(self):
        reasons = self._speed_ui_reasons()
        if reasons:
            self.real_speed_request_label.setText("未发送：" + "；".join(reasons))
            return
        target = float(self.real_speed_spin.value())
        if self.goal_active:
            answer = QMessageBox.question(self, "运行中修改真机速度",
                f"将示教器速度倍率设为{target:.0f}%，会立即影响正在执行的轨迹。\n"
                "不是预演速度，不会改写轨迹文件；请确认现场条件。继续？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes: return
            if self._speed_ui_reasons(): return
        self.real_speed_pending = True
        self.real_speed_request_label.setText(f"已请求 {target:.0f}%：等待控制器返回，尚未确认生效")
        self.worker.request_speed_percent.emit(target)
        self._update_live_execution_panel()

    def _on_speed_request_done(self, ok, percent, message):
        # A timeout does not withdraw a command already handed to the service.
        if "等待原请求返回" not in message:
            self.real_speed_pending = False
        if ok: self.last_speed_request_percent = percent
        self.real_speed_request_label.setText(f"请求{percent:.0f}%：{message}")
        self._log(("[SPEED ACK] " if ok else "[SPEED UNKNOWN/FAIL] ") + f"{percent:.0f}% " + message)
        self._update_live_execution_panel()

    def _on_trajectory_feedback(self, record):
        if self.execution_trajectory is None or not self.goal_active: return
        if record.get("goal_label") != self._current_goal_display(): return
        self.execution_feedback = record

    def _execution_reference(self, now):
        feedback = self.execution_feedback
        if not feedback or now - feedback["received"] > 1.:
            return None
        if feedback.get("goal_label") != self._current_goal_display(): return None
        return feedback

    def _ensure_execution_live_view(self):
        if (self.goal_active or self._motion_busy()) and (
                self.preview_active or getattr(self.robot_view, "preview_active", False)):
            self._stop_preview()

    def _update_live_execution_panel(self):
        if not hasattr(self, "real_speed_apply_button"): return
        reasons = self._speed_ui_reasons()
        self.real_speed_apply_button.setEnabled(not reasons)
        self.real_speed_apply_button.setToolTip("；".join(reasons) if reasons else "点击后调用UR驱动的示教器倍率接口")
        fresh = (self.state.speed_scaling is not None and math.isfinite(self.state.speed_scaling)
                 and time.monotonic() - self.state.speed_scaling_stamp <= 1.)
        self.real_speed_actual_label.setText("实际生效倍率：" + (f"{self.state.speed_scaling:.1f}%（实时反馈，含控制器限速）" if fresh else "UNKNOWN / 反馈过期"))
        if self.goal_active or self._motion_busy():
            valid = self._feedback_valid(self.state)
            if valid:
                q = ", ".join(f"{v:.2f}" for v in np.rad2deg(self.state.positions))
                text = "3D跟随真机反馈（10Hz显示，不是定时预演）\n实际关节°：[" + q + "]"
                if (self.state.tcp_position is not None and time.monotonic() - self.state.tcp_stamp <= 1.):
                    text += "\n实时TCP mm：" + np.array2string(np.asarray(self.state.tcp_position)*1000, precision=2)
                    text += "；frame=" + self.state.tcp_frame_id
                else: text += "\n控制器TCP话题过期/UNKNOWN；红点来源以3D标注为准"
                self.live_execution_label.setText(text)
                self.live_execution_label.setStyleSheet("color:#146c2e;")
            else:
                self.live_execution_label.setText("真机反馈过期 / UNKNOWN；3D保留最后画面，不表示机器人已停或到达目标")
                self.live_execution_label.setStyleSheet("color:#9b1c1c;")
