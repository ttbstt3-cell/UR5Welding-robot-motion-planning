"""Simple software preflight for manually confirmed commissioning execution.

No field-report forms, permit files or automatic motion. Offline validation
is never relabelled as physical/servo acceptance.
"""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import (QFileDialog, QGroupBox, QHBoxLayout, QLabel,
                            QPushButton, QSizePolicy, QVBoxLayout, QMessageBox)
from safety.execution_preparation import file_hash, numeric_hash, offline_context


class ExecutionPreparationPanel:
    def _init_execution_preparation(self):
        self.execution_report_file = None
        self._execution_context_cache = None
        self._motion_limit_cache = None
        self.tool_payload_snapshot = None
        self.tool_payload_process = None

    def _build_execution_preparation(self, layout):
        box = QGroupBox("真机调试执行（软件检查 + 人工二次确认）")
        row = QHBoxLayout(box)
        self.preparation_status_label = QLabel("请先检查执行条件。离线PASS不等于实际伺服/物理安全验收。")
        self.preparation_status_label.setWordWrap(True)
        self.preparation_status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        row.addWidget(self.preparation_status_label, 1)
        self.execution_report_button = QPushButton("选择离线报告…")
        self.execution_report_button.setToolTip("通常自动读取轨迹同目录的report.json，无需手动选择")
        self.execution_report_button.clicked.connect(self._select_execution_report)
        row.addWidget(self.execution_report_button)
        self.execution_preparation_box = box; layout.addWidget(box)
        tool_box = QGroupBox("工具负载 / 加速度验收（只读核对，不改机器人参数）")
        tool_layout = QVBoxLayout(tool_box)
        self.tool_dynamics_label = QLabel("控制器登记负载：UNKNOWN；实物及加速度验收：待核实")
        self.tool_dynamics_label.setWordWrap(True)
        tool_layout.addWidget(self.tool_dynamics_label)
        tool_buttons = QHBoxLayout()
        self.tool_payload_read_button = QPushButton("读取登记负载（只读）")
        self.tool_payload_read_button.clicked.connect(self._read_tool_payload)
        self.tool_acceptance_draft_button = QPushButton("保存验收草稿…")
        self.tool_acceptance_draft_button.clicked.connect(self._save_tool_acceptance_draft)
        self.tool_acceptance_import_button = QPushButton("导入已审核验收记录…")
        self.tool_acceptance_import_button.clicked.connect(self._import_tool_acceptance)
        for button in (self.tool_payload_read_button, self.tool_acceptance_draft_button, self.tool_acceptance_import_button):
            tool_buttons.addWidget(button)
        tool_layout.addLayout(tool_buttons)
        layout.addWidget(tool_box)
        # MainWindow's bounded display timer refreshes these widgets too.
        # Do not schedule another full preflight on top of every ROS update.

    def _refresh_execution_review(self):
        self._update_trajectory_gate(); self._update_buttons()

    def _execution_preparation_loaded(self, path):
        path = Path(path)
        reports = (path.with_name(path.stem + "_report.json"), path.parent / "report.json")
        self.execution_report_file = next((p for p in reports if p.is_file()), None)
        self._execution_context_cache = None

    def _execution_context(self):
        if self.loaded_trajectory is None or self.execution_report_file is None:
            raise ValueError("需要加载连续规划输出及对应离线报告")
        # A raw preview cannot be commissioned. Reject it before parsing YAML,
        # hashing geometry or reading a report on the GUI refresh path.
        if self.loaded_trajectory.metadata.get("workflow_stage") != "PHYSICAL_VALIDATION_PENDING":
            raise ValueError("需要加载已完成连续规划的离线优化轨迹，不是原始预览")
        binding = self._current_calibration_binding()
        from safety.robot_motion_limits import PROFILE
        motion_limits_hash = file_hash(PROFILE)
        home_hash = file_hash(self._realhome_config_path())
        key = (numeric_hash(self.loaded_trajectory), file_hash(self.loaded_trajectory.source_path),
               file_hash(self.execution_report_file), json.dumps(binding, sort_keys=True), home_hash,
               motion_limits_hash)
        cached = self._execution_context_cache
        if cached and cached[0] == key:
            context = cached[1]
            if (file_hash(context["reference_path"]) == context["reference_sha256"]
                    and file_hash(context["recorded_trajectory_path"]) == context["trajectory_sha256"]):
                return context
        home_record = self._saved_real_home_record()
        home = home_record.get("realhome_version") if home_record else None
        if file_hash(self._realhome_config_path()) != home_hash:
            raise ValueError("RealHome配置在读取期间发生变化，请重新检查")
        context = offline_context(self.loaded_trajectory, self.execution_report_file, binding, home)
        self._execution_context_cache = (key, context)
        return context

    def _execution_software_reasons(self):
        try:
            self._execution_context()
            return []
        except (ValueError, OSError, KeyError, TypeError) as exc:
            return ["轨迹软件检查未通过：" + str(exc)]

    def _trajectory_motion_limit_reasons(self):
        """Bound cached numeric work to immutable trajectory/model/limit data."""
        traj = self.loaded_trajectory
        if traj is None: return []
        try:
            from safety.robot_motion_limits import PROFILE, URDF, ROOT, check_motion
            from safety.tool_dynamics_acceptance import RECORD, fresh_sample, bindings
            live = fresh_sample(getattr(self, 'tool_payload_snapshot', None))
            key = (numeric_hash(traj), file_hash(PROFILE), file_hash(URDF),
                   file_hash(ROOT / 'real_tcp_calibration_active.json'),
                   file_hash(RECORD) if RECORD.is_file() else None,
                   json.dumps(bindings(), sort_keys=True), json.dumps(live, sort_keys=True))
            cached = self._motion_limit_cache
            if cached and cached[0] == key: return list(cached[1])
            if traj.velocities is None or traj.accelerations is not None:
                reasons = ['MOTION_LIMITS_INVALID：执行限值验收需要positions+velocities三次插值输入']
            else:
                reasons, _ = check_motion(traj.positions, traj.time_from_start, traj.velocities, execution=True, live_payload=live)
            self._motion_limit_cache = (key, tuple(reasons))
            return reasons
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            return ['MOTION_LIMITS_UNKNOWN：' + str(exc)]

    def _update_execution_preparation(self):
        if not hasattr(self, "preparation_status_label"):
            return
        self._update_tool_dynamics_panel()
        self.execution_report_button.setEnabled(not self.planning_busy and not self._motion_busy()
                                               and not self.goal_active)
        if self.planning_busy:
            self.preparation_status_label.setText("轨迹规划进行中；完成后重新检查执行条件，真机执行已禁用。")
            return
        software_reasons = self._execution_software_reasons()
        reasons = software_reasons + self._automatic_execution_gate_reasons()
        if self.loaded_trajectory is not None and not software_reasons:
            self.trajectory_info_labels["coordinate"].setText("ROS关节角（rad）")
        if reasons:
            self.preparation_status_label.setText("待完成：" + reasons[0])
            self.preparation_status_label.setStyleSheet("color:#8a5a00;")
        else:
            self.preparation_status_label.setText("软件检查PASS，可点击上方“确认并执行”。按轨迹时间/速度运行，"
                "不是自动5%降速；实际伺服/物理安全仍由现场调试确认。")
            self.preparation_status_label.setStyleSheet("color:#146c2e;")

    def _check_execution_preparation(self):
        self._execution_context_cache = None
        if self.loaded_trajectory is not None:
            self._validate_path()
        self._update_trajectory_gate(); self._update_buttons()

    def _select_execution_report(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择该轨迹的连续规划离线报告", "", "Report (*.json)")
        if path:
            self.execution_report_file = Path(path)
            self._execution_context_cache = None
            self._check_execution_preparation()
