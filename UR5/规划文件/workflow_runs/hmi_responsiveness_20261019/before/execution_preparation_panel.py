"""Simple software preflight for manually confirmed commissioning execution.

No field-report forms, permit files or automatic motion. Offline validation
is never relabelled as physical/servo acceptance.
"""
from __future__ import annotations

import json
from pathlib import Path
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QFileDialog, QGroupBox, QHBoxLayout, QLabel,
                            QPushButton, QSizePolicy)
from safety.execution_preparation import file_hash, numeric_hash, offline_context


class ExecutionPreparationPanel:
    def _init_execution_preparation(self):
        self.execution_report_file = None
        self._execution_context_cache = None

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
        self.execution_review_timer = QTimer(self)
        self.execution_review_timer.setInterval(1000)
        self.execution_review_timer.timeout.connect(self._refresh_execution_review)
        self.execution_review_timer.start()

    def _refresh_execution_review(self):
        self._update_trajectory_gate(); self._update_buttons()

    def _execution_preparation_loaded(self, path):
        report = Path(path).parent / "report.json"
        self.execution_report_file = report if report.is_file() else None
        self._execution_context_cache = None

    def _execution_context(self):
        if self.loaded_trajectory is None or self.execution_report_file is None:
            raise ValueError("需要加载连续规划输出及对应离线报告")
        home_record = self._saved_real_home_record()
        home = home_record.get("realhome_version") if home_record else None
        binding = self._current_calibration_binding()
        key = (numeric_hash(self.loaded_trajectory), file_hash(self.loaded_trajectory.source_path),
               file_hash(self.execution_report_file), json.dumps(binding, sort_keys=True), home)
        cached = self._execution_context_cache
        if cached and cached[0] == key:
            context = cached[1]
            if (file_hash(context["reference_path"]) == context["reference_sha256"]
                    and file_hash(context["recorded_trajectory_path"]) == context["trajectory_sha256"]):
                return context
        context = offline_context(self.loaded_trajectory, self.execution_report_file, binding, home)
        self._execution_context_cache = (key, context)
        return context

    def _execution_software_reasons(self):
        try:
            self._execution_context()
            return []
        except (ValueError, OSError, KeyError, TypeError) as exc:
            return ["轨迹软件检查未通过：" + str(exc)]

    def _update_execution_preparation(self):
        if not hasattr(self, "preparation_status_label"):
            return
        reasons = self._execution_software_reasons() + self._automatic_execution_gate_reasons()
        if self.loaded_trajectory is not None and not self._execution_software_reasons():
            self.trajectory_info_labels["coordinate"].setText("ROS关节角（rad）")
        if reasons:
            self.preparation_status_label.setText("待完成：" + reasons[0])
            self.preparation_status_label.setStyleSheet("color:#8a5a00;")
        else:
            self.preparation_status_label.setText("软件检查PASS，可点击上方“确认并执行”。按轨迹时间/速度运行，"
                "不是自动5%降速；实际伺服/物理安全仍由现场调试确认。")
            self.preparation_status_label.setStyleSheet("color:#146c2e;")
        self.execution_report_button.setEnabled(not self.planning_busy and not self._motion_busy()
                                               and not self.goal_active)

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
