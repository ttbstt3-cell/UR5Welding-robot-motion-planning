"""Evidence registration and session-scoped authorization; never sends motion."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QCheckBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QScrollArea, QSizePolicy, QVBoxLayout, QWidget)

from safety.execution_preparation import (draft_record, file_hash, numeric_hash,
    offline_context, validate_record, issue_authorization, validate_authorization)


class ExecutionPreparationPanel:
    def _init_execution_preparation(self):
        self.execution_report_file = None
        self.field_acceptance_path = None
        self.execution_authorization = None
        self._execution_context_cache = None

    def _build_execution_preparation(self, layout):
        box = QGroupBox("真机执行准备与现场验收（登记/授权均不发送运动）")
        grid = QGridLayout(box)
        note = QLabel("离线PASS不等于现场验收。请由现场责任人提供物理安全和伺服验收证据；"
                      "软件只检查记录一致性，不能替代风险评估或实体安全装置。")
        note.setWordWrap(True); note.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        grid.addWidget(note, 0, 0, 1, 3)
        self.preparation_labels = {}
        for row, (key, title) in enumerate((("offline", "离线优化 / 版本"),
                ("runtime", "机器人 / 控制器 / 起点"), ("field", "物理安全 / 伺服验收"),
                ("authorization", "当前会话执行授权")), 1):
            grid.addWidget(QLabel(title), row, 0)
            label = QLabel("待检查"); label.setWordWrap(True)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.preparation_labels[key] = label; grid.addWidget(label, row, 1, 1, 2)
        actions = (("check", "检查执行条件", self._check_execution_preparation),
            ("report", "选择离线报告…", self._select_execution_report),
            ("register", "登记现场验收…", self._register_field_acceptance),
            ("import", "导入现场验收记录…", self._import_field_acceptance),
            ("draft", "保存验收草稿…", self._save_acceptance_draft),
            ("authorize", "确认执行授权…（不运动）", self._authorize_execution))
        self.preparation_buttons = {}
        for i, (key, title, callback) in enumerate(actions):
            button = QPushButton(title); button.setMinimumHeight(self._u(38))
            button.clicked.connect(callback); self.preparation_buttons[key] = button
            grid.addWidget(button, 5+i//3, i%3)
        revoke = QPushButton("撤销本次授权"); revoke.clicked.connect(self._revoke_execution_authorization)
        self.preparation_buttons["revoke"] = revoke; grid.addWidget(revoke, 7, 0)
        warning = QLabel("授权仅对当前会话/当前轨迹有效，最长1小时；更换轨迹或证据、版本变化即失效。"
                         "执行按轨迹时间与速度发送，不会自动降为5%。")
        warning.setWordWrap(True); warning.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        grid.addWidget(warning, 7, 1, 1, 2)
        self.execution_preparation_box = box; layout.addWidget(box)
        self.execution_review_timer = QTimer(self)
        self.execution_review_timer.setInterval(1000)
        self.execution_review_timer.timeout.connect(self._refresh_execution_review)
        self.execution_review_timer.start()

    def _refresh_execution_review(self):
        self._update_trajectory_gate(); self._update_buttons()

    def _execution_preparation_loaded(self, path):
        self.execution_authorization = None
        self.field_acceptance_path = None
        report = Path(path).parent / "report.json"
        self.execution_report_file = report if report.is_file() else None
        self._execution_context_cache = None

    def _execution_context(self):
        if self.loaded_trajectory is None or self.execution_report_file is None:
            raise ValueError("需要加载连续规划输出及对应离线报告")
        record = self._saved_real_home_record()
        home = record.get("realhome_version") if record else None
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

    def _field_acceptance(self, context):
        if self.field_acceptance_path is None:
            raise ValueError("缺少现场物理安全 / 伺服验收记录，请登记或导入")
        path = Path(self.field_acceptance_path)
        record = json.loads(path.read_text(encoding="utf-8"))
        validate_record(record, context, path.parent)
        return record

    def _execution_authorization_reasons(self):
        try:
            context = self._execution_context()
            self._field_acceptance(context)
            validate_authorization(self.execution_authorization, context)
            return []
        except (ValueError, OSError, KeyError, TypeError) as exc:
            return ["真机执行准备未通过：" + str(exc)]

    def _update_execution_preparation(self):
        if not hasattr(self, "preparation_labels"):
            return
        context = None; field_ok = False
        try:
            context = self._execution_context()
            self.preparation_labels["offline"].setText("PASS · 离线证据与当前版本一致")
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.preparation_labels["offline"].setText("待完成：" + str(exc))
        runtime = self._automatic_execution_gate_reasons()
        self.preparation_labels["runtime"].setText("PASS" if not runtime else "；".join(runtime))
        try:
            if context is None:
                raise ValueError("请先完成离线/版本检查")
            record = self._field_acceptance(context); field_ok = True
            self.preparation_labels["field"].setText("记录检查PASS · 现场责任人：" + record["inspector"])
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.preparation_labels["field"].setText("待验收：" + str(exc))
        reasons = self._execution_authorization_reasons()
        valid = not reasons and not runtime and self.trajectory_validation is not None
        self.preparation_labels["authorization"].setText(
            "授权有效 · 可二次确认执行（按轨迹速度）" if valid else "BLOCKED · " +
            (reasons[0] if reasons else "实时/轨迹条件未通过"))
        self.preparation_labels["authorization"].setStyleSheet("color:#146c2e;" if valid else "color:#8a5a00;")
        if self.loaded_trajectory is not None:
            coordinate = self.loaded_trajectory.metadata.get("coordinate_space", "unknown")
            self.trajectory_info_labels["coordinate"].setText(
                "ROS joint radians（现场记录确认）" if valid and coordinate == "unknown" else str(coordinate))
        busy = self.planning_busy or self._motion_busy() or self.goal_active or getattr(self, "robot_tcp_busy", False)
        for key, button in self.preparation_buttons.items():
            button.setEnabled(not busy)
        for key in ("register", "draft"):
            self.preparation_buttons[key].setEnabled(not busy and context is not None)
        self.preparation_buttons["authorize"].setEnabled(not busy and context is not None and field_ok and not runtime)
        self.preparation_buttons["revoke"].setEnabled(self.execution_authorization is not None)

    def _check_execution_preparation(self):
        self._execution_context_cache = None
        if self.loaded_trajectory is not None:
            self._validate_path()
        self._update_execution_preparation(); self._update_trajectory_gate(); self._update_buttons()

    def _select_execution_report(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择连续规划离线报告", "", "Report (*.json)")
        if path:
            self.execution_report_file = Path(path); self.execution_authorization = None
            self._execution_context_cache = None; self._check_execution_preparation()

    def _import_field_acceptance(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入现场验收记录（不是离线规划报告）", "", "JSON (*.json)")
        if path:
            self.field_acceptance_path = Path(path); self.execution_authorization = None
            self._check_execution_preparation()

    def _save_review_record(self, record):
        folder = self._execution_review_directory(); folder.mkdir(parents=True, exist_ok=True)
        default = folder / ("field_acceptance_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
        path, _ = QFileDialog.getSaveFileName(self, "保存新的现场验收记录（不覆盖已有文件）", str(default), "JSON (*.json)")
        if not path:
            return False
        with Path(path).open("x", encoding="utf-8") as handle:
            json.dump(record, handle, ensure_ascii=False, indent=2, allow_nan=False)
        self.field_acceptance_path = Path(path); self.execution_authorization = None
        self._check_execution_preparation()
        return True

    def _execution_review_directory(self):
        from gui.main_window import ROOT
        return ROOT / "logs" / "execution_reviews"

    def _save_acceptance_draft(self):
        try:
            self._save_review_record(draft_record(self._execution_context()))
        except (ValueError, OSError, KeyError, TypeError) as exc:
            QMessageBox.warning(self, "无法保存验收草稿", str(exc))

    def _register_field_acceptance(self):
        try:
            context = self._execution_context()
        except (ValueError, OSError, KeyError, TypeError) as exc:
            QMessageBox.warning(self, "请先检查离线轨迹", str(exc)); return
        dialog = QDialog(self); dialog.setWindowTitle("登记已完成的现场验收（不执行机器人）")
        dialog.resize(800, 760); outer = QVBoxLayout(dialog)
        info = QLabel("只能填写已实际完成的验收结论，须附报告。数值必须来自实测/已批准规程，"
                      "不得为了放行填写；软件不核实附件内容真实性，也不是安全认证。"
                      "首次伺服调试应在独立批准的现场流程下完成，本窗口不会自动调试或运动。")
        info.setWordWrap(True); outer.addWidget(info)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); page = QWidget(); form = QFormLayout(page)
        scroll.setWidget(page); outer.addWidget(scroll)
        text = {}
        for key, title in (("inspector", "现场责任人"), ("qualification", "资质/岗位"),
                           ("procedure_reference", "批准的验收规程/记录编号"),
                           ("performed_at_utc", "实际验收时间（ISO含时区）")):
            text[key] = QLineEdit(); form.addRow(title, text[key])
        files = {}
        for name, title in (("physical", "物理安全验收附件"), ("servo", "伺服/实际控制器验收附件")):
            field = QLineEdit(); field.setReadOnly(True); row = QHBoxLayout(); row.addWidget(field)
            button = QPushButton("选择…"); row.addWidget(button)
            def choose(_checked=False, target=field):
                path, _ = QFileDialog.getOpenFileName(dialog, "选择现场验收证据", "", "Evidence (*)")
                if path: target.setText(path)
            button.clicked.connect(choose); form.addRow(title, row); files[name] = field
        values = {}
        for key, title, decimals in (("minimum_verified_clearance_mm", "实测最小间隙 mm（非MATLAB模型间隙）", 3),
                ("uncertainty_mm", "总安装/标定不确定性 mm", 3),
                ("required_margin_mm", "现场批准的独立安全余量 mm", 3),
                ("max_tcp_tracking_error_mm", "实测最大TCP跟踪误差 mm", 6),
                ("max_tcp_orientation_error_deg", "实测最大姿态误差 °", 6),
                ("approved_velocity_rad_s", "已验收速度包络 rad/s", 9),
                ("approved_acceleration_rad_s2", "已验收加速度包络 rad/s²", 9)):
            value = QDoubleSpinBox(); value.setDecimals(decimals); value.setRange(-1, 100000)
            value.setValue(-1); value.setSpecialValueText("未填写"); form.addRow(title, value); values[key] = value
        checks = {}
        for key, title in (("installation_verified", "已检查真实TCP、工具安装与机器人模型一致"),
                ("workpiece_environment_verified", "已检查工件、地面、环境及人员隔离"),
                ("wire_approximation_reviewed", "已独立评估焊丝碰撞缩短3mm近似及小间隙风险"),
                ("actual_controller_and_scaling_verified", "已实测当前scaled控制器P+V插值、关节坐标及时间倍率行为")):
            checks[key] = QCheckBox(title); form.addRow(checks[key])
        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        draft = buttons.addButton("保存草稿", QDialogButtonBox.ActionRole)
        reviewed = buttons.addButton("登记已完成验收", QDialogButtonBox.ActionRole)
        buttons.rejected.connect(dialog.reject); outer.addWidget(buttons)
        def save(is_draft):
            record = draft_record(context); record.update({key: value.text().strip() for key, value in text.items()})
            record["status"] = "DRAFT" if is_draft else "REVIEWED"
            try:
                for name, field in files.items():
                    p = Path(field.text()) if field.text() else None
                    record[name].update(status="PENDING" if is_draft else "PASS",
                        report_file=str(p.resolve()) if p else "", report_sha256=file_hash(p) if p else "")
                for key, widget in values.items():
                    target = "physical" if key in ("minimum_verified_clearance_mm", "uncertainty_mm", "required_margin_mm") else "servo"
                    record[target][key] = widget.value() if widget.value() >= 0 else None
                for key, widget in checks.items():
                    record["servo" if key == "actual_controller_and_scaling_verified" else "physical"][key] = widget.isChecked()
                if not is_draft:
                    validate_record(record, context, Path.cwd())
                if self._save_review_record(record): dialog.accept()
            except (ValueError, OSError, KeyError, TypeError) as exc:
                QMessageBox.warning(dialog, "记录不满足要求，可先保存草稿", str(exc))
        draft.clicked.connect(lambda: save(True)); reviewed.clicked.connect(lambda: save(False)); dialog.exec_()

    def _authorize_execution(self):
        try:
            self._validate_path()
            context = self._execution_context(); record = self._field_acceptance(context)
            reasons = self._automatic_execution_gate_reasons()
            if reasons: raise ValueError("；".join(reasons))
            dialog = QDialog(self); dialog.setWindowTitle("签署本次执行授权（此操作不发送运动）")
            form = QVBoxLayout(dialog); label = QLabel("授权当前精确轨迹，最长1小时。"
                "实际运动仍须再点击执行并二次确认。授权是本地责任记录，不是安全认证。")
            label.setWordWrap(True); form.addWidget(label)
            operator = QLineEdit(); operator.setPlaceholderText("授权操作人姓名"); form.addWidget(operator)
            acknowledged = QCheckBox("已核对当前轨迹、现场验收证据和实际执行速度，承担现场操作责任")
            form.addWidget(acknowledged)
            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            ok = buttons.button(QDialogButtonBox.Ok); ok.setText("签署授权（不运动）"); ok.setEnabled(False)
            def update(): ok.setEnabled(bool(operator.text().strip()) and acknowledged.isChecked())
            operator.textChanged.connect(update); acknowledged.toggled.connect(update)
            buttons.accepted.connect(dialog.accept); buttons.rejected.connect(dialog.reject); form.addWidget(buttons)
            if dialog.exec_() != QDialog.Accepted: return
            # Recheck after modal review: feedback/versions may have changed.
            fresh = self._execution_context()
            if fresh != context or self._automatic_execution_gate_reasons():
                raise ValueError("条件在授权期间发生变化，请重新检查")
            self.execution_authorization = issue_authorization(record, fresh, self.field_acceptance_path, operator.text())
            folder = self._execution_review_directory(); folder.mkdir(parents=True, exist_ok=True)
            p = folder / ("authorization_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
            with p.open("x", encoding="utf-8") as handle:
                json.dump(self.execution_authorization, handle, ensure_ascii=False, indent=2, allow_nan=False)
            self._log("本地执行授权已登记（未发送运动）；实际执行仍需二次确认：" + str(p))
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.execution_authorization = None; QMessageBox.warning(self, "无法签署执行授权", str(exc))
        self._update_trajectory_gate(); self._update_buttons()

    def _revoke_execution_authorization(self):
        self.execution_authorization = None
        self._log("本次执行授权已撤销；不会发送运动或自动取消正在执行的goal，停止请使用STOP MOTION")
        self._update_trajectory_gate(); self._update_buttons()
