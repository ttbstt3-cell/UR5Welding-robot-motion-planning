"""Manual active-TCP parameter workflow; no trajectory/action commands.

Only an explicit confirmed write launches the native parameter writer. Reads
use a separate output-only RTDE session and cannot alter robot parameters.
"""
from __future__ import annotations

import hashlib
import json
import time
import uuid
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QLabel, QMessageBox, QPushButton

from calibration.tcp_frames import (
    calibration_write_quality,
    controller_snapshot_to_flange_tcp,
    flange_offset_to_controller_tcp,
    validate_rtde_model_alignment,
    validate_saved_pivot_payload,
)
from calibration.tcp_pivot import fit_pivot_calibration
from calibration.controller_tcp_sync import (
    sync_controller_tcp_record, validate_controller_tcp_record,
)
from calibration.tcp_frames import transform_to_ur_pose, ur_pose_to_transform
from gui.robot_view import URDF_PATH, UrdfKinematicModel

HMI_ROOT = Path(__file__).resolve().parents[1]


class TcpRobotPanel:
    def _init_robot_tcp_panel(self, layout) -> None:
        self.robot_tcp_snapshot = None
        self.robot_tcp_received = 0.0
        self.robot_tcp_process = None
        self.robot_tcp_busy = False
        self.robot_tcp_mode = ""
        self.robot_tcp_report_path = None
        self.robot_tcp_auto_sync = True
        self.robot_tcp_auto_last_error = ""
        self.robot_tcp_sync_error = "等待新鲜控制器 TCP"
        self.robot_tcp_sync_stamp = 0.0
        self.tcp_pendant_candidate = None
        self.tcp_pendant_candidate_path = None
        self.tcp_saved_verification_mm = []
        self.tcp_saved_verification_recomputed = False
        self.tcp_fk_model = UrdfKinematicModel()
        self.tcp_fk_sha256 = hashlib.sha256(URDF_PATH.read_bytes()).hexdigest()
        box = QGroupBox("6. 真机当前 TCP（直接读取；写入需人工确认）")
        grid = QGridLayout(box)
        grid.setContentsMargins(self._u(10), self._u(10), self._u(10), self._u(10))
        grid.setVerticalSpacing(self._u(8))
        self.tcp_robot_read_button = QPushButton("读取并同步示教器 TCP")
        self.tcp_saved_load_button = QPushButton("加载已保存标定")
        self.tcp_robot_write_button = QPushButton("写入真机当前 TCP（运行时）…")
        self.tcp_pendant_candidate_button = QPushButton("采用示教器当前 TCP（生成候选）")
        self.tcp_pendant_save_button = QPushButton("确认并保存新 TCP 版本")
        self.tcp_pendant_apply_button = QPushButton("应用到真实规划模型")
        self.tcp_robot_read_button.clicked.connect(self._read_robot_tcp)
        self.tcp_saved_load_button.clicked.connect(self._load_saved_tcp_fit)
        self.tcp_robot_write_button.clicked.connect(self._write_robot_tcp)
        self.tcp_pendant_candidate_button.clicked.connect(self._make_pendant_tcp_candidate)
        self.tcp_pendant_save_button.clicked.connect(self._save_pendant_tcp_candidate)
        self.tcp_pendant_apply_button.clicked.connect(self._apply_pendant_tcp_candidate)
        for i, button in enumerate((self.tcp_robot_read_button, self.tcp_saved_load_button,
                                    self.tcp_robot_write_button)):
            button.setMinimumHeight(self._u(42))
            grid.addWidget(button, 0, i)
        self.tcp_robot_auto_sync_box = QCheckBox("自动同步到规划（不写机器人）")
        self.tcp_robot_auto_sync_box.setChecked(True)
        self.tcp_robot_auto_sync_box.toggled.connect(self._set_tcp_auto_sync)
        grid.addWidget(self.tcp_robot_auto_sync_box, 0, 3)
        # Superseded manual local-adoption workflow. Keep methods for history,
        # but do not offer a second planning TCP source in the current UI.
        for button in (self.tcp_pendant_candidate_button, self.tcp_pendant_save_button,
                       self.tcp_pendant_apply_button):
            button.setParent(box)
            button.hide()
        self.tcp_pendant_save_button.setEnabled(False)
        self.tcp_pendant_apply_button.setEnabled(False)
        for button in (self.tcp_pendant_candidate_button, self.tcp_pendant_save_button,
                       self.tcp_pendant_apply_button):
            button.setMinimumHeight(self._u(38))
        self.robot_tcp_labels = {}
        for row, (key, title) in enumerate((
            ("current", "UR5 当前偏移"),
            ("hmi", "HMI 已应用 TCP（规划模型）"),
            ("difference", "与示教器 TCP 差异"),
            ("proposed", "标定后待写入"),
            ("alignment", "URDF / 控制器坐标核对"),
            ("quality", "标定写入质量"),
            ("state", "操作状态"),
        ), start=2):
            grid.addWidget(QLabel(title), row, 0, Qt.AlignTop)
            value = QLabel("—")
            value.setWordWrap(True)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(value, row, 1, 1, 3)
            self.robot_tcp_labels[key] = value
        note = QLabel(
            "这里直接读取 UR5 控制器的 active tcp_offset（XYZ: mm；旋转向量 RX/RY/RZ: rad），"
            "控制器当前生效 TCP 是真实规划唯一来源；坐标核对通过后自动同步完整 6D 到本地规划版本。"
            "TCP 变化后旧轨迹失效，必须重新规划和碰撞验证；四点标定只作为备用记录。"
            "写入只更新 XYZ，保留实时读取的工具姿态。写入前需手动停止机器人程序并保持静止。\n"
            "写入的是当前激活 TCP。重启或重新运行程序可能恢复安装默认值；"
            "永久保存仍需在示教器安装配置中保存。软件不会自动写回、运行程序或上电。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#6b7280;")
        grid.addWidget(note, 9, 0, 1, 4)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        layout.addWidget(box)
        self.robot_tcp_ui_timer = QTimer(self)
        self.robot_tcp_ui_timer.setInterval(1000)
        self.robot_tcp_ui_timer.timeout.connect(self._update_robot_tcp_controls)
        self.robot_tcp_ui_timer.start()
        self.robot_tcp_auto_timer = QTimer(self)
        self.robot_tcp_auto_timer.setInterval(5000)
        self.robot_tcp_auto_timer.timeout.connect(self._auto_read_robot_tcp)
        self.robot_tcp_auto_timer.start()
        QTimer.singleShot(1500, self._auto_read_robot_tcp)
        self._update_robot_tcp_controls()

    def _set_tcp_auto_sync(self, enabled: bool) -> None:
        self.robot_tcp_auto_sync = bool(enabled)
        if self.robot_tcp_auto_sync:
            self._auto_read_robot_tcp()
        self._log("示教器 TCP 自动同步：" + ("开启（只读）" if enabled else "关闭"))

    def _auto_read_robot_tcp(self) -> None:
        """Refresh the controller's active TCP through the output-only RTDE helper."""
        if not self.robot_tcp_auto_sync:
            return
        # Do not start a second RTDE session while the manual button/build is
        # active.  The helper only reads tcp_offset/actual_TCP_pose and never
        # sends set_tcp in this mode.
        if self.robot_tcp_busy or self._motion_busy() or self.goal_active or self._jog_pending:
            return
        if getattr(self, "tool_payload_process", None) is not None:
            return
        self._launch_robot_tcp_helper("read")

    def _sync_controller_tcp_to_planning(self) -> bool:
        """Mirror fresh controller parameters locally; never invoke writer."""
        if self.robot_tcp_snapshot is None or time.monotonic() - self.robot_tcp_received > 30.0:
            self.robot_tcp_sync_error = "UNKNOWN：没有新鲜控制器 TCP"
            return False
        if self._motion_busy() or self.goal_active or self._jog_pending or self.planning_busy or self.robot_tcp_busy:
            self.robot_tcp_sync_error = "等待运动/规划结束后同步；不更改运行中的模型版本"
            return False
        try:
            if hashlib.sha256(URDF_PATH.read_bytes()).hexdigest() != self.tcp_fk_sha256:
                raise ValueError("URDF 文件已改变，请重启 HMI 加载模型后同步")
            record, changed = sync_controller_tcp_record(
                self.tcp_calibration_active_file.parent, self.tcp_fk_model,
                self.robot_tcp_snapshot, self.tcp_fk_sha256)
            if changed:
                self._invalidate_tcp_trajectories()
            if changed or getattr(self, "robot_tcp_view_version", None) != record["version"]:
                setter = getattr(self.robot_view, "set_planning_tcp_transform", None)
                if callable(setter):
                    setter(record["T_flange_tcp"])
                else:
                    self.robot_view.set_display_tcp_offset(record["position_flange_tcp_m"])
                self.robot_tcp_view_version = record["version"]
            if changed:
                self._log("示教器完整 6D TCP 已自动同步到本地规划版本：" + record["version"]
                          + "；旧轨迹失效，需重新规划/碰撞验证；未写机器人")
                self._refresh_online_goal_display()
                self._refresh_scene_status_label()
            self.robot_tcp_sync_error = ""
            self.robot_tcp_sync_stamp = self.robot_tcp_received
            self.robot_tcp_labels["state"].setText("SYNCED：示教器 TCP = 真实规划 TCP（未写机器人）")
            return True
        except (OSError, ValueError, TypeError, KeyError) as exc:
            reason = "TCP 自动同步 BLOCKED：" + str(exc)
            if reason != self.robot_tcp_sync_error:
                self._log(reason)
            self.robot_tcp_sync_error = reason
            self.robot_tcp_labels["state"].setText(reason)
            return False

    @staticmethod
    def _format_tcp6(values) -> str:
        p = np.asarray(values, dtype=float)
        return ("XYZ [%.3f, %.3f, %.3f] mm\nRX/RY/RZ [%.6f, %.6f, %.6f] rad"
                % tuple(np.r_[p[:3] * 1000.0, p[3:]]))

    def _hmi_applied_flange_offset(self):
        """Return only the controller-authoritative planning mirror."""
        try:
            record = json.loads(self.tcp_calibration_active_file.read_text(encoding="utf-8"))
            T = validate_controller_tcp_record(record, self.tcp_fk_model, self.tcp_fk_sha256)
            return T[:3, 3], self.tcp_calibration_active_file.name
        except (OSError, ValueError, TypeError, KeyError):
            return None, "UNKNOWN: controller ACTIVE mirror unavailable"

    def _update_robot_tcp_comparison(self) -> None:
        """Render controller TCP, HMI TCP and their frame-consistent delta."""
        snapshot = self.robot_tcp_snapshot
        if snapshot is None or time.monotonic() - self.robot_tcp_received > 30.0:
            self.robot_tcp_labels["hmi"].setText("UNKNOWN：等待新鲜示教器 TCP，不使用旧标定替代")
            self.robot_tcp_labels["difference"].setText("UNKNOWN")
            return
        try:
            record = json.loads(self.tcp_calibration_active_file.read_text(encoding="utf-8"))
            T = validate_controller_tcp_record(record, self.tcp_fk_model, self.tcp_fk_sha256)
            frames = self.tcp_fk_model.link_transforms_in_base(np.zeros(6))
            controller_T = np.linalg.inv(np.linalg.inv(frames["flange"]) @ frames["tool0"]) @ T
            controller_like = transform_to_ur_pose(controller_T)
            measured = np.asarray(snapshot["tcp_offset"], dtype=float)
            delta_xyz = float(np.linalg.norm(controller_like[:3] - measured[:3]) * 1000.0)
            relative = controller_T[:3, :3].T @ ur_pose_to_transform(measured)[:3, :3]
            delta_rpy = float(np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))))
            self.robot_tcp_labels["hmi"].setText(
                self._format_tcp6(controller_like)
                + "\n来源：示教器当前 TCP 自动同步（完整 6D）；非第二套标定"
            )
            self.robot_tcp_labels["difference"].setText(
                f"位置 {delta_xyz:.3f} mm；姿态 {delta_rpy:.3f}°"
                + ("（一致）" if delta_xyz < 0.001 and delta_rpy < 0.001 else "（STALE：等待自动同步）")
            )
        except Exception as exc:
            self.robot_tcp_labels["hmi"].setText(f"转换失败：{exc}")
            self.robot_tcp_labels["difference"].setText("UNKNOWN")

    def _tcp_independent_errors(self) -> list[float]:
        from calibration.tcp_pivot import verify_pivot_sample
        result = self.tcp_calibration_result
        if result is None:
            return []
        if self.tcp_verification_poses:
            return [verify_pivot_sample(sample["T"], result) * 1000.0
                    for sample in self.tcp_verification_poses]
        return list(self.tcp_saved_verification_mm)

    def _tcp_write_reasons(self) -> list[str]:
        reasons = []
        if self.robot_tcp_busy:
            reasons.append("正在读取或写入 TCP")
        if self._motion_busy() or self.goal_active or self._jog_pending or self.planning_busy:
            reasons.append("请等待运动/规划任务结束")
        result = self.tcp_calibration_result
        if result is None or not result.valid:
            reasons.append("需要有效的 TCP 标定")
        elif self.tcp_calibration_poses:
            quality = calibration_write_quality([s["T"] for s in self.tcp_calibration_poses], result)
            if not quality["pass"]:
                reasons.append(quality["reason"])
        # Independent verification is optional for the ordinary four-point
        # calibration save/apply workflow.  A real-robot write is still
        # protected by the strict rank/conditioning/residual/uncertainty
        # checks above, a fresh RTDE snapshot, stationary state and the
        # explicit confirmation dialog below.
        snap = self.robot_tcp_snapshot
        if snap is None or time.monotonic() - self.robot_tcp_received > 30.0:
            reasons.append("请重新读取真机 TCP（快照有效期 30 s）")
        else:
            try:
                alignment = validate_rtde_model_alignment(self.tcp_fk_model, snap)
                if not alignment["pass"]:
                    reasons.append("URDF 与控制器坐标核对未通过")
            except Exception as exc:
                reasons.append(f"真机快照无效：{exc}")
            if snap.get("robot_mode") != 7 or snap.get("safety_mode") != 1:
                reasons.append("需要机器人 RUNNING 且安全 NORMAL")
            if snap.get("runtime_state") != 1:
                reasons.append("请在示教器手动停止程序后重新读取 TCP")
            velocities = np.asarray(snap.get("actual_qd", []), dtype=float)
            if velocities.shape != (6,) or not np.isfinite(velocities).all() or np.max(np.abs(velocities)) >= 0.005:
                reasons.append("机器人尚未静止")
        return reasons

    def _update_robot_tcp_controls(self) -> None:
        if not hasattr(self, "robot_tcp_labels"):
            return
        if self.robot_tcp_sync_stamp != self.robot_tcp_received and self.robot_tcp_snapshot is not None:
            self._sync_controller_tcp_to_planning()
        if self.robot_tcp_snapshot is None or time.monotonic() - self.robot_tcp_received > 30.0:
            self.robot_tcp_labels["state"].setText("UNKNOWN：等待读取新鲜示教器 TCP；规划禁止使用历史值替代")
        locked = (self.robot_tcp_busy or self._motion_busy() or self.goal_active
                  or self._jog_pending or self.planning_busy)
        self.tcp_robot_read_button.setEnabled(not locked)
        self.tcp_saved_load_button.setEnabled(not locked and not self.planning_busy)
        if hasattr(self, "tcp_pendant_candidate_button"):
            candidate_reasons = self._pendant_candidate_reasons()
            self.tcp_pendant_candidate_button.setEnabled(not candidate_reasons)
            self.tcp_pendant_candidate_button.setToolTip(
                "；".join(candidate_reasons) if candidate_reasons
                else "只读转换控制器当前完整 6D TCP 到 URDF flange，生成候选"
            )
            self.tcp_pendant_save_button.setEnabled(
                bool(self.tcp_pendant_candidate)
                and self.tcp_pendant_candidate.get("status") == "CANDIDATE"
                and not locked
            )
            self.tcp_pendant_apply_button.setEnabled(
                bool(self.tcp_pendant_candidate)
                and self.tcp_pendant_candidate.get("status") == "CONFIRMED"
                and not locked
            )
        reasons = self._tcp_write_reasons()
        self.tcp_robot_write_button.setEnabled(not reasons)
        self.tcp_robot_write_button.setToolTip("；".join(reasons) if reasons else "人工确认后写入并 RTDE 读回核验")
        if self.tcp_calibration_result is not None and self.tcp_calibration_poses:
            try:
                quality = calibration_write_quality([s["T"] for s in self.tcp_calibration_poses], self.tcp_calibration_result)
                prefix = "PASS" if quality["pass"] else "BLOCKED"
                self.robot_tcp_labels["quality"].setText(
                    f"{prefix}：XYZ 偏移估计标准差 {quality['std_xyz_mm']} mm。{quality['reason']}"
                )
            except Exception as exc:
                self.robot_tcp_labels["quality"].setText(f"BLOCKED：{exc}")
        else:
            self.robot_tcp_labels["quality"].setText("等待标定；真机写入仍需严格质量门与人工确认")
        self._update_robot_tcp_comparison()
        if self.robot_tcp_snapshot is not None and self.tcp_calibration_result is not None:
            try:
                target = flange_offset_to_controller_tcp(self.tcp_fk_model,
                    self.tcp_calibration_result.tcp_flange_m, self.robot_tcp_snapshot["tcp_offset"])
                self.robot_tcp_labels["proposed"].setText(self._format_tcp6(target) + "\n姿态保留真机当前设置")
            except Exception as exc:
                self.robot_tcp_labels["proposed"].setText(f"转换失败：{exc}")

    def _pendant_candidate_reasons(self) -> list[str]:
        snapshot = self.robot_tcp_snapshot
        reasons = []
        if snapshot is None:
            reasons.append("尚未读取控制器当前 TCP")
        elif time.monotonic() - self.robot_tcp_received > 30.0:
            reasons.append("TCP 快照已超过 30 s，请重新读取")
        if self.robot_tcp_busy:
            reasons.append("TCP 读取仍在进行")
        if self._motion_busy() or self.goal_active or self._jog_pending or self.planning_busy:
            reasons.append("请等待运动或规划任务结束")
        if snapshot is not None:
            try:
                alignment = validate_rtde_model_alignment(self.tcp_fk_model, snapshot)
                if not alignment["pass"]:
                    reasons.append("URDF 与控制器 TCP 坐标核对未通过")
                controller_snapshot_to_flange_tcp(self.tcp_fk_model, snapshot)
            except Exception as exc:
                reasons.append(f"不能转换到 URDF flange：{exc}")
        return reasons

    def _make_pendant_tcp_candidate(self) -> None:
        """Compatibility action: use the automatic controller mirror only."""
        self._sync_controller_tcp_to_planning()

    def _save_pendant_tcp_candidate(self) -> None:
        """Compatibility action: use the automatic controller mirror only."""
        self._sync_controller_tcp_to_planning()

    def _apply_pendant_tcp_candidate(self) -> None:
        """Compatibility action: use the automatic controller mirror only."""
        self._sync_controller_tcp_to_planning()

    def _load_saved_tcp_fit(self) -> None:
        try:
            payload = json.loads(self.tcp_calibration_file.read_text(encoding="utf-8"))
            warning = None
            try:
                check = validate_saved_pivot_payload(payload, self.tcp_fk_model, self.tcp_fk_sha256)
            except Exception as exc:
                # A previous session may contain useful q_actual samples whose
                # old fit no longer passes today's stricter write gate (for
                # example max residual >2 mm or no persisted independent
                # sample). Restore those samples for continued calibration,
                # but never label them write-ready.
                warning = str(exc)
                if payload.get("urdf_sha256") != self.tcp_fk_sha256:
                    raise
                samples = payload.get("pose_samples")
                if not isinstance(samples, list) or not 4 <= len(samples) <= 6:
                    raise
                check = {"independent_verification_recomputed": False}
            self.tcp_calibration_poses = [
                {"q": np.asarray(row["q_rad"], dtype=float),
                 "T": self.tcp_fk_model.link_transforms_in_base(row["q_rad"])["flange"],
                 "timestamp": str(row.get("timestamp", "saved"))}
                for row in payload["pose_samples"]
            ]
            self.tcp_verification_poses = [
                {"q": np.asarray(row["q_rad"], dtype=float),
                 "T": self.tcp_fk_model.link_transforms_in_base(row["q_rad"])["flange"],
                 "timestamp": str(row.get("timestamp", "saved"))}
                for row in payload.get("verification_samples", [])
            ]
            self.tcp_verify_capture_button.setText(
                f"可选：采集验证姿态（{len(self.tcp_verification_poses)}）"
            )
            self.tcp_saved_verification_mm = list(payload.get("independent_verification_mm", []))
            self.tcp_saved_verification_recomputed = check["independent_verification_recomputed"]
            self._render_tcp_pose_table()
            self._compute_tcp_calibration()
            errors = self._tcp_independent_errors()
            if errors:
                self.tcp_result_labels["independent"].setText(
                    f"{'PASS' if max(errors) <= 2 else 'FAIL'}（保存记录）；最大 {max(errors):.3f} mm"
                )
                self.tcp_apply_button.setEnabled(
                    not self._tcp_application_reasons(self.tcp_calibration_result)
                )
            if warning:
                self.tcp_result_labels["status"].setText("LOADED（仅恢复姿态，需重新验证；禁止写入真机）")
                self.tcp_result_labels["status"].setStyleSheet("color:#8a6d00;font-weight:bold;")
                self.robot_tcp_labels["state"].setText("已恢复历史姿态；当前质量门未通过，不能写入真机")
                self._log("已恢复历史 TCP 姿态，但保存结果未通过当前质量门：" + warning)
            else:
                self._log("已加载保存的 TCP 标定并由原始关节样本重新求解；没有写入机器人")
            self._update_robot_tcp_controls()
        except Exception as exc:
            self._log(f"加载 TCP 标定失败：{exc}")
            self.robot_tcp_labels["state"].setText(f"加载失败：{exc}")

    def _read_robot_tcp(self) -> None:
        if self.robot_tcp_busy or self._motion_busy() or self.goal_active or self._jog_pending:
            return
        self._launch_robot_tcp_helper("read")

    def _launch_robot_tcp_helper(self, mode: str, request_path=None) -> None:
        helper = HMI_ROOT / "bin" / "robot_tcp_helper"
        source = HMI_ROOT / "native" / "robot_tcp_helper.cpp"
        rebuild = not helper.is_file() or (source.is_file() and source.stat().st_mtime > helper.stat().st_mtime)
        self.robot_tcp_busy = True
        self.robot_tcp_mode = "build" if rebuild else mode
        self._robot_tcp_next = (mode, request_path)
        process = QProcess(self)
        self.robot_tcp_process = process
        process.setProcessChannelMode(QProcess.SeparateChannels)
        process.finished.connect(self._robot_tcp_finished)
        process.errorOccurred.connect(self._robot_tcp_process_error)
        if rebuild:
            self.robot_tcp_labels["state"].setText("正在准备官方 UR 客户端…")
            process.start("/bin/bash", [str(HMI_ROOT / "native" / "build_robot_tcp_helper.sh")])
        else:
            ip = str(self.config.get("ros", {}).get("robot_ip", "192.168.0.10"))
            args = ["--robot-ip", ip]
            if request_path is not None:
                args.extend(["--request", str(request_path)])
            self.robot_tcp_labels["state"].setText("读取中…" if mode == "read" else "写入中：等待真机读回核验…")
            process.start(str(helper), args)
        self._update_robot_tcp_controls()
        self._update_buttons()

    def _robot_tcp_process_error(self, error) -> None:
        if error == QProcess.FailedToStart:
            if self.robot_tcp_process is not None:
                self.robot_tcp_process.deleteLater()
                self.robot_tcp_process = None
            self.robot_tcp_busy = False
            self.robot_tcp_snapshot = None
            self.robot_tcp_received = 0.0
            self.robot_tcp_sync_error = "UNKNOWN：TCP 客户端启动失败"
            self.robot_tcp_labels["state"].setText("TCP 客户端启动失败，未写入")
            self._log("TCP 客户端启动失败；检查 native/build_robot_tcp_helper.sh")
            self._update_robot_tcp_controls()
            self._update_buttons()

    def _robot_tcp_finished(self, code: int, _status) -> None:
        process = self.robot_tcp_process
        if process is None:
            return
        output = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        diagnostic = bytes(process.readAllStandardError()).decode("utf-8", errors="replace")
        mode = self.robot_tcp_mode
        self.robot_tcp_process = None
        self.robot_tcp_busy = False
        process.deleteLater()
        normal_exit = _status == QProcess.NormalExit
        if mode == "build":
            if code == 0 and normal_exit:
                next_mode, request_path = self._robot_tcp_next
                self._launch_robot_tcp_helper(next_mode, request_path)
            else:
                self.robot_tcp_snapshot = None
                self.robot_tcp_received = 0.0
                self.robot_tcp_sync_error = "UNKNOWN：TCP 客户端构建失败"
                self.robot_tcp_labels["state"].setText("客户端构建失败：未连接/写入机器人")
                self._log("TCP 客户端构建失败：" + diagnostic[-1200:])
                self._update_robot_tcp_controls()
            return
        try:
            payload = json.loads(output)
        except Exception:
            payload = {"success": False, "reason": "客户端没有返回完整结果", "diagnostic": diagnostic[-1200:]}
        if mode == "apply" and self.robot_tcp_report_path is not None:
            self.robot_tcp_report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        if code == 0 and normal_exit and payload.get("success") is True:
            self.robot_tcp_auto_last_error = ""
            snapshot = payload.get("snapshot") if mode == "read" else payload.get("after")
            previous = None
            changed = False
            if snapshot is not None:
                if mode == "read" and hasattr(self, "_accept_tool_payload_read"):
                    self._accept_tool_payload_read(payload)
                previous = self.robot_tcp_snapshot
                changed = bool(
                    previous is not None
                    and not np.allclose(previous["tcp_offset"], snapshot["tcp_offset"], atol=1e-7, rtol=0)
                )
                if changed:
                    self._invalidate_tcp_trajectories()
                self.robot_tcp_snapshot = snapshot
                self.robot_tcp_received = time.monotonic()
                # Keep the red LIVE TCP independent from the ROS topic.  On
                # CB3 installations tcp_pose_broadcaster may be configured
                # but have no publisher; the same read-only RTDE packet still
                # contains actual_TCP_pose and actual_q.  This never writes
                # TCP parameters and never changes preview state.
                try:
                    actual_pose = np.asarray(snapshot.get("actual_TCP_pose", []), dtype=float)
                    actual_q = np.asarray(snapshot.get("actual_q", []), dtype=float)
                    setter = getattr(self.robot_view, "set_controller_tcp_pose", None)
                    if callable(setter) and actual_pose.size >= 3 and actual_q.size == 6:
                        setter(actual_pose[:3], actual_q.tolist(), "base")
                        refresher = getattr(self, "_refresh_scene_status_label", None)
                        if callable(refresher):
                            refresher()
                except Exception as exc:
                    self._log("RTDE TCP 显示同步失败（不影响只读数据）：" + str(exc))
                current_text = self._format_tcp6(snapshot["tcp_offset"])
                actual_pose = np.asarray(snapshot.get("actual_TCP_pose", []), dtype=float)
                if actual_pose.size >= 3 and np.all(np.isfinite(actual_pose[:3])):
                    current_text += ("\n实际 TCP（controller_base）XYZ "
                                     "[%.3f, %.3f, %.3f] mm"
                                     % tuple(actual_pose[:3] * 1000.0))
                self.robot_tcp_labels["current"].setText(
                    current_text
                    + f"\n更新时间：{time.strftime('%H:%M:%S')}（RTDE 控制器当前生效 TCP）"
                )
                try:
                    alignment = validate_rtde_model_alignment(self.tcp_fk_model, snapshot)
                    self.robot_tcp_labels["alignment"].setText(
                        f"{'PASS' if alignment['pass'] else 'FAIL'}："
                        f"{alignment['position_error_mm']:.3f} mm / {alignment['orientation_error_deg']:.3f}°"
                    )
                except Exception as exc:
                    self.robot_tcp_labels["alignment"].setText(f"FAIL：{exc}")
            if mode == "apply":
                self._invalidate_tcp_trajectories()
                self.robot_view.clear_calibrated_tcp_offset()
                self.robot_tcp_labels["state"].setText("PASS：真机当前 TCP 读回一致；仅当前运行生效，未保存安装配置")
                self._log("[PASS] 真机 TCP 参数写入与读回一致；RX/RY/RZ 保持；旧轨迹已清除。" +
                          f"审计记录：{self.robot_tcp_report_path}")
            else:
                self._sync_controller_tcp_to_planning()
                # Automatic polling is intentionally quiet; log the initial
                # snapshot and any active-TCP change, not one line every 5 s.
                if (not self.robot_tcp_auto_sync or previous is None or changed):
                    suffix = "；检测到 active tcp_offset 变化，旧轨迹已标记失效" if changed else ""
                    self._log(
                        "已只读获取真机 tcp_offset / actual_TCP_pose / actual_q，"
                        "同一 RTDE 数据包；未修改真机" + suffix
                    )
        else:
            reason = str(payload.get("reason", "未知错误"))
            if hasattr(self, "_invalidate_tool_payload_read"):
                self._invalidate_tool_payload_read()
            # A read failure cannot be presented as a live synchronized TCP.
            self.robot_tcp_snapshot = None
            self.robot_tcp_received = 0.0
            self.robot_tcp_sync_error = "UNKNOWN：" + reason
            self.robot_tcp_labels["current"].setText("UNKNOWN：控制器 TCP 读取失败")
            if mode == "apply":
                # A failed ACK is never treated as proof that no change took place.
                self._invalidate_tcp_trajectories()
                self.robot_tcp_snapshot = None
                self.robot_tcp_received = 0.0
                self.robot_tcp_labels["current"].setText("写入结果未确认：请重新读取真机 TCP")
            self.robot_tcp_labels["state"].setText("FAIL：" + reason)
            if (not self.robot_tcp_auto_sync) or reason != self.robot_tcp_auto_last_error:
                self._log("TCP 参数操作失败：" + reason)
            self.robot_tcp_auto_last_error = reason
        self._update_robot_tcp_controls()
        self._update_buttons()

    def _write_robot_tcp(self) -> None:
        reasons = self._tcp_write_reasons()
        if reasons:
            self._log("真机 TCP 写入被阻止：" + "；".join(reasons))
            return
        old = np.asarray(self.robot_tcp_snapshot["tcp_offset"], dtype=float)
        target = flange_offset_to_controller_tcp(self.tcp_fk_model, self.tcp_calibration_result.tcp_flange_m, old)
        answer = QMessageBox.question(self, "确认修改真机当前 TCP",
            "旧 TCP：\n" + self._format_tcp6(old) + "\n\n新 TCP：\n" + self._format_tcp6(target) +
            "\n\n只修改位置，保留真机工具姿态。机器人程序须已手动停止，机械臂静止。"
            "\n写入后会清除旧轨迹，要求重新规划/验证。"
            "\n本次只设置当前激活 TCP，不保存示教器安装默认值。\n确认写入？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        # Check again after the modal dialog; helper independently rechecks
        # mode, safety, stationary speed and the original six TCP parameters.
        reasons = self._tcp_write_reasons()
        if reasons:
            self._log("确认后状态已变化，TCP 未写入：" + "；".join(reasons))
            return
        directory = HMI_ROOT / "logs" / "tcp_changes" / (time.strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
        directory.mkdir(parents=True, exist_ok=False)
        backup_path = directory / "before.json"
        backup_path.write_text(json.dumps({"snapshot": self.robot_tcp_snapshot,
                                           "urdf_sha256": self.tcp_fk_sha256}, indent=2), encoding="utf-8")
        request_path = directory / "request.json"
        request = {"target_tcp_offset": target.tolist(), "expected_current_tcp": old.tolist(),
                   "confirmed": True, "request_id": directory.name, "backup_path": str(backup_path),
                   "urdf_sha256": self.tcp_fk_sha256, "calibration": self._tcp_payload()}
        request_path.write_text(json.dumps(request, ensure_ascii=False, indent=2), encoding="utf-8")
        self.robot_tcp_report_path = directory / "result.json"
        self._invalidate_tcp_trajectories()
        self._launch_robot_tcp_helper("apply", request_path)

    def _invalidate_tcp_trajectories(self) -> None:
        self._stop_preview()
        self.loaded_trajectory = None
        self.loaded_path = None
        self.trajectory_validation = None
        self.loaded_path_name = "TCP 已更新：需重新规划与碰撞验证"
        self.path_label.setText(self.loaded_path_name)
        self.preview_status_label.setText("TCP 已更新：旧轨迹失效")
        self.robot_view.clear_planned_tcp_path()
        self._update_trajectory_gate()
