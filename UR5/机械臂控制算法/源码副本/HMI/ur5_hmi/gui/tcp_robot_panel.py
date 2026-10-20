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
        self.tcp_robot_read_button = QPushButton("读取 UR5 当前 TCP")
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
        self.tcp_robot_auto_sync_box = QCheckBox("自动同步示教器 TCP（只读）")
        self.tcp_robot_auto_sync_box.setChecked(True)
        self.tcp_robot_auto_sync_box.toggled.connect(self._set_tcp_auto_sync)
        grid.addWidget(self.tcp_robot_auto_sync_box, 0, 3)
        grid.addWidget(self.tcp_pendant_candidate_button, 1, 0, 1, 2)
        grid.addWidget(self.tcp_pendant_save_button, 1, 2)
        grid.addWidget(self.tcp_pendant_apply_button, 1, 3)
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
            "不是软件 marker；\"HMI 已应用 TCP（规划模型）\"是上次人工确认的规划版本，不会被只读同步自动覆盖。"
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
        self._launch_robot_tcp_helper("read")

    @staticmethod
    def _format_tcp6(values) -> str:
        p = np.asarray(values, dtype=float)
        return ("XYZ [%.3f, %.3f, %.3f] mm\nRX/RY/RZ [%.6f, %.6f, %.6f] rad"
                % tuple(np.r_[p[:3] * 1000.0, p[3:]]))

    def _hmi_applied_flange_offset(self):
        """Return the HMI's approved flange offset and its source.

        The HMI calibration file stores position in URDF flange coordinates;
        the controller ``tcp_offset`` stores the same tool point in its
        tool0/controller representation.  Conversion is done only after a
        fresh controller snapshot is available.  HMI position-only
        calibration intentionally reuses the controller's current rotation.
        """
        try:
            active = getattr(self, "tcp_calibration_active_file", None)
            approved = getattr(self, "tcp_calibration_applied_file", None)
            paths = [p for p in (active, approved) if p is not None]
            for path in paths:
                if not path.is_file():
                    continue
                payload = json.loads(path.read_text(encoding="utf-8"))
                value = np.asarray(payload.get("position_flange_tcp_m"), dtype=float).reshape(-1)
                if (payload.get("valid") and payload.get("applied_to_planning_model")
                        and payload.get("active_for_planning", True)
                        and value.shape == (3,) and np.all(np.isfinite(value))):
                    return value, path.name
        except Exception:
            pass
        try:
            value = np.asarray(self._existing_flange_tcp_offset(), dtype=float).reshape(-1)
            if value.shape == (3,) and np.all(np.isfinite(value)):
                return value, "frozen ur5_tool_calibration.mat baseline"
        except Exception:
            pass
        return None, "unavailable"

    def _update_robot_tcp_comparison(self) -> None:
        """Render controller TCP, HMI TCP and their frame-consistent delta."""
        snapshot = self.robot_tcp_snapshot
        if snapshot is None:
            self.robot_tcp_labels["hmi"].setText("等待 HMI TCP 标定")
            self.robot_tcp_labels["difference"].setText("—")
            return
        offset_flange, source = self._hmi_applied_flange_offset()
        if offset_flange is None:
            self.robot_tcp_labels["hmi"].setText("未找到有效 HMI TCP 标定")
            self.robot_tcp_labels["difference"].setText("UNKNOWN")
            return
        try:
            controller_like = flange_offset_to_controller_tcp(
                self.tcp_fk_model, offset_flange, snapshot["tcp_offset"]
            )
            measured = np.asarray(snapshot["tcp_offset"], dtype=float)
            delta_xyz = float(np.linalg.norm(controller_like[:3] - measured[:3]) * 1000.0)
            # HMI's approved record is position-only and explicitly preserves
            # the controller's active RX/RY/RZ; therefore this orientation
            # comparison is zero by policy, not an invented independent fit.
            delta_rpy = 0.0
            self.robot_tcp_labels["hmi"].setText(
                self._format_tcp6(controller_like)
                + f"\n来源：{source}；姿态沿用控制器当前值"
            )
            self.robot_tcp_labels["difference"].setText(
                f"位置 {delta_xyz:.3f} mm；姿态 {delta_rpy:.3f}°"
                + ("（一致）" if delta_xyz <= 1.0 else "（需人工确认）")
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
        reasons = self._pendant_candidate_reasons()
        if reasons:
            self.robot_tcp_labels["state"].setText("候选生成被阻止：" + "；".join(reasons))
            self._log("示教器 TCP 候选未生成：" + "；".join(reasons))
            return
        try:
            converted = controller_snapshot_to_flange_tcp(self.tcp_fk_model, self.robot_tcp_snapshot)
            candidate = {
                "schema_version": 1,
                "calibration_type": "controller_active_tcp_adoption_6d",
                "coordinate_frames": {
                    "base": "URDF base",
                    "flange": "URDF flange",
                    "tcp": "controller active TCP / proposed real welding TCP",
                },
                "source": "UR5 controller active tcp_offset + actual_TCP_pose (same RTDE snapshot)",
                "controller_tcp_offset": self.robot_tcp_snapshot["tcp_offset"],
                "controller_actual_tcp_pose_base": self.robot_tcp_snapshot["actual_TCP_pose"],
                "actual_q_rad": self.robot_tcp_snapshot["actual_q"],
                "T_flange_tcp": converted["T_flange_tcp"],
                "position_flange_tcp_m": converted["position_flange_tcp_m"],
                "position_flange_tcp_mm": (np.asarray(converted["position_flange_tcp_m"]) * 1000.0).tolist(),
                "rotation_flange_tcp_rotvec_rad": converted["rotation_flange_tcp_rotvec_rad"],
                "rotation_source": converted["rotation_source"],
                "urdf_sha256": self.tcp_fk_sha256,
                "status": "CANDIDATE",
                "valid": True,
                "verified_tcp_at_wire_tip": False,
                "operator_confirmation_required": True,
                "applied_to_planning_model": False,
                "requires_replan": True,
                "requires_collision_revalidation": True,
                "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "version": "UR5-HMI-PENDANT-TCP-6D-1",
            }
            self.tcp_pendant_candidate = candidate
            self.tcp_pendant_save_button.setEnabled(True)
            self.tcp_pendant_apply_button.setEnabled(False)
            self.robot_tcp_labels["proposed"].setText(
                self._format_tcp6(np.r_[converted["position_flange_tcp_m"],
                                          converted["rotation_flange_tcp_rotvec_rad"]])
                + "\n来源：控制器当前 active TCP；坐标已转换到 URDF flange；待确认焊丝尖端定义"
            )
            self.robot_tcp_labels["state"].setText(
                "已生成候选：未保存、未应用；请核实示教器 TCP 是否实际定义在焊丝尖端"
            )
            self._log("已生成示教器当前 TCP 的 URDF flange 6D 候选；未修改旧标定、MATLAB 或真机")
        except Exception as exc:
            self._log(f"示教器 TCP 候选生成失败：{exc}")
            self.robot_tcp_labels["state"].setText(f"候选生成失败：{exc}")

    def _save_pendant_tcp_candidate(self) -> None:
        candidate = self.tcp_pendant_candidate
        if not candidate:
            return
        # Auto-sync is read-only but may have observed a newer active TCP
        # between candidate generation and the modal confirmation.  Never
        # save a stale candidate as if it were the current pendant value.
        snapshot = self.robot_tcp_snapshot
        if (snapshot is None or time.monotonic() - self.robot_tcp_received > 30.0
                or not (
                np.allclose(np.asarray(candidate.get("controller_tcp_offset", []), dtype=float),
                            np.asarray(snapshot.get("tcp_offset", []), dtype=float), atol=1e-7, rtol=0)
                and np.allclose(np.asarray(candidate.get("actual_q_rad", []), dtype=float),
                                np.asarray(snapshot.get("actual_q", []), dtype=float), atol=1e-7, rtol=0))):
            self.tcp_pendant_candidate = None
            self.tcp_pendant_save_button.setEnabled(False)
            self.robot_tcp_labels["state"].setText("候选已过期：控制器 TCP 已更新，请重新生成")
            self._log("示教器 TCP 候选未保存：检测到 RTDE 快照已变化")
            return
        answer = QMessageBox.question(
            self, "确认保存示教器当前 TCP 新版本",
            "请先确认：示教器当前生效 TCP 的参考点确实是焊丝/导电嘴尖端，\n"
            "而不是法兰、喷嘴外壳或其他工具参考点。\n\n"
            "确认后只保存独立版本，不覆盖旧 real_tcp_calibration_applied.json、\n"
            "冻结 ur5_tool_calibration.mat，也不会修改示教器或发送运动。\n"
            "只有随后再次确认“应用到真实规划模型”时，依赖旧 TCP 的轨迹才会标记为 STALE。\n\n确认保存？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        try:
            version_dir = HMI_ROOT.parent / "real_tcp_calibration_versions"
            version_dir.mkdir(parents=True, exist_ok=True)
            name = "tcp_from_pendant_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8] + ".json"
            path = version_dir / name
            saved = dict(candidate)
            saved["status"] = "CONFIRMED"
            saved["confirmed_by_operator"] = True
            saved["verified_tcp_at_wire_tip"] = True
            saved["confirmed_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            saved["version_file"] = str(path)
            path.write_text(json.dumps(saved, ensure_ascii=False, indent=2), encoding="utf-8")
            latest = HMI_ROOT.parent / "real_tcp_calibration_from_pendant_latest.json"
            latest.write_text(json.dumps(saved, ensure_ascii=False, indent=2), encoding="utf-8")
            self.tcp_pendant_candidate = saved
            self.tcp_pendant_candidate_path = path
            self.tcp_pendant_save_button.setEnabled(False)
            self.tcp_pendant_apply_button.setEnabled(True)
            self.robot_tcp_labels["state"].setText(
                f"已保存 CONFIRMED 新版本：{path.name}；尚未应用到规划模型"
            )
            self._log(f"示教器 TCP 已保存为独立版本：{path}；旧标定未覆盖，尚未应用")
        except Exception as exc:
            QMessageBox.critical(self, "保存示教器 TCP 版本失败", str(exc))

    def _apply_pendant_tcp_candidate(self) -> None:
        candidate = self.tcp_pendant_candidate
        if not candidate or candidate.get("status") != "CONFIRMED":
            return
        active_path = getattr(self, "tcp_calibration_active_file", HMI_ROOT.parent / "real_tcp_calibration_active.json")
        answer = QMessageBox.question(
            self, "二次确认：应用示教器 TCP 到真实规划模型",
            "这一步只更新 HMI 与 MATLAB 真实规划的 TCP 输入。\n"
            "不会写入示教器 TCP，不会移动机器人。\n"
            "应用后旧轨迹立即 STALE，必须重新做焊丝几何、工件/环境碰撞和轨迹验证。\n\n"
            f"新版本：{candidate.get('version_file', '独立候选')}\n"
            f"flange TCP XYZ：{candidate.get('position_flange_tcp_mm')} mm\n"
            "确认焊丝尖端已通过实物核对，并继续应用？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        payload = dict(candidate)
        payload["status"] = "ACTIVE"
        payload["active_for_planning"] = True
        payload["applied_to_planning_model"] = True
        payload["applied_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        payload["requires_replan"] = True
        payload["requires_collision_revalidation"] = True
        active_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self.tcp_pendant_candidate = payload
        self._invalidate_tcp_trajectories()
        if hasattr(self, "_mark_trajectory_stale"):
            self._mark_trajectory_stale("控制器示教器 TCP 新版本已应用")
        if hasattr(self.robot_view, "set_calibrated_tcp_offset"):
            self.robot_view.set_calibrated_tcp_offset(np.asarray(payload["position_flange_tcp_m"], dtype=float))
        self._update_robot_tcp_controls()
        self._log(f"示教器 TCP 已应用到 HMI/MATLAB 真实规划输入：{active_path}；旧轨迹 STALE；未写入真机")

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
                self.robot_tcp_labels["state"].setText("已读取 UR5 当前 TCP；未修改任何参数")
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

