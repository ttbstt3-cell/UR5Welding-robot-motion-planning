"""Two-stage offline planning. No ROS commands are emitted from this module."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import hashlib
from pathlib import Path
import time
import os
import xml.etree.ElementTree as ET

import numpy as np
from PyQt5.QtCore import QProcess


class PlanningStage(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING PREVIEW"
    PREVIEW_READY = "Preview Ready"
    PREVIEW_WARNING = "PREVIEW WARNING"
    CONTINUOUS_PLANNING = "CONTINUOUS_PLANNING"
    PREPARING = "CONTINUOUS_PLANNING"  # Compatibility alias, not execution approval.
    PHYSICAL_VALIDATION_PENDING = "PHYSICAL_VALIDATION_PENDING"
    EXECUTION_READY = "Execution Ready"
    PREVIEW_FAILED = "PREVIEW FAILED"
    CONTINUOUS_PLANNING_FAILED = "CONTINUOUS_PLANNING_FAILED"
    PREPARATION_FAILED = "CONTINUOUS_PLANNING_FAILED"  # Compatibility alias.
    CANCELED = "CANCELED"


@dataclass
class PlanningWorkflow:
    stage: PlanningStage = PlanningStage.IDLE
    raw_path: Path | None = None
    ready_path: Path | None = None
    optimized_path: Path | None = None
    reason: str = ""

    def begin_preview(self):
        self.stage = PlanningStage.PLANNING
        self.raw_path = self.ready_path = self.optimized_path = None
        self.reason = ""

    def preview_complete(self, path: Path, report: dict):
        if not path.is_file() or not (report.get("preview_available") or report.get("previewReady")):
            raise ValueError(report.get("failure_reason", "No complete preview path"))
        self.raw_path = path
        passed = (report.get("collision_status") == "PASS"
                  and report.get("collision_check_pass") is True)
        self.stage = PlanningStage.PREVIEW_READY if passed else PlanningStage.PREVIEW_WARNING
        self.reason = "" if passed else "WARNING：碰撞或显示一致性审计未通过/未完成"

    def begin_preparation(self):
        if self.raw_path is None or not self.raw_path.is_file():
            raise ValueError("请先生成 full_raw_trajectory 预览轨迹")
        self.stage = PlanningStage.PREPARING
        self.ready_path = self.optimized_path = None
        self.reason = ""

    def preparation_complete(self, path: Path, report: dict):
        """Accept offline evidence only; this workflow cannot grant execution."""
        audit = report.get("controller_validation", {})
        binding = report.get("version_validation", {})
        checks = report.get("offline_checks", {})
        required = ("interface_c2", "tcp_contour", "tcp_time_sync", "tcp_orientation",
                    "velocity", "acceleration", "controller_interpolation", "collision",
                    "joint_limits", "versions")
        accepted = (report.get("executionReady") is False and path.is_file()
                    and report.get("success") is True
                    and report.get("continuous_planning_complete") is True
                    and report.get("continuous_planning_status") == "PASS"
                    and report.get("stage") == PlanningStage.PHYSICAL_VALIDATION_PENDING.value
                    and report.get("physical_validation_status") == "PENDING"
                    and report.get("servo_validation_status") == "PENDING"
                    and all(checks.get(key) is True for key in required)
                    and report.get("continuity_scope", {}).get("interface_c2_pass") is True
                    and audit.get("complete") is True
                    and audit.get("collision_pass") is True
                    and audit.get("joint_limits_pass") is True
                    and audit.get("analytic_extrema_pass") is True
                    and audit.get("interpolation_mode") == "positions_plus_velocities_cubic_hermite"
                    and int(audit.get("sample_count", 0)) >= 5000
                    and binding.get("pass") is True)
        # Independently check scalar metrics, so inconsistent summary flags
        # cannot turn a real numerical failure into a completed offline stage.
        velocity_limit, acceleration_limit = np.deg2rad(5), .261799
        if report.get("motion_limits") is not None:
            try:
                from safety.robot_motion_limits import validate_contract
                limits = validate_contract(report["motion_limits"])
                velocity_limit, acceleration_limit = limits["scalar_velocity_rad_s"], limits["scalar_acceleration_rad_s2"]
            except (ValueError, OSError, KeyError, TypeError):
                accepted = False
        for key, limit in (("max_tcp_contour_error_mm", .5), ("max_tcp_time_sync_error_mm", .5),
                           ("max_tcp_orientation_error_deg", .5),
                           ("max_joint_velocity_rad_per_sec", velocity_limit),
                           ("max_joint_acceleration_rad_per_sec2", acceleration_limit)):
            value = audit.get(key, float("nan"))
            try:
                number = float(value)
                strict = key in ("max_tcp_contour_error_mm", "max_tcp_time_sync_error_mm")
                accepted = accepted and np.isfinite(number) and number >= 0 and (number < limit if strict else number <= limit)
            except (ValueError, TypeError):
                accepted = False
        if not accepted:
            raise ValueError(report.get("failure_reason") or "连续规划离线验证未通过或证据不完整")
        self.optimized_path = path
        self.ready_path = None
        self.reason = "连续规划完成（离线验证通过）；真机执行准备：待验证"
        self.stage = PlanningStage.PHYSICAL_VALIDATION_PENDING

    def fail(self, phase: str, reason: str):
        self.stage = (PlanningStage.PREVIEW_FAILED if phase == "preview"
                      else PlanningStage.PREPARATION_FAILED)
        self.reason = reason
        self.ready_path = self.optimized_path = None


class PlanningWorkflowPanel:
    def _trajectory_result_directory(self):
        from gui.main_window import ROOT
        return Path(getattr(self, "trajectory_results_directory", ROOT.parent / "规划文件" / "当前轨迹"))

    def _publish_planning_trajectory(self, phase):
        from trajectory.result_store import publish_result
        raw = self.planning_workflow.raw_path
        parent = self._file_sha256(raw) if phase == "prepare" and raw else None
        try:
            return publish_result(self.planning_output_path, self.planning_report_path,
                                  self._trajectory_result_directory(), phase,
                                  parent_preview_sha256=parent)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # Naming/copying is presentation-only, not a new algorithm gate.
            self._log("易读轨迹副本保存失败，仍载入原始结果：" + str(exc))
            return self.planning_output_path

    def _set_endpoint_states(self, start=None, goal=None):
        for name, status in (("start", start), ("goal", goal)):
            label = getattr(self, f"online_{name}_state_label", None)
            if label is not None and status is not None:
                label.setText(str(status))
                color = "#146c2e" if status == "PASS" else "#9b1c1c" if status == "FAIL" else "#8a6d00"
                label.setStyleSheet(f"color:{color};font-weight:bold;")

    def _consume_endpoint_log_line(self, line):
        for name in ("START", "GOAL"):
            if line.startswith(name + " COLLISION CHECK:"):
                status = "PASS" if line.split(":", 1)[1].strip() == "FREE" else "FAIL"
                self._set_endpoint_states(**{name.lower(): status})
                return True
        return False

    def _start_online_planning(self):
        return self._start_preview_planning()

    def _online_process_finished(self, exit_code, exit_status):
        return self._finish_planning_workflow(exit_code, exit_status)

    def _refresh_online_goal_display(self):
        return self._refresh_preview_goal_display()

    def _refresh_preview_goal_display(self):
        if not hasattr(self, "online_goal_source_label"):
            return
        task = self._online_task()
        targets = {"RealHome_to_PreWeld": "PreWeld", "Weld_Seam": "PreWeld → PostWeld",
                   "PostWeld_to_RealHome": "RealHome", "Full_Welding_Task": "RealHome（经焊缝）"}
        self.online_goal_label.setText(targets.get(task, task))
        real = (hasattr(self, "calibration_source_combo")
                and self.calibration_source_combo.currentText() == "REAL CALIBRATION")
        self.online_goal_source_label.setText(
            "真实场景：验证当前起点 → 当前场景 ACCP 重新生成 PreWeld → CBi-RRT*；不复用旧焊缝缓存" if real else
            "冻结仿真场景：由场景焊缝 + ACCP生成目标；无需真实执行验收")
        self.online_task_status_label.setText("PREVIEW · 可离线规划")
        self.online_task_status_label.setStyleSheet("color:#146c2e;font-weight:bold;")
        free = task != "Weld_Seam"
        weld = task in ("Weld_Seam", "Full_Welding_Task")
        for key in ("free_algorithm_label", "free_algorithm"):
            self.online_field_widgets[key].setVisible(free)
        for key in ("weld_algorithm_label", "weld_algorithm"):
            self.online_field_widgets[key].setVisible(weld)
        self.online_start_combo.setEnabled(task in ("RealHome_to_PreWeld", "Full_Welding_Task"))

    def _workflow_available(self):
        return not (self.planning_busy or self._motion_busy() or self.goal_active
                    or getattr(self, "robot_tcp_busy", False)
                    or getattr(self, "realhome_validation_busy", False))

    def _start_preview_planning(self):
        if not self._workflow_available():
            return
        record = self._saved_real_home_record()
        if record is None:
            self._workflow_failure("preview", "Saved RealHome 不存在或无效")
            return
        home = np.asarray(record["q_rad"], dtype=float)
        if self.online_start_combo.currentData() == "current":
            if not self._feedback_valid(self.state):
                self._workflow_failure("preview", "Current Robot 需要最新 /joint_states；可选择 Saved RealHome 离线规划")
                return
            start = np.asarray(self.state.positions, dtype=float)
        else:
            start = home
        # Snapshot metadata only: there is no execution preflight or anchor read.
        binding = self._current_calibration_binding()
        payload = {"task": self._online_task(), "qStart": start.tolist(), "qHome": home.tolist(),
                   "algorithm": str(self.online_algorithm_combo.currentData()),
                   "seed": int(self.online_seed_spin.value()),
                   "maxPlanningTimeSec": float(self.online_budget_spin.value()),
                   "planning_scene": binding["planning_scene"], "calibration_binding": binding,
                   "preview_only": True, "executionReady": False,
                   "realhome_version": record["realhome_version"],
                   "collision_scene_version": binding["collision_scene_version"],
                   "preweld_version": None, "regenerate_preweld": True,
                   "start_source": str(self.online_start_combo.currentData())}
        self.workflow_request_snapshot = dict(payload)
        self._set_endpoint_states(start="CHECKING", goal="NOT GENERATED")
        if self.loaded_trajectory is not None:
            self._mark_trajectory_stale("已发起新的预览规划，旧加载结果暂不可执行")
        self.planning_workflow.begin_preview()
        self._launch_workflow_process("preview", "plan_hmi_preview_request", payload)

    def _start_continuous_planning(self):
        if not self._workflow_available():
            return
        try:
            from safety.robot_motion_limits import load_limits
            motion_limits = load_limits()
            if self.planning_workflow.raw_path is not None:
                from trajectory.trajectory_loader import load_trajectory
                raw = load_trajectory(str(self.planning_workflow.raw_path))
                current = self._current_calibration_binding()
                record = self._saved_real_home_record()
                if (record is None or raw.metadata.get("realhome_version") != record["realhome_version"]
                        or raw.metadata.get("preweld_version") != current["collision_scene_version"]
                        or raw.metadata.get("collision_scene_version") != current["collision_scene_version"]):
                    raise ValueError("STALE: RealHome / PreWeld / collision scene版本已改变，请重新规划")
            self.planning_workflow.begin_preparation()
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self._workflow_failure("prepare", str(exc))
            return
        payload = {"rawFile": str(self.planning_workflow.raw_path),
                   "motion_limits_file": motion_limits["profile_path"],
                   "motion_limits_sha256": motion_limits["profile_sha256"],
                   "raw_sha256": self._file_sha256(self.planning_workflow.raw_path),
                   "current_binding": self._current_calibration_binding(),
                   # Readiness applies only to this second-stage request. MATLAB
                   # records these reasons at its final version/application gate.
                   "execution_gate_reasons": self._real_planning_readiness_reasons(),
                   "maxBoundaryRefinementDepth": 2, "maxBoundaryIKCalls": 100,
                   "maxBoundaryTimeSec": 30, "minControllerSamples": 5000}
        self._launch_workflow_process("prepare", "prepare_hmi_continuous_request", payload)

    def _launch_workflow_process(self, phase, entry, payload):
        from gui.main_window import ROOT
        request_id = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000:06d}"
        run = ROOT.parent / "规划文件" / "workflow_runs" / f"{phase}_{request_id}"
        run.mkdir(parents=True, exist_ok=False)
        self.workflow_phase = phase
        self.workflow_canceled = False
        self.planning_request_path = run / "request.json"
        self.planning_report_path = run / "report.json"
        self.workflow_log_path = run / "matlab.log"
        self.planning_output_path = run / ("full_raw_trajectory.mat" if phase == "preview" else "offline_optimized_trajectory.mat")
        payload.update(request_id=request_id, outputDir=str(run))
        self.planning_request_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        matlab_dir = ROOT.parent / "matlab仿真"
        benchmark = ROOT.parent.parent / "RRT" / "benchmark"
        expression = (f"addpath({self._matlab_quote(matlab_dir)}); "
                      f"addpath({self._matlab_quote(benchmark)}); "
                      f"req=jsondecode(fileread({self._matlab_quote(self.planning_request_path)})); "
                      f"{entry}(req);")
        self.online_output_view.clear()
        self.online_output_view.appendPlainText(f"{entry}\n{run}")
        self._set_online_stage(self.planning_workflow.stage.value)
        self.online_execution_preparation_label.setText("真机执行准备：待验证（无执行许可）")
        self.online_report_button.setEnabled(False)
        self.online_save_analysis_button.setEnabled(False)
        self.online_status_label.setText("生成仿真预览轨迹…" if phase == "preview" else "检查衔接 / 局部修复 / 控制器验证…")
        process = QProcess(self)
        process.readyReadStandardOutput.connect(lambda: self._online_log_process_output("stdout"))
        process.readyReadStandardError.connect(lambda: self._online_log_process_output("stderr"))
        process.errorOccurred.connect(self._online_process_error)
        process.finished.connect(self._online_process_finished)
        self.planning_process = process
        self.planning_busy = True
        self.continuity_busy = phase == "prepare"
        self._update_buttons()
        self._workflow_process_started_at = time.perf_counter()
        process.start("matlab", ["-batch", expression])

    def _workflow_failure(self, phase, reason):
        self.planning_workflow.fail(phase, reason)
        self.online_status_label.setText(reason)
        self._set_online_stage(self.planning_workflow.stage.value)
        self._log(f"Offline {phase} failed: {reason}")
        self.online_execution_preparation_label.setText("真机执行准备：未通过（无执行许可）")
        self.online_report_button.setEnabled(False)
        self.online_save_analysis_button.setEnabled(False)

    def _finish_planning_workflow(self, exit_code, _exit_status):
        callback_started = time.perf_counter()
        adoption_seconds = None
        self._online_log_process_output("stdout")
        self._online_log_process_output("stderr")
        process = self.planning_process
        self.planning_process = None
        self.planning_busy = self.continuity_busy = False
        if process is not None:
            process.deleteLater()
        phase = self.workflow_phase
        if self.workflow_canceled:
            self.planning_workflow.stage = PlanningStage.CANCELED
            self._set_online_stage("CANCELED")
            self._update_buttons()
            return
        try:
            report = json.loads(self.planning_report_path.read_text(encoding="utf-8"))
            if phase == "preview":
                self._set_endpoint_states(start=report.get("start_state_status", "UNKNOWN"),
                                          goal=report.get("goal_state_status", "NOT GENERATED"))
                if "realhome_version" in report:
                    record = self._saved_real_home_record()
                    if record is None or report["realhome_version"] != record["realhome_version"]:
                        raise ValueError("STALE: RealHome在规划期间已更新，请重新规划")
                if "collision_scene_version" in report:
                    if report["collision_scene_version"] != self._current_calibration_binding()["collision_scene_version"]:
                        raise ValueError("STALE: 碰撞场景在规划期间已更新，请重新规划")
            if int(exit_code) != 0:
                raise ValueError(report.get("failure_reason", f"MATLAB exit {exit_code}"))
            if phase == "preview":
                self.planning_workflow.preview_complete(self.planning_output_path, report)
                published = self._publish_planning_trajectory("preview")
                self.planning_workflow.raw_path = published
                self.full_raw_trajectory_path = published
                adoption_started = time.perf_counter()
                self._adopt_trajectory_file(published, auto_validate=False)
                adoption_seconds = time.perf_counter() - adoption_started
                self.online_status_label.setText(self.planning_workflow.stage.value + " · 可播放 / 拖动；碰撞结果：" +
                    str(report.get("collision_status", "UNKNOWN · 旧预览缺少对象对审计")) +
                    "；已自动导入轨迹执行页（仅预演，下一步点击轨迹连续规划）")
            else:
                # Physical/servo PENDING is not an algorithm failure. Verify
                # the independent offline contract before accepting artifacts.
                checked = PlanningWorkflow(raw_path=self.planning_workflow.raw_path)
                checked.preparation_complete(self.planning_output_path, report)
                from trajectory.trajectory_loader import load_trajectory
                candidate = load_trajectory(str(self.planning_output_path))
                if (not candidate.metadata.get("preview_only")
                        or not candidate.metadata.get("diagnostic_only")
                        or candidate.metadata.get("workflow_stage") != "PHYSICAL_VALIDATION_PENDING"
                        or self._scalar_bool(candidate.metadata.get("executionReady"), True)):
                    raise ValueError("离线优化输出缺少非执行用途元数据，禁止载入")
                mismatch = self._trajectory_binding_reasons(candidate)
                if mismatch:
                    raise ValueError("Version mismatch: " + "; ".join(mismatch))
                fresh_reasons = self._real_planning_readiness_reasons()
                if fresh_reasons:
                    raise ValueError("Real-scene verification failed: " + "; ".join(fresh_reasons))
                published = self._publish_planning_trajectory("prepare")
                self._adopt_trajectory_file(published, auto_validate=False)
                self.planning_workflow.preparation_complete(published, report)
                self.online_status_label.setText(self.planning_workflow.reason + "；已自动导入轨迹执行：" + published.name)
                self.online_execution_preparation_label.setText("真机执行准备：待验证（物理安全 / 伺服验收；无执行许可）")
                self.offline_analysis_report_path = self.planning_report_path
                # Report rendering is presentation-only; a plotting failure
                # must not relabel a completed numerical validation as FAIL.
                try:
                    from gui.offline_planning_report import render_report
                    render_report(self.planning_report_path)
                except Exception as exc:
                    self._log(f"离线图表生成失败，原始报告及CSV仍可查看：{exc}")
                self.online_report_button.setEnabled(True)
                self.online_save_analysis_button.setEnabled(True)
            self._show_online_result(self.planning_report_path)
            self._set_online_stage(self.planning_workflow.stage.value)
            self._apply_planned_tcp_path()
            self._set_preview_pose(0)
            if phase == "prepare":
                self.tabs.setCurrentIndex(self.trajectory_tab_index)
                self.trajectory_scroll.verticalScrollBar().setValue(0)
                self._log("连续规划完成，已自动导入并打开轨迹执行页；未自动执行：" + self.loaded_path_name)
        except Exception as exc:
            self._workflow_failure(phase, str(exc))
            if phase == "prepare" and self.planning_report_path.is_file():
                self._show_online_result(self.planning_report_path)
        self._update_buttons()
        # Diagnostic timings only: no acceptance or execution state changes.
        performance = {
            "phase": phase,
            "file_load_and_scene_adoption_sec": adoption_seconds,
            "completion_callback_sec": time.perf_counter() - callback_started,
            "process_and_transfer_sec": (
                callback_started - self._workflow_process_started_at
                if hasattr(self, "_workflow_process_started_at") else None
            ),
            "robot_motion": "NO",
        }
        try:
            self.planning_report_path.with_name("hmi_transfer_performance.json").write_text(
                json.dumps(performance, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except OSError as exc:
            self._log(f"无法保存界面载入耗时：{exc}")

    def _open_offline_analysis(self):
        from PyQt5.QtCore import QUrl
        from PyQt5.QtGui import QDesktopServices
        path = getattr(self, "offline_analysis_report_path", None)
        if path is not None:
            html = path.with_name("offline_analysis.html")
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(html if html.is_file() else path)))

    def _save_offline_analysis(self):
        from PyQt5.QtWidgets import QFileDialog
        from gui.offline_planning_report import save_analysis_bundle
        path = getattr(self, "offline_analysis_report_path", None)
        if path is None:
            return
        destination, _ = QFileDialog.getSaveFileName(self, "保存离线分析（不可用于真机执行）",
            str(path.parent / "offline_analysis.zip"), "离线分析包 (*.zip)")
        if destination:
            try:
                save_analysis_bundle(path, Path(destination))
                self._log("已保存离线分析包（无执行许可）：" + destination)
            except Exception as exc:
                self._log("保存离线分析失败：" + str(exc))

    def _update_preview_values(self, index):
        if not hasattr(self, "online_preview_pose_label") or self.loaded_trajectory is None:
            return
        traj = self.loaded_trajectory
        q = traj.positions[index]
        tcp = "—"
        pose = traj.metadata.get("tcpPosePath")
        if pose is not None:
            try:
                T = np.asarray(pose[index], dtype=float)
                rpy = np.rad2deg(self._rpy_zyx_from_rotation(T[:3, :3]))
                tcp = "XYZ mm " + np.array2string(T[:3, 3]*1000, precision=3) + " | RPY° " + np.array2string(rpy, precision=3)
            except (ValueError, IndexError, TypeError):
                pass
        if tcp == "—" and traj.tcp_path is not None:
            tcp = "XYZ mm " + np.array2string(traj.tcp_path[index]*1000, precision=3)
        labels = traj.metadata.get("segmentLabel", [])
        segment = labels[index] if len(labels) > index else "—"
        self.online_preview_pose_label.setText(f"预览 {index+1}/{traj.point_count} · {segment}\nTCP ({traj.metadata.get('tcp_frame', 'base_link')}): {tcp}\nq°: " + self._format_joint_row(q))
        diagnostic = getattr(self.robot_view, "preview_frame_diagnostics", None)
        if callable(diagnostic):
            d = diagnostic()
            def point(key):
                value = d.get(key)
                return "—" if value is None else np.array2string(np.asarray(value)*1000, precision=3)
            text = self.online_preview_pose_label.text()
            if "flange_pose" in d:
                text += "\nDISPLAY base XYZ mm · TCP " + np.array2string(d["tcp_pose"][:3,3]*1000, precision=3)
                text += " · gun tip " + point("gun_tip_world") + " · wire tip " + point("wire_tip_world")
                text += "\nflange XYZ mm " + np.array2string(d["flange_pose"][:3,3]*1000, precision=3)
                text += " · RPY° " + np.array2string(np.rad2deg(self._rpy_zyx_from_rotation(d["flange_pose"][:3,:3])), precision=3)
                tooltip = "T_base_flange\n" + np.array2string(d["flange_pose"], precision=8)
                tooltip += "\nT_flange_to_gun\n" + np.array2string(d["T_flange_to_gun"], precision=8)
                self.online_preview_pose_label.setToolTip(tooltip)
            pairs = d.get("collision_pairs", [])
            text += "\nCollision: " + d["collision_status"] + (" · " + "; ".join(pairs) if pairs else "")
            if d.get("display_warning"):
                text += " · " + d["display_warning"]
            self.online_preview_pose_label.setText(text)
            width = max(400, self.online_preview_pose_label.width())
            self.online_preview_pose_label.setMinimumHeight(max(self._u(150), self.online_preview_pose_label.heightForWidth(width)))
            if d["collision_status"] != "PASS" and traj.metadata.get("preview_only"):
                self.planning_workflow.stage = PlanningStage.PREVIEW_WARNING
                self._set_online_stage(PlanningStage.PREVIEW_WARNING.value)
                self.online_status_label.setText("WARNING · " + ("; ".join(pairs) or d.get("display_warning") or "碰撞审计未完成"))
                labels = getattr(self, "online_result_labels", {})
                if "collision" in labels:
                    labels["collision"].setText("WARNING")
                    labels["collision"].setStyleSheet("color:#8a6d00;font-weight:bold;")
                observed = getattr(self, "_preview_pair_observations", {})
                if pairs and observed.get(index) != pairs:
                    observed[index] = list(pairs)
                    self._preview_pair_observations = observed
                    self._log(f"Preview frame {index+1}: " + "; ".join(pairs) + " (display surface diagnostic)")


def workflow_binding(binding: dict, scene_geometry_hash: str | None) -> dict:
    """Content identities for the derived wire and collision dependencies."""
    result = dict(binding)
    def digest(keys):
        return hashlib.sha256("|".join(str(result.get(k) or "MISSING") for k in keys).encode()).hexdigest()
    result["wire_collision_shorten_mm"] = 3
    result["scene_geometry_sha256"] = scene_geometry_hash
    result["environment_geometry_sha256"] = None
    result["environment_geometry_version"] = None
    result["fixed_environment_contacts_sha256"] = None
    if result.get("planning_scene") == "REAL_CALIBRATION":
        root = Path(__file__).resolve().parents[2]
        environment = root / "environment_geometry_real.json"
        support = root / "fixed_environment_contacts.json"
        if support.is_file():
            result["fixed_environment_contacts_sha256"] = hashlib.sha256(support.read_bytes()).hexdigest()
        if environment.is_file():
            content = environment.read_bytes()
            result["environment_geometry_sha256"] = hashlib.sha256(content).hexdigest()
            try:
                data = json.loads(content)
                result["environment_geometry_version"] = data.get("version") or "ENVIRONMENT-" + result["environment_geometry_sha256"]
            except (ValueError, TypeError):
                result["environment_geometry_version"] = "INVALID"
        # Fingerprint the actual REAL preview dependency even when a separate
        # legacy 'valid' flag is absent. This is identity, not application approval.
        mount = root / "real_gun_mount_calibration.json"
        result["gun_mount_sha256"] = hashlib.sha256(mount.read_bytes()).hexdigest() if mount.is_file() else None
        result.update(robot_collision_binding(root))
    else:
        result["robot_collision_geometry_sha256"] = None
        result["robot_collision_coverage"] = "MISSING"
    result["wire_model_version"] = "WIRE-" + digest(("real_gun_geometry_sha256", "gun_mount_sha256", "tcp_calibration_sha256", "wire_collision_shorten_mm"))
    scene_keys = ("urdf_sha256", "tcp_calibration_sha256", "real_gun_geometry_sha256", "gun_mount_sha256", "workpiece_sha256", "scene_geometry_sha256", "wire_model_version", "planning_scene", "robot_collision_geometry_sha256", "robot_collision_coverage", "environment_geometry_sha256")
    if result.get("planning_scene") == "REAL_CALIBRATION":
        scene_keys += ("fixed_environment_contacts_sha256",)
    result["collision_scene_version"] = "SCENE-" + digest(scene_keys)
    return result


def robot_collision_binding(root: Path) -> dict:
    """Expected URDF mesh identity; MATLAB independently verifies loaded bodies."""
    names = ("base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link")
    try:
        tree = ET.parse(root / "real_robot_description_calibrated.urdf").getroot()
        links = {link.attrib["name"]: link for link in tree.findall("link")}
        descriptor = []
        complete = True
        for name in names:
            hashes = []
            for mesh in links[name].findall("collision/geometry/mesh"):
                uri = mesh.attrib["filename"]
                if uri.startswith("package://"):
                    relative = uri[len("package://"):]
                    prefixes = [Path("/opt/ros/humble")] + [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
                    paths = [prefix / "share" / relative for prefix in prefixes]
                    path = next((p for p in paths if p.is_file()), None)
                else:
                    path = Path(uri) if Path(uri).is_absolute() else root / uri
                if path is None or not path.is_file():
                    complete = False
                    continue
                hashes.append(hashlib.sha256(path.read_bytes()).hexdigest())
            complete = complete and bool(hashes)
            descriptor.append(f"{name}:{len(hashes)}:" + ",".join(hashes))
        return {"robot_collision_geometry_sha256": hashlib.sha256("|".join(descriptor).encode()).hexdigest(),
                "robot_collision_coverage": "COMPLETE" if complete else "MISSING"}
    except (OSError, KeyError, ET.ParseError):
        return {"robot_collision_geometry_sha256": None, "robot_collision_coverage": "MISSING"}
