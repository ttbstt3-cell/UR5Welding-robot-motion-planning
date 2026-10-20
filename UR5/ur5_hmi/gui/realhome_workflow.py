"""Persist versioned RealHome samples and validate them offline with MATLAB.

This module reads joint feedback and invokes an offline endpoint check. It has
no connection to a ROS publisher or motion/action client.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import yaml
from PyQt5.QtCore import QProcess


def _joint_sample(q) -> list[float]:
    values = np.asarray(q, dtype=float).reshape(-1)
    if values.size != 6 or not np.isfinite(values).all():
        raise ValueError("RealHome 必须包含六个有限关节角（rad）")
    return [float(value) for value in values]


def _write_config(path: Path, payload: dict) -> None:
    # Atomic replacement prevents planning from reading half a YAML record.
    temporary = path.with_name(path.name + ".realhome.tmp")
    temporary.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    temporary.replace(path)


def read_realhome_record(config_path: Path) -> dict | None:
    """Read disk on every request; legacy samples remain unvalidated."""
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        q = _joint_sample(config.get("real_home", []))
        record = copy.deepcopy(config.get("real_home_metadata", {}))
        if not isinstance(record, dict) or record.get("q_rad") != q or not record.get("realhome_version"):
            identity = hashlib.sha256(json.dumps(q, separators=(",", ":")).encode()).hexdigest()
            record = {"q_rad": q, "frame_id": "URDF_joint_configuration", "units": "rad",
                      "realhome_version": "LEGACY-REALHOME-" + identity,
                      "timestamp": None, "collision_scene_version": None,
                      "calibration_binding": {}, "validation": {"status": "UNKNOWN",
                      "reason": "历史 RealHome 缺少场景版本；规划前重新验证"}}
        return record
    except (OSError, ValueError, TypeError, yaml.YAMLError):
        return None


def save_realhome_record(config_path: Path, q, binding: dict, *, timestamp: str | None = None) -> dict:
    """Save a user-acquired sample without changing/wrapping any joint angle."""
    values = _joint_sample(q)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    stamp = timestamp or datetime.now(timezone.utc).isoformat()
    digest = hashlib.sha256(json.dumps({"q": values, "binding": binding, "timestamp": stamp,
                                      "nonce": time.time_ns()}, sort_keys=True).encode()).hexdigest()
    record = {"schema_version": 1, "q_rad": values, "units": "rad",
              "frame_id": "URDF_joint_configuration", "timestamp": stamp,
              "source": "ROS2_JOINT_FEEDBACK", "realhome_version": "REALHOME-" + digest,
              "robot_model_version": binding.get("urdf_version"),
              "calibration_binding": copy.deepcopy(binding),
              "validation": {"status": "PENDING", "collision_pairs": []}}
    for key in ("urdf_version", "tcp_calibration_version", "gun_geometry_version",
                "wire_model_version", "collision_scene_version"):
        record[key] = binding.get(key)
    record["tool_version"] = binding.get("gun_geometry_version")
    config["real_home"] = values
    config["real_home_metadata"] = record
    _write_config(config_path, config)
    return record


def store_realhome_validation(config_path: Path, version: str, scene_version: str, validation: dict) -> bool:
    """Ignore completion from a previous saved sample or scene snapshot."""
    record = read_realhome_record(config_path)
    if (record is None or record.get("realhome_version") != version
            or record.get("collision_scene_version") != scene_version):
        return False
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    record["validation"] = copy.deepcopy(validation)
    config["real_home_metadata"] = record
    _write_config(config_path, config)
    return True


class RealHomeWorkflowPanel:
    def _realhome_config_path(self) -> Path:
        from gui.main_window import CONFIG
        return Path(getattr(self, "realhome_config_path", CONFIG))

    def _saved_real_home_record(self) -> dict | None:
        return read_realhome_record(self._realhome_config_path())

    def _read_saved_real_home(self):
        record = self._saved_real_home_record()
        return np.asarray(record["q_rad"], dtype=float) if record else None

    def _refresh_realhome_panel(self) -> None:
        record = self._saved_real_home_record()
        if record is None:
            self.home_info.setText("尚未保存有效 RealHome")
            self.home_validation_label.setText("RealHome Validation: UNKNOWN")
            return
        version = record["realhome_version"]
        self.home_info.setText("RealHome：\n" + "  ".join(
            f"J{i + 1}={np.rad2deg(q):.3f}°" for i, q in enumerate(record["q_rad"]))
            + "\n保存时间：" + str(record.get("timestamp") or "历史记录")
            + "\n版本：" + (version[:24] + "…" if len(version) > 24 else version))
        self.home_info.setToolTip(json.dumps(record, ensure_ascii=False, indent=2))
        validation = record.get("validation", {})
        status = validation.get("status", "UNKNOWN")
        if record.get("collision_scene_version") != self._current_calibration_binding().get("collision_scene_version"):
            status = "STALE"
        pairs = validation.get("collision_pairs", [])
        descriptions = []
        for pair in pairs:
            if isinstance(pair, dict):
                descriptions.append(str(pair.get("body_a", pair.get("body1", "?"))) + " ↔ "
                                    + str(pair.get("body_b", pair.get("body2", "?"))))
            else:
                descriptions.append(str(pair))
        reason = validation.get("failure_reason", validation.get("reason", ""))
        self.home_validation_label.setText("RealHome Validation: " + status
            + (("\n" + "; ".join(descriptions)) if descriptions else ("\n" + reason if reason else "")))
        color = ("#146c2e" if status in {"PASS", "VALIDATION_PASS"}
                 else "#9b1c1c" if status in {"FAIL", "COLLISION_FAIL", "MISSING_DATA"}
                 else "#8a6d00")
        self.home_validation_label.setStyleSheet(f"font-weight:bold;color:{color};")

    def _save_realhome_current_pose(self) -> None:
        if self._motion_busy() or self.planning_busy:
            self._log("RealHome 保存等待当前运动/离线规划结束")
            return
        if not self._feedback_valid(self.state):
            self._log("RealHome 未保存：需要新鲜有效的真实关节反馈")
            return
        binding = self._current_calibration_binding()
        if binding.get("planning_scene") != "REAL_CALIBRATION":
            message = "RealHome为真实机器人姿态，请选择REAL CALIBRATION后保存"
            self.home_validation_label.setText("RealHome 未保存：\n" + message)
            self.home_validation_label.setStyleSheet("font-weight:bold;color:#8a6d00;")
            self._log(message)
            return
        try:
            record = save_realhome_record(self._realhome_config_path(), list(self.state.positions),
                                          binding)
            self.config = yaml.safe_load(self._realhome_config_path().read_text(encoding="utf-8")) or {}
        except Exception as exc:
            self._log(f"RealHome 保存失败：{exc}")
            self.home_validation_label.setText(f"RealHome Validation: FAIL\n{exc}")
            return
        # Removing preparation references cannot delete user trajectories; all
        # previous files are retained while this sample becomes the next start.
        from gui.planning_workflow import PlanningWorkflow
        self.planning_workflow = PlanningWorkflow()
        self.full_raw_trajectory_path = None
        self.trajectory_validation = None
        self._mark_trajectory_stale("RealHome 版本已改变，需要重新规划和端点验证")
        index = self.online_start_combo.findData("real_home")
        if index >= 0:
            self.online_start_combo.setCurrentIndex(index)
        self._update_online_start_display()
        self._set_online_stage("IDLE")
        self.online_status_label.setText("RealHome 已更新，等待端点验证和重新规划")
        self._refresh_realhome_panel()
        self._log(f"RealHome 已保存：{record['realhome_version']}；正在检查端点")
        self._start_realhome_validation(record)
        self._update_buttons()

    def _start_realhome_validation(self, record: dict) -> None:
        from gui.main_window import ROOT
        previous = getattr(self, "realhome_validation_process", None)
        if previous is not None:
            previous.terminate()
        run_id = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000:06d}"
        run_root = Path(getattr(self, "realhome_validation_run_root", ROOT.parent / "规划文件" / "workflow_runs"))
        run = run_root / ("realhome_validation_" + run_id)
        run.mkdir(parents=True, exist_ok=False)
        payload = {"qHome": record["q_rad"], "qStart": record["q_rad"],
                   "realhome_version": record["realhome_version"],
                   "collision_scene_version": record.get("collision_scene_version"),
                   "calibration_binding": record["calibration_binding"],
                   "realhome_config_path": str(self._realhome_config_path().resolve()),
                   "planning_scene": record["calibration_binding"].get("planning_scene"),
                   "outputDir": str(run), "preview_only": True, "executionReady": False}
        request = run / "request.json"
        request.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        expression = (f"addpath({self._matlab_quote(ROOT.parent / 'matlab仿真')}); "
                      f"req=jsondecode(fileread({self._matlab_quote(request)})); "
                      "validate_hmi_realhome_request(req);")
        process = QProcess(self)
        self.realhome_validation_process = process
        self.realhome_validation_busy = True
        self.realhome_validation_run = run
        process.readyReadStandardOutput.connect(lambda: self._realhome_validation_output(process, run, False))
        process.readyReadStandardError.connect(lambda: self._realhome_validation_output(process, run, True))
        process.finished.connect(lambda code, _status: self._realhome_validation_finished(process, record, run, code))
        process.errorOccurred.connect(lambda error: self._realhome_validation_error(process, record, run, error))
        process.start("matlab", ["-batch", expression])

    def _realhome_validation_output(self, process, run: Path, stderr: bool) -> None:
        data = process.readAllStandardError() if stderr else process.readAllStandardOutput()
        output = bytes(data).decode("utf-8", errors="replace")
        if output:
            with (run / "matlab.log").open("a", encoding="utf-8") as log:
                log.write(output)

    def _realhome_validation_error(self, process, record, run, error) -> None:
        if error == QProcess.FailedToStart:
            validation = {"status": "FAIL", "collision_pairs": [],
                          "failure_reason": "无法启动 MATLAB 端点验证", "report_path": str(run / "report.json")}
            self._accept_realhome_validation(record, validation)
            if getattr(self, "realhome_validation_process", None) is process:
                self.realhome_validation_process = None
                self.realhome_validation_busy = False
            self._update_buttons()
            process.deleteLater()

    def _accept_realhome_validation(self, record: dict, validation: dict) -> bool:
        current = self._current_calibration_binding()
        if record.get("collision_scene_version") != current.get("collision_scene_version"):
            self._log("RealHome 端点报告 STALE：验证期间碰撞场景版本改变")
            self._refresh_realhome_panel()
            return False
        accepted = store_realhome_validation(self._realhome_config_path(), record["realhome_version"],
                                             record.get("collision_scene_version"), validation)
        if accepted:
            self._refresh_realhome_panel()
            self._log("RealHome Validation: " + validation["status"] + "；报告：" + validation.get("report_path", ""))
        else:
            self._log("忽略旧 RealHome 端点报告：已保存更新姿态")
        return accepted

    def _realhome_validation_finished(self, process, record: dict, run: Path, code: int) -> None:
        self._realhome_validation_output(process, run, False)
        self._realhome_validation_output(process, run, True)
        try:
            report = json.loads((run / "report.json").read_text(encoding="utf-8"))
            if (report.get("realhome_version") != record["realhome_version"]
                    or report.get("collision_scene_version") != record.get("collision_scene_version")):
                raise ValueError(report.get("failure_reason") or "端点验证报告版本与保存样本不一致")
            endpoint = report.get("endpoint_validation", {})
            report_status = report.get("status")
            passed = (code == 0 and report.get("success") is True
                      and report.get("binding_valid") is True
                      and report_status in {"PASS", "VALIDATION_PASS"}
                      and endpoint.get("valid") is True)
            if passed:
                # New MATLAB reports use the explicit four-way status. Keep
                # accepting legacy PASS reports from already-running HMI
                # processes without downgrading a new report.
                validation_status = "VALIDATION_PASS" if report_status == "VALIDATION_PASS" else "PASS"
            elif report_status in {"MISSING_DATA", "VERSION_MISMATCH", "COLLISION_FAIL"}:
                validation_status = report_status
            else:
                validation_status = "COLLISION_FAIL" if endpoint else "MISSING_DATA"
            validation = {"status": validation_status,
                          "collision_pairs": endpoint.get("collision_pairs", report.get("collision_pairs", [])),
                          "failure_reason": endpoint.get("failure_reason", report.get("failure_reason", "")),
                          "endpoint_validation": endpoint,
                          "binding_valid": report.get("binding_valid", False),
                          "timestamp": datetime.now(timezone.utc).isoformat(),
                          "report_path": str(run / "report.json")}
        except Exception as exc:
            validation = {"status": "FAIL", "collision_pairs": [], "failure_reason": str(exc),
                          "report_path": str(run / "report.json")}
        self._accept_realhome_validation(record, validation)
        if getattr(self, "realhome_validation_process", None) is process:
            self.realhome_validation_process = None
            self.realhome_validation_busy = False
        self._update_buttons()
        process.deleteLater()
