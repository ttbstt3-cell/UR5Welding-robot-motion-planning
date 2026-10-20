"""Local, evidence-bound execution review. No ROS imports or motion commands.

Physical/servo findings are an identified on-site assessor's declarations,
supported by immutable report attachments. This code checks consistency, not
the truth of a physical inspection, and is NOT a safety-rated interlock.
"""
from __future__ import annotations

from datetime import datetime, timezone
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from safety.joint_limits import JOINT_NAMES

SCHEMA = "UR5_FIELD_ACCEPTANCE_V1"
PERMIT_SCHEMA = "UR5_LOCAL_EXECUTION_AUTHORIZATION_V1"
MODE = "positions_plus_velocities_cubic_hermite"


def _json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":"))


@lru_cache(maxsize=128)
def _hash_at_stat(path, identity):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    after = Path(path).stat()
    if identity != (after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns):
        raise ValueError("文件在读取期间发生变化：" + path)
    return digest.hexdigest()


def file_hash(path):
    p = Path(path).resolve(strict=True)
    s = p.stat()
    return _hash_at_stat(str(p), (s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns))


def numeric_hash(traj):
    h = hashlib.sha256(_json(traj.joint_names).encode())
    for field in ("positions", "time_from_start", "velocities", "accelerations"):
        value = getattr(traj, field)
        h.update(field.encode())
        if value is None:
            h.update(b"NONE")
        else:
            a = np.asarray(value, dtype="<f8", order="C")
            h.update(_json(a.shape).encode()); h.update(a.tobytes())
    return h.hexdigest()


def timestamp(value):
    dt = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if dt.tzinfo is None:
        raise ValueError("验收时间必须包含时区")
    return dt.timestamp()


def offline_context(traj, report_file, current_binding, realhome_version):
    """Recheck genuine offline output, immutable reference and current versions."""
    from gui.planning_workflow import PlanningWorkflow
    if traj is None:
        raise ValueError("请先加载连续规划输出")
    if traj.metadata.get("workflow_stage") != "PHYSICAL_VALIDATION_PENDING":
        raise ValueError("需要加载已完成连续规划的离线优化轨迹，不是原始预览")
    report_path = Path(report_file).resolve(strict=True)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    source = Path(traj.source_path).resolve(strict=True)
    # A copied MAT is allowed only if its bytes still match the recorded output.
    recorded = Path(report["artifacts"]["trajectory_mat"]).resolve(strict=True)
    if file_hash(source) != file_hash(recorded):
        raise ValueError("加载文件与离线报告记录的轨迹不一致")
    state = PlanningWorkflow()
    state.preparation_complete(recorded, report)
    binding = traj.metadata.get("calibration_binding", {})
    if not binding or _json(binding) != _json(current_binding):
        raise ValueError("VERSION_MISMATCH：轨迹与当前TCP/工件/焊枪/环境版本不一致")
    if _json(report["version_validation"]["current_binding"]) != _json(binding):
        raise ValueError("离线报告与轨迹的版本绑定不一致")
    if not realhome_version or realhome_version != traj.metadata.get("realhome_version"):
        raise ValueError("VERSION_MISMATCH：RealHome已变化或缺失")
    if traj.joint_names != JOINT_NAMES or traj.velocities is None or traj.accelerations is not None:
        raise ValueError("控制器验收要求标准关节顺序及positions+velocities，不接受不同插值输入")
    reference_path = Path(report["artifacts"]["trajectory_json"]).resolve(strict=True)
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    for key, actual in (("qPath", traj.positions), ("timeFromStart", traj.time_from_start),
                        ("velocities", traj.velocities)):
        expected = np.asarray(reference[key], dtype=float).reshape(np.shape(actual))
        # JSON decimal round-trip allowance only; NOT a planning tolerance.
        if not np.isfinite(expected).all() or not np.allclose(actual, expected, rtol=0, atol=2e-14):
            raise ValueError("轨迹内存数据与离线参考不一致：" + key)
    return dict(trajectory_sha256=file_hash(source), numeric_sha256=numeric_hash(traj),
                offline_report_sha256=file_hash(report_path),
                reference_sha256=file_hash(reference_path), calibration_binding=binding,
                reference_path=str(reference_path), recorded_trajectory_path=str(recorded),
                realhome_version=realhome_version,
                interpolation_mode=MODE, report_path=str(report_path),
                planned_max_velocity=report["controller_validation"]["max_joint_velocity_rad_per_sec"],
                planned_max_acceleration=report["controller_validation"]["max_joint_acceleration_rad_per_sec2"])


def draft_record(context):
    return dict(schema=SCHEMA, status="DRAFT", context=json.loads(_json(context)), inspector="",
                qualification="", performed_at_utc="", procedure_reference="",
                physical=dict(status="PENDING", report_file="", report_sha256="",
                              minimum_verified_clearance_mm=None, uncertainty_mm=None,
                              required_margin_mm=None, installation_verified=False,
                              workpiece_environment_verified=False, wire_approximation_reviewed=False),
                servo=dict(status="PENDING", report_file="", report_sha256="",
                           controller="scaled_joint_trajectory_controller", interpolation_mode=MODE,
                           joint_coordinate_convention="ROS_JOINT_STATES_RAD",
                           max_tcp_tracking_error_mm=None, max_tcp_orientation_error_deg=None,
                           approved_velocity_rad_s=None, approved_acceleration_rad_s2=None,
                           actual_controller_and_scaling_verified=False))


def _number(record, key):
    v = record.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
        raise ValueError("缺少有效实测/验收数值：" + key)
    return float(v)


def validate_record(record, context, base_dir, now=None):
    now = datetime.now(timezone.utc).timestamp() if now is None else now
    if record.get("schema") != SCHEMA or record.get("status") != "REVIEWED":
        raise ValueError("现场记录仍为草稿或格式不正确；不能用勾选代替验收")
    if _json(record.get("context")) != _json(context):
        raise ValueError("STALE：现场记录不属于当前轨迹、RealHome或版本")
    for key in ("inspector", "qualification", "procedure_reference"):
        if not str(record.get(key, "")).strip():
            raise ValueError("缺少现场验收责任人/资质/规程：" + key)
    age = now - timestamp(record.get("performed_at_utc"))
    if not 0 <= age <= 24 * 3600:
        raise ValueError("现场验收记录已超过24小时或时间在未来")
    for name in ("physical", "servo"):
        part = record.get(name, {})
        if part.get("status") != "PASS":
            raise ValueError(name + " 验收未通过")
        p = Path(part.get("report_file", ""))
        if not p.is_absolute():
            p = Path(base_dir) / p
        if not p.is_file() or p.stat().st_size == 0 or file_hash(p) != part.get("report_sha256"):
            raise ValueError(name + " 验收附件缺失、为空或哈希不符")
    p, s = record["physical"], record["servo"]
    for key in ("installation_verified", "workpiece_environment_verified", "wire_approximation_reviewed"):
        if p.get(key) is not True:
            raise ValueError("现场几何/焊丝碰撞近似尚未确认：" + key)
    clearance = _number(p, "minimum_verified_clearance_mm")
    uncertainty = _number(p, "uncertainty_mm"); margin = _number(p, "required_margin_mm")
    if margin <= 0 or clearance - uncertainty < margin:
        raise ValueError("现场最小净间隙不足以覆盖不确定性及独立安全余量")
    if (s.get("controller") != "scaled_joint_trajectory_controller"
            or s.get("interpolation_mode") != MODE
            or s.get("joint_coordinate_convention") != "ROS_JOINT_STATES_RAD"
            or s.get("actual_controller_and_scaling_verified") is not True):
        raise ValueError("实际控制器、插值规则、时间倍率或关节坐标尚未验收")
    if (_number(s, "max_tcp_tracking_error_mm") >= .5
            or _number(s, "max_tcp_orientation_error_deg") > .5):
        raise ValueError("实际伺服跟踪误差未满足0.5mm / 0.5°要求")
    if not context["planned_max_velocity"] <= _number(s, "approved_velocity_rad_s") <= math.radians(5):
        raise ValueError("轨迹速度不在现场已验收的运动包络内")
    if not context["planned_max_acceleration"] <= _number(s, "approved_acceleration_rad_s2") <= .261799:
        raise ValueError("轨迹加速度不在现场已验收的运动包络内")
    return True


def issue_authorization(record, context, record_path, operator, now=None):
    now = datetime.now(timezone.utc).timestamp() if now is None else now
    actual = json.loads(Path(record_path).read_text(encoding="utf-8"))
    if _json(actual) != _json(record):
        raise ValueError("现场记录在审核期间发生变化，请重新导入")
    validate_record(record, context, Path(record_path).parent, now)
    if not operator.strip():
        raise ValueError("必须填写授权操作人")
    return dict(schema=PERMIT_SCHEMA, context=json.loads(_json(context)), operator=operator.strip(),
                issued_at=now, expires_at=min(now + 3600, timestamp(record["performed_at_utc"]) + 86400),
                acceptance_file=str(Path(record_path).resolve()), acceptance_sha256=file_hash(record_path),
                scope="CURRENT_SESSION_EXACT_TRAJECTORY_ONLY", motion_sent=False)


def validate_authorization(permit, context, now=None):
    now = datetime.now(timezone.utc).timestamp() if now is None else now
    if (not isinstance(permit, dict) or permit.get("schema") != PERMIT_SCHEMA
            or _json(permit.get("context")) != _json(context)):
        raise ValueError("未签署当前轨迹执行授权，或授权已STALE")
    issued, expiry = permit.get("issued_at"), permit.get("expires_at")
    if (not isinstance(issued, (int, float)) or not isinstance(expiry, (int, float))
            or not math.isfinite(issued) or not math.isfinite(expiry)
            or not issued <= now < expiry <= issued + 3600 or not permit.get("operator")):
        raise ValueError("执行授权已过期或时间/操作人无效")
    path = Path(permit["acceptance_file"])
    if file_hash(path) != permit["acceptance_sha256"]:
        raise ValueError("现场验收记录已被更改，授权失效")
    validate_record(json.loads(path.read_text(encoding="utf-8")), context, path.parent, now)
    return True
