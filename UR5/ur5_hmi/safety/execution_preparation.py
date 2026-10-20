"""Validate recorded offline output for manually confirmed commissioning.

Software-only evidence; not physical or servo certification. No ROS commands.
"""
from __future__ import annotations
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import numpy as np
from safety.joint_limits import JOINT_NAMES


def _json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


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
    p = Path(path).resolve(strict=True); s = p.stat()
    return _hash_at_stat(str(p), (s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns))


def numeric_hash(traj):
    h = hashlib.sha256(_json(traj.joint_names).encode())
    for field in ("positions", "time_from_start", "velocities", "accelerations"):
        value = getattr(traj, field); h.update(field.encode())
        if value is None:
            h.update(b"NONE")
        else:
            a = np.asarray(value, dtype="<f8", order="C")
            h.update(_json(a.shape).encode()); h.update(a.tobytes())
    return h.hexdigest()


def offline_context(traj, report_file, current_binding, realhome_version):
    """Require complete offline evidence and current immutable inputs."""
    from gui.planning_workflow import PlanningWorkflow
    if traj is None:
        raise ValueError("请先加载连续规划输出")
    if traj.metadata.get("workflow_stage") != "PHYSICAL_VALIDATION_PENDING":
        raise ValueError("需要加载已完成连续规划的离线优化轨迹，不是原始预览")
    report_path = Path(report_file).resolve(strict=True)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    source = Path(traj.source_path).resolve(strict=True)
    recorded = Path(report["artifacts"]["trajectory_mat"]).resolve(strict=True)
    if file_hash(source) != file_hash(recorded):
        raise ValueError("加载文件与离线报告记录的轨迹不一致")
    state = PlanningWorkflow(); state.preparation_complete(recorded, report)
    binding = traj.metadata.get("calibration_binding", {})
    if binding.get("planning_scene") != "REAL_CALIBRATION":
        raise ValueError("真机调试只接受REAL场景，不接受冻结仿真轨迹")
    if not binding or _json(binding) != _json(current_binding):
        raise ValueError("VERSION_MISMATCH：轨迹与当前TCP/工件/焊枪/环境版本不一致")
    if _json(report["version_validation"]["current_binding"]) != _json(binding):
        raise ValueError("离线报告与轨迹的版本绑定不一致")
    if not realhome_version or realhome_version != traj.metadata.get("realhome_version"):
        raise ValueError("VERSION_MISMATCH：RealHome已变化或缺失")
    if traj.joint_names != JOINT_NAMES or traj.velocities is None or traj.accelerations is not None:
        raise ValueError("控制器验收要求标准关节顺序及positions+velocities，不接受不同插值输入")
    for index, key in ((0, "requested_start_q"), (-1, "real_home_q")):
        q = np.asarray(traj.metadata.get(key, []), dtype=float).reshape(-1)
        if q.shape != (6,) or not np.allclose(traj.positions[index], q, rtol=0, atol=1e-12):
            raise ValueError("REAL关节端点与规划请求的弧度关节值不一致：" + key)
    reference_path = Path(report["artifacts"]["trajectory_json"]).resolve(strict=True)
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    for key, actual in (("qPath", traj.positions), ("timeFromStart", traj.time_from_start),
                        ("velocities", traj.velocities)):
        expected = np.asarray(reference[key], dtype=float).reshape(np.shape(actual))
        # Decimal round-trip allowance only, never a planning threshold.
        if not np.isfinite(expected).all() or not np.allclose(actual, expected, rtol=0, atol=2e-14):
            raise ValueError("轨迹内存数据与离线参考不一致：" + key)
    return dict(trajectory_sha256=file_hash(source), numeric_sha256=numeric_hash(traj),
        offline_report_sha256=file_hash(report_path), reference_sha256=file_hash(reference_path),
        calibration_binding=binding, realhome_version=realhome_version, coordinate_space="ros_real",
        reference_path=str(reference_path), recorded_trajectory_path=str(recorded))
