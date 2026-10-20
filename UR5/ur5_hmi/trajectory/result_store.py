"""Readable HMI result copies; never retime, change metadata or authorize motion.

The original run and its scene/reference dependencies remain the evidence source.
Only byte-identical MAT/report copies and a latest-result index are published.
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import tempfile

from safety.execution_preparation import file_hash
from trajectory.trajectory_loader import load_trajectory

INDEX_NAME = "最新轨迹索引.json"


def _atomic_json(path, payload):
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=".trajectory-index-", delete=False) as handle:
        temporary = Path(handle.name)
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    temporary.replace(path)


def read_index(directory):
    path = Path(directory) / INDEX_NAME
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def publish_result(source, report_file, directory, phase, *, parent_preview_sha256=None):
    if phase not in ("preview", "prepare"):
        raise ValueError("未知轨迹阶段")
    source = Path(source).resolve(strict=True)
    report_file = Path(report_file).resolve(strict=True)
    directory = Path(directory).resolve()
    trajectory = load_trajectory(str(source))
    report = json.loads(report_file.read_text(encoding="utf-8"))
    stage = trajectory.metadata.get("workflow_stage")
    if phase == "preview":
        if stage != "PREVIEW" or not (report.get("preview_available") or report.get("previewReady")):
            raise ValueError("没有可发布的完整预览轨迹")
    else:
        from gui.planning_workflow import PlanningWorkflow
        PlanningWorkflow().preparation_complete(source, report)
        if stage != "PHYSICAL_VALIDATION_PENDING":
            raise ValueError("没有通过离线连续规划的轨迹")
    match = re.search(r"(\d{8})_(\d{6})_(\d{6})$", source.parent.name)
    if match:
        day, clock, suffix = match.groups()
        stamp = f"{day[:4]}-{day[4:6]}-{day[6:]}_{clock[:2]}-{clock[2:4]}-{clock[4:]}_{suffix}"
    else:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
    title = "预览轨迹_仅预演" if phase == "preview" else "连续规划完成轨迹"
    scaling = trajectory.metadata.get("offline_speed_scaling")
    if phase == "prepare" and isinstance(scaling, dict):
        percent = float(scaling["percent_of_input"])
        if not 1 <= percent <= 100:
            raise ValueError("离线调速百分比无效")
        title = ("保守调试轨迹" if percent <= 50 else "离线调速轨迹") + f"_{percent:g}pct"
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f"{title}_{stamp}.mat"
    destination_report = destination.with_name(destination.stem + "_report.json")
    digest = file_hash(source)
    # Timestamped results are immutable: don't replace an older result with
    # different bytes, even if somebody reuses a request identifier.
    for original, copy in ((source, destination), (report_file, destination_report)):
        if copy.exists():
            if file_hash(copy) != file_hash(original):
                raise ValueError("同名轨迹已存在且内容不同：" + str(copy))
        else:
            shutil.copy2(original, copy)
            if file_hash(copy) != file_hash(original):
                raise ValueError("轨迹复制校验失败")
    entry = dict(path=str(destination), sha256=digest, report=str(destination_report),
        report_sha256=file_hash(destination_report), source_path=str(source), source_report=str(report_file),
        source_run=str(source.parent), phase=phase,
        realhome_version=str(trajectory.metadata.get("realhome_version", "")),
        collision_scene_version=str(trajectory.metadata.get("collision_scene_version", "")),
        published_at=datetime.now().astimezone().isoformat(), executionReady=False,
        parent_preview_sha256=parent_preview_sha256)
    index = read_index(directory)
    index.update(schema_version=1, selected=phase)
    index[phase] = entry
    if phase == "preview":
        # A new preview is not the old preview's optimized trajectory.
        index["prepare"] = None
    elif index.get("preview") and parent_preview_sha256 != index["preview"]["sha256"]:
        raise ValueError("连续规划不属于当前预览，不能替换最新轨迹")
    _atomic_json(directory / INDEX_NAME, index)
    return destination


def latest_result(directory):
    index = read_index(directory)
    entry = index.get(index.get("selected", ""))
    if not entry:
        return None
    path = Path(entry["path"])
    if not path.is_file() or file_hash(path) != entry["sha256"]:
        raise ValueError("最新轨迹文件已丢失或发生变化，请重新规划或手动选择")
    report = Path(entry["report"])
    if not report.is_file() or file_hash(report) != entry["report_sha256"]:
        raise ValueError("最新轨迹对应报告已丢失或发生变化，请重新检查")
    return path
