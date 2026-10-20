"""Presentation / user-requested export only. Never creates execution permits."""
from __future__ import annotations

import html
import json
from pathlib import Path
import zipfile

import numpy as np


def render_report(report_path: Path) -> Path:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    audit = report["controller_validation"]
    folder = report_path.parent
    curves = folder / "offline_tracking_curves.csv"
    picture = folder / "offline_tracking_errors.png"
    if curves.is_file():
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        values = np.genfromtxt(curves, delimiter=",", names=True)
        figure = Figure(figsize=(11, 7), constrained_layout=True)
        FigureCanvasAgg(figure)
        axes = figure.subplots(3, 1, sharex=True)
        for axis, name, label in zip(axes, ("tcp_contour_mm", "tcp_sync_mm", "orientation_deg"),
                                     ("TCP contour (mm)", "TCP time-sync (mm)", "Orientation (deg)")):
            axis.plot(values["time_sec"], values[name], linewidth=.8)
            axis.axhline(.5, color="red", linestyle="--", linewidth=.8)
            axis.set_ylabel(label)
            axis.grid(alpha=.25)
        axes[-1].set_xlabel("Time (s); FK display samples, not an additional safety certificate")
        figure.savefig(picture, dpi=130)
    candidate = report.get("hybrid_velocity_candidate", {})
    metrics = [("Weld velocity source (seed before full-task optimization)",
                candidate.get("source", report.get("hybrid_velocity_source", "Not recorded"))),
               ("Non-optimal C fallback (NOT a D optimizer success)",
                candidate.get("nonoptimal_fallback", "Not recorded")),
               ("TCP contour max / mm", audit["max_tcp_contour_error_mm"]),
               ("TCP time-sync max / mm", audit["max_tcp_time_sync_error_mm"]),
               ("TCP orientation max / deg", audit["max_tcp_orientation_error_deg"]),
               ("Joint velocity max / rad/s", audit["max_joint_velocity_rad_per_sec"]),
               ("Joint acceleration max / rad/s²", audit["max_joint_acceleration_rad_per_sec2"]),
               ("Full collision samples", audit["sample_count"]),
               ("Collision PASS", audit["collision_pass"]),
               ("Joint limits PASS", audit["joint_limits_pass"]),
               ("Version checks PASS", report["version_validation"]["pass"]),
               ("Duration / s", report["duration"]),
               ("Segment-interior jerk max / rad/s³", report["jerk_statistics"]["max"]),
               ("Segment-interior jerk RMS / rad/s³", report["jerk_statistics"]["rms"]),
               ("Minimum sampled positive clearance / mm (NOT physical safety margin)",
                audit.get("min_positive_clearance_mm"))]
    rows = "".join(f"<tr><td>{html.escape(name)}</td><td>{html.escape(str(value))}</td></tr>"
                   for name, value in metrics)
    continuity = html.escape(json.dumps(report["continuity_scope"], ensure_ascii=False, indent=2))
    pending = "".join(f"<li>{html.escape(reason)}</li>" for reason in report["pending_reasons"])
    image = '<img src="offline_tracking_errors.png" style="max-width:100%">' if picture.is_file() else ""
    text = f'''<!doctype html><html lang="zh"><meta charset="utf-8">
<title>UR5 连续规划离线分析 · 不可用于真机执行</title>
<style>body{{font:16px sans-serif;max-width:1100px;margin:32px auto;line-height:1.6}}
td{{border-bottom:1px solid #ddd;padding:6px 18px}}pre{{white-space:pre-wrap}}.pending{{color:#8a6d00}}</style>
<h1>连续规划完成（离线验证通过）</h1>
<h2 class="pending">真机执行准备：待验证</h2>
<p>executionReady=false。以下轨迹、曲线及报告仅供离线预演和分析，不含执行许可。</p>
<table>{rows}</table><h2>连续性范围</h2><pre>{continuity}</pre>
<p>分段内jerk与节点加速度跳变分别统计；不能将段接口C²通过解释为全程C²连续。</p>
<h2>仍需验证</h2><ul>{pending}</ul><h2>误差曲线</h2>{image}
<p>图表为独立FK显示采样；验收指标来自完整控制器等价验证。实际UR伺服响应尚未验收。</p>
<p><a href="offline_optimized_trajectory.mat">非执行用途优化轨迹 MAT</a> ·
<a href="offline_optimized_trajectory.json">非执行用途优化轨迹 JSON</a> ·
<a href="offline_tracking_curves.csv">曲线 CSV</a> · <a href="report.json">完整报告</a></p></html>'''
    path = folder / "offline_analysis.html"
    path.write_text(text, encoding="utf-8")
    return path


def save_analysis_bundle(report_path: Path, destination: Path) -> None:
    """Whitelist diagnostic artifacts, never gather arbitrary execution files."""
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (report.get("executionReady") is not False
            or report.get("continuous_planning_complete") is not True):
        raise ValueError("没有已完成的非执行用途离线分析")
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for name in ("report.json", "offline_analysis.html", "offline_tracking_errors.png",
                     "offline_tracking_curves.csv", "offline_optimized_trajectory.mat",
                     "offline_optimized_trajectory.json", "matlab.log"):
            source = report_path.parent / name
            if source.is_file():
                bundle.write(source, arcname=name)
        bundle.writestr("OFFLINE_ONLY.txt", "Diagnostic preview/analysis only. executionReady=false. No robot execution permit.\n")
