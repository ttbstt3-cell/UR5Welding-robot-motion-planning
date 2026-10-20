"""Inventory exact old HMI trajectory payloads; optionally move them to trash.

Never removes calibration, frozen scene/anchors, experiments or complete runs.
Apply requires the exact inventory digest and rechecks every individual file.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety.execution_preparation import file_hash
from trajectory.result_store import read_index


def inventory(root):
    planning = root / "规划文件"
    current = planning / "当前轨迹"
    index = read_index(current)
    entries = [index[k] for k in ("preview", "prepare") if index.get(k)]
    if not entries:
        raise ValueError("先保存当前轨迹，禁止在没有保留目标时清理")
    keep_runs = {Path(entry["source_run"]).resolve() for entry in entries}
    keep_files = {Path(entry[k]).resolve() for entry in entries for k in ("path", "report")}
    candidates = set()
    for p in planning.glob("*.mat"):
        if p.name.startswith(("online_", "realhome_to_preweld")) or re.fullmatch(r"\d{8}_\d{6}_\d{6}\.mat", p.name):
            candidates.add(p)
    candidates.update((root / "轨迹文件").glob("*.mat"))
    daily = re.compile(r"(?:preview|prepare)_\d{8}_\d{6}_\d{6}")
    for run in (planning / "workflow_runs").iterdir():
        if run.is_dir() and daily.fullmatch(run.name) and run.resolve() not in keep_runs:
            for name in ("full_raw_trajectory.mat", "full_raw_trajectory.json",
                         "offline_optimized_trajectory.mat", "offline_optimized_trajectory.json"):
                if (run / name).is_file(): candidates.add(run / name)
    for p in (root / "planning_runs").glob("hmi_*/**/trajectory.mat"):
        candidates.add(p)
    for p in current.glob("*.mat"):
        if p.resolve() not in keep_files:
            candidates.add(p)
            companion = p.with_name(p.stem + "_report.json")
            if companion.is_file(): candidates.add(companion)
    rows = []
    for path in sorted(candidates):
        if path.is_symlink() or not path.is_file(): continue
        resolved = path.resolve(strict=True)
        if not resolved.is_relative_to(root) or resolved in keep_files or any(resolved.is_relative_to(p) for p in keep_runs):
            continue
        stat = path.stat()
        rows.append(dict(path=str(resolved), size=stat.st_size, sha256=file_hash(resolved), mtime_ns=stat.st_mtime_ns))
    digest = hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()
    return dict(schema_version=1, created_at=datetime.now().astimezone().isoformat(),
        root=str(root), inventory_sha256=digest, files=rows, count=len(rows),
        total_bytes=sum(r["size"] for r in rows), protected_runs=sorted(map(str, keep_runs)),
        protected_files=sorted(map(str, keep_files)),
        scope="Old daily HMI trajectory payloads only; experiments/models/calibration/logs/reports retained",
        recovery="Desktop Trash", robot_motion="NO")


def apply(plan):
    root = Path(plan["root"]).resolve(strict=True)
    # Don't delete the backing file of an in-memory HMI or a running planner.
    processes = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
    for line in processes.splitlines():
        if "python3 main.py" in line:
            pid = line.strip().split()[0]
            try: cwd = Path("/proc") / pid / "cwd"; is_hmi = cwd.resolve() == root / "ur5_hmi"
            except OSError: is_hmi = False
            if is_hmi: raise RuntimeError("请先关闭HMI，再清理旧轨迹")
        if "MATLAB -batch" in line and str(root / "matlab仿真") in line:
            raise RuntimeError("当前有MATLAB规划进程，暂停清理")
    # Recompute the complete protected-target selection immediately before use.
    if inventory(root)["inventory_sha256"] != plan["inventory_sha256"]:
        raise RuntimeError("清理目标已变化，必须重新核对清单")
    moved = []
    for row in plan["files"]:
        path = Path(row["path"])
        if path.is_symlink() or file_hash(path) != row["sha256"]:
            raise RuntimeError("轨迹内容变化，停止清理：" + str(path))
        subprocess.run(["gio", "trash", "--", str(path)], check=True, capture_output=True, text=True)
        moved.append(row)
        yield dict(path=str(path), trashed=True, size=row["size"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--apply-sha")
    args = parser.parse_args()
    if args.apply_sha:
        plan = json.loads(args.manifest.read_text())
        if args.apply_sha != plan["inventory_sha256"]: raise ValueError("清单摘要不一致")
        result = dict(plan, moved=[], mode="APPLY_TO_TRASH")
        progress = args.manifest.with_name(args.manifest.stem + "_trashed.json")
        progress.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        for row in apply(plan):
            result["moved"].append(row)
            progress.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(dict(trashed_count=len(result["moved"]), recovery="Desktop Trash", manifest=str(progress)), ensure_ascii=False))
    else:
        plan = inventory(args.root.resolve(strict=True))
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(json.dumps(plan, ensure_ascii=False, indent=2))
        print(json.dumps({k: v for k,v in plan.items() if k != "files"}, ensure_ascii=False, indent=2))
