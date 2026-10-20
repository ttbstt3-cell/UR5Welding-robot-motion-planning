"""Readable-file/autoload regression with real saved inputs, no ROS or motion."""
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety.execution_preparation import file_hash, offline_context
from trajectory.result_store import publish_result, latest_result, read_index
from trajectory.trajectory_loader import load_trajectory


def run(prepared, current_preview, output):
    report = prepared / "report.json"
    request = json.loads((prepared / "request.json").read_text())
    raw = Path(request["rawFile"])
    source = prepared / "offline_optimized_trajectory.mat"
    original_hashes = [file_hash(p) for p in (report, raw, source)]
    with tempfile.TemporaryDirectory(prefix="ur5_readable_results_TEST_") as temp:
        folder = Path(temp)
        p = publish_result(raw, raw.parent / "report.json", folder, "preview")
        assert p.name.startswith("预览轨迹_仅预演_") and latest_result(folder) == p
        assert file_hash(p) == file_hash(raw)
        optimized = publish_result(source, report, folder, "prepare", parent_preview_sha256=file_hash(raw))
        assert optimized.name.startswith("连续规划完成轨迹_")
        assert latest_result(folder) == optimized
        traj = load_trajectory(str(optimized))
        companion = optimized.with_name(optimized.stem + "_report.json")
        offline_context(traj, companion, traj.metadata["calibration_binding"], traj.metadata["realhome_version"])
        assert file_hash(optimized) == file_hash(source) and file_hash(companion) == file_hash(report)
        original_index = (folder / "最新轨迹索引.json").read_bytes()
        try: publish_result(source, report, folder, "preview")
        except ValueError: pass
        else: raise AssertionError("Wrong phase published")
        assert (folder / "最新轨迹索引.json").read_bytes() == original_index
        broken = folder / "bad_report.json"
        bad = json.loads(report.read_text()); bad["controller_validation"]["collision_pass"] = False
        broken.write_text(json.dumps(bad))
        try: publish_result(source, broken, folder, "prepare", parent_preview_sha256=file_hash(raw))
        except ValueError: pass
        else: raise AssertionError("Collision failure published as finished")
        assert latest_result(folder) == optimized
        new = publish_result(current_preview, current_preview.parent / "report.json", folder, "preview")
        assert latest_result(folder) == new and read_index(folder)["prepare"] is None
        # Missing/tampered latest never silently falls back to an older path.
        new.write_bytes(b"TEST_CORRUPTED_TRAJECTORY")
        try: latest_result(folder)
        except ValueError: pass
        else: raise AssertionError("Tampered latest accepted")
    assert original_hashes == [file_hash(p) for p in (report, raw, source)]
    result = dict(pass_all=True, byte_identical_q_time_velocity_metadata=True,
        renamed_trajectory_and_associated_report_pass=True, latest_selection_pass=True,
        new_preview_invalidates_old_continuous_selection=True,
        invalid_phase_and_failed_collision_rejected=True, tampered_latest_rejected=True,
        original_inputs_unchanged=True, executionReady=False, robot_motion="NO")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), Path(sys.argv[3]).resolve())
