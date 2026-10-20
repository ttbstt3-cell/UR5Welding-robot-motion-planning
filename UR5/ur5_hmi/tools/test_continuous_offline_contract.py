"""Reject inconsistent PASS flags, execution claims and missing audit evidence."""
from __future__ import annotations
import copy
import json
from pathlib import Path
import sys
import tempfile
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.planning_workflow import PlanningStage, PlanningWorkflow
from gui.offline_planning_report import save_analysis_bundle


def run(report_path: Path, output_path: Path):
    report = json.loads(report_path.read_text())
    artifact = Path(report["artifacts"]["trajectory_mat"])
    state = PlanningWorkflow(raw_path=artifact)
    state.begin_preparation()
    assert state.stage == PlanningStage.CONTINUOUS_PLANNING
    state.preparation_complete(artifact, report)
    assert state.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING and state.ready_path is None
    cases = []
    for field in ("success", "continuous_planning_complete"):
        r = copy.deepcopy(report); r[field] = False; cases.append((field, r))
    for field in ("complete", "collision_pass", "joint_limits_pass", "analytic_extrema_pass"):
        r = copy.deepcopy(report); r["controller_validation"][field] = False; cases.append((field, r))
    for field, bad in (("max_tcp_time_sync_error_mm", .568271),
                       ("max_tcp_orientation_error_deg", .548976),
                       ("max_joint_acceleration_rad_per_sec2", .3),
                       ("max_joint_velocity_rad_per_sec", .1),
                       ("max_tcp_contour_error_mm", float("nan")),
                       ("sample_count", 4999)):
        r = copy.deepcopy(report); r["controller_validation"][field] = bad; cases.append((field, r))
    r = copy.deepcopy(report); r["executionReady"] = True; cases.append(("execution claim", r))
    r = copy.deepcopy(report); r["version_validation"]["pass"] = False; cases.append(("version mismatch", r))
    r = copy.deepcopy(report); r["offline_checks"].pop("interface_c2"); cases.append(("missing C2", r))
    r = copy.deepcopy(report); r["controller_validation"]["interpolation_mode"] = "quintic"; cases.append(("wrong interpolation", r))
    rejected = []
    for name, case in cases:
        invalid = PlanningWorkflow(raw_path=artifact)
        try:
            invalid.preparation_complete(artifact, case)
        except ValueError as exc:
            invalid.fail("prepare", str(exc))
        else:
            raise AssertionError("Invalid evidence accepted: " + name)
        assert invalid.stage == PlanningStage.CONTINUOUS_PLANNING_FAILED
        assert invalid.ready_path is None and invalid.optimized_path is None
        assert invalid.raw_path == artifact
        rejected.append(name)
    with tempfile.TemporaryDirectory() as folder:
        archive = Path(folder)/"offline.zip"
        save_analysis_bundle(report_path, archive)
        with zipfile.ZipFile(archive) as bundle:
            names = bundle.namelist()
            assert not any("execute_ready" in n for n in names)
            assert "OFFLINE_ONLY.txt" in names
            assert json.loads(bundle.read("report.json"))["executionReady"] is False
    output_path.write_text(json.dumps(dict(pass_all=True, rejected_false_pass_cases=rejected,
        offline_pass_state=state.stage.value, ready_path=None, robot_motion="NO"), indent=2))
    print("OFFLINE_CONTRACT_PASS", len(rejected), "negative cases")


if __name__ == "__main__":
    run(Path(sys.argv[1]), Path(sys.argv[2]))
