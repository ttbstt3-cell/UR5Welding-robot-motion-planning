"""Use real offline report; no ROS node, no GUI launch, no current-result publish."""
import copy
import json
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root / "ur5_hmi"))
from gui.planning_workflow import PlanningWorkflow, PlanningStage
from trajectory.trajectory_loader import load_trajectory
from trajectory.trajectory_utils import normalize_trajectory
from safety.execution_preparation import offline_context
from safety.robot_motion_limits import check_motion

here = Path(__file__).resolve().parent
production = "--production" in sys.argv
path = here / ("production_current" if production else "staged_current") / "report.json"
report = json.loads(path.read_text())
artifact = Path(report["artifacts"]["trajectory_mat"])
state = PlanningWorkflow(raw_path=Path(json.loads((here.parent / "prepare_20261019_152229_336637" / "request.json").read_text())["rawFile"]))
state.begin_preparation()
state.preparation_complete(artifact, report)
assert state.stage == PlanningStage.PHYSICAL_VALIDATION_PENDING and state.ready_path is None
data = load_trajectory(str(artifact))
traj = normalize_trajectory(data, source_path=str(artifact))
context = offline_context(traj, path, report["version_validation"]["current_binding"], traj.metadata["realhome_version"])
assert traj.accelerations is None  # Diagnostic acceleration must not change cubic to quintic.
assert bool(traj.metadata["executionReady"]) is False and traj.metadata["path_time_reference"]
execution_reasons, metrics = check_motion(traj.positions, traj.time_from_start, traj.velocities, execution=True)
assert any("MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING" in reason for reason in execution_reasons)
cases = []
for key in ("success", "continuous_planning_complete"):
    r = copy.deepcopy(report); r[key] = False; cases.append((key, r))
for key in ("complete", "collision_pass", "joint_limits_pass", "analytic_extrema_pass"):
    r = copy.deepcopy(report); r["controller_validation"][key] = False; cases.append((key, r))
for key, value in (("max_tcp_time_sync_error_mm", .5), ("max_tcp_contour_error_mm", .5),
                   ("max_tcp_orientation_error_deg", .50001), ("max_joint_velocity_rad_per_sec", 4),
                   ("max_joint_acceleration_rad_per_sec2", 41), ("sample_count", 4999),
                   ("interpolation_mode", "quintic"), ("max_tcp_time_sync_error_mm", float("nan"))):
    r = copy.deepcopy(report); r["controller_validation"][key] = value; cases.append((key, r))
r = copy.deepcopy(report); r["executionReady"] = True; cases.append(("execution claim", r))
r = copy.deepcopy(report); r["version_validation"]["pass"] = False; cases.append(("version mismatch", r))
r = copy.deepcopy(report); r["offline_checks"].pop("interface_c2"); cases.append(("missing C2", r))
for name, r in cases:
    try:
        PlanningWorkflow().preparation_complete(artifact, r)
    except ValueError:
        continue
    raise AssertionError("False PASS accepted: " + name)
result = dict(pass_all=True, negative_cases=len(cases), negative_names=[n for n, _ in cases],
              artifact_load=True, offline_context=True, executionReady=False,
              state=state.stage.value, robot_motion="NO", context_keys=list(context),
              actual_execution_remains_blocked=execution_reasons)
(here / ("production_workflow_contract.json" if production else "new_workflow_contract.json")).write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
