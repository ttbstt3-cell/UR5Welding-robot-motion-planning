"""Compile inspectable regression metrics from completed offline evidence."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read(relative):
    return json.loads((HERE / relative).read_text())


def audit_metrics(audit):
    fields = ["sample_count", "max_tcp_contour_error_mm", "rms_tcp_contour_error_mm",
              "max_tcp_time_sync_error_mm", "rms_tcp_time_sync_error_mm",
              "max_tcp_orientation_error_deg", "min_positive_clearance_mm",
              "collision_pass", "joint_limits_pass", "complete"]
    return {k: audit[k] for k in fields}


if __name__ == "__main__":
    derivatives = read("derivative_analysis.json")
    output = {"executionReady": False, "robot_motion": "NO", "cases": {}}
    for case in ("current", "historical"):
        parallel = read(f"{case}_spline/parallel_validation.json")
        serial = read(f"{case}_spline/full_validation.json")
        output["cases"][case] = {
            "derivatives": derivatives[case],
            "optimized_audit": audit_metrics(parallel["audit"]),
            "serial_audit_sec": serial["audit"]["performance"]["total_sec"],
            "parallel_audit_sec": parallel["audit"]["performance"]["total_sec"],
            "all_q_exact": parallel["all_execution_q_exact"],
            "all_tcp_exact": parallel["all_execution_tcp_exact"],
            "path_reference_exact": parallel["reference_geometry_exact"],
        }
        assert parallel["full_validation_pass"] and not parallel["executionReady"]
    production = read("production_current/report.json")
    assert production["success"] and production["final_acceptance_pass"]
    assert not production["executionReady"] and production["stage"] == "PHYSICAL_VALIDATION_PENDING"
    output["production"] = {
        "planning_time": production["planning_time"],
        "stages": production["stage_timings"],
        "audit": audit_metrics(production["controller_validation"]),
        "realhome_version": production["realhome_version"],
        "collision_performance": production["controller_validation"]["performance"],
    }
    output["baseline_profile"] = read("current/diagnosis.json")["audit"]["performance"]
    output["baseline_full_equivalence"] = read("full_equivalence.json")["pass"]
    output["backend_contracts"] = read("production_backend_contracts.json")
    assert output["backend_contracts"]["pass"]
    output["protected"] = read("protected_verification.json")["unchanged_all"]
    assert output["protected"]
    (HERE / "comparison_summary.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(output["production"], ensure_ascii=False, indent=2))
