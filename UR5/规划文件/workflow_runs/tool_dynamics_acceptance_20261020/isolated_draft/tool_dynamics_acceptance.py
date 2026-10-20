"""Scoped commissioning evidence, not a safety certificate or robot writer.

An imported, human-reviewed record is checked against its attached evidence,
the unchanged trajectory/models, analytical peaks and fresh RTDE payload.
Controller registration or a boolean alone can never approve a record.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from safety.execution_preparation import file_hash
from safety.joint_limits import JOINT_NAMES

ROOT = Path(__file__).resolve().parents[2]
RECORD = ROOT / "real_tool_dynamics_acceptance.json"
MAX_AGE_SEC = 10.0
BINDING_FILES = ("robot_motion_limits.json", "real_robot_description_calibrated.urdf",
    "real_tcp_calibration_active.json", "real_gun_geometry/real_gun_geometry.json",
    "real_gun_mount_calibration.json", "workpiece_pose_real.json",
    "environment_geometry_real.json", "fixed_environment_contacts.json")


def bindings():
    return {name: file_hash(ROOT / name) for name in BINDING_FILES}


def command_scope(q, t, v):
    """Same q/v and nanosecond timestamps as the actual ROS serialization."""
    h = hashlib.sha256(json.dumps(JOINT_NAMES).encode())
    stamps = np.asarray([int(float(s)) * 10**9 + int(round((float(s)-int(float(s)))*1e9))
                         for s in np.asarray(t).reshape(-1)], dtype="<i8")
    for a in (np.asarray(q, dtype="<f8", order="C"), stamps, np.asarray(v, dtype="<f8", order="C")):
        if not np.isfinite(a).all():
            raise ValueError("TOOL_ACCEPTANCE_INVALID: nonfinite trajectory")
        h.update(str(a.shape).encode()); h.update(a.tobytes())
    return h.hexdigest()


def vector(value, size, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError("TOOL_ACCEPTANCE_INVALID: " + name)
    return a


def payload_sample(result, received=None):
    if (result.get("operation") != "read" or result.get("success") is not True
            or result.get("output_only") is not True or result.get("input_recipe_empty") is not True):
        raise ValueError("TOOL_PAYLOAD_UNKNOWN: output-only RTDE result required")
    s = result["snapshot"]
    mass = float(s["payload"])
    if not np.isfinite(mass) or not 0 < mass <= 5 or not np.isfinite(float(s["timestamp"])) or s["timestamp"] <= 0:
        raise ValueError("TOOL_PAYLOAD_UNKNOWN: invalid CB3 payload/timestamp")
    vector(s["payload_cog"], 3, "payload_cog")
    vector(s["tcp_offset"], 6, "tcp_offset")
    return {"snapshot": dict(s), "received_monotonic": time.monotonic() if received is None else received,
            "source": "URCL_OUTPUT_ONLY_RTDE"}


def fresh_sample(sample, now=None):
    try:
        age = (time.monotonic() if now is None else now) - sample["received_monotonic"]
        if sample["source"] != "URCL_OUTPUT_ONLY_RTDE" or not np.isfinite(age) or not 0 <= age <= MAX_AGE_SEC:
            return None
        return sample
    except (TypeError, KeyError):
        return None


def draft(q, t, v, live=None):
    sample = fresh_sample(live)
    return dict(schema_version=1, status="DRAFT", executionReady=False,
        interpolation="positions_plus_velocities_cubic_hermite", trajectory_command_sha256=command_scope(q,t,v),
        bindings=bindings(), joint_names=JOINT_NAMES,
        controller_registration=sample["snapshot"] if sample else None,
        verified_tool={"payload_kg": None, "cog_m": None, "frame": "controller_tool_output_flange"},
        approved_limits={"velocity_rad_s": None, "acceleration_rad_s2": None},
        evidence={"physical_tool_and_capacity": {"path": "", "sha256": ""},
                  "load_dependent_dynamics": {"path": "", "sha256": ""}},
        review={"reviewer": "", "reviewed_at": "", "valid_until": "", "physical_tool_confirmed": False},
        note="Controller registration is not a physical measurement. Attach reviewed engineering/commissioning evidence; no robot commands.")


def validate(record, q, t, v, live, metrics, limits):
    def require(condition, reason):
        if not condition:
            raise ValueError("TOOL_ACCEPTANCE_" + reason)
    require(isinstance(record, dict) and record.get("schema_version") == 1, "INVALID_SCHEMA")
    require(record.get("status") == "REVIEWED" and record.get("executionReady") is False, "PENDING_REVIEW")
    require(record.get("interpolation") == "positions_plus_velocities_cubic_hermite" and record.get("joint_names") == JOINT_NAMES, "INTERPOLATION_MISMATCH")
    require(record.get("trajectory_command_sha256") == command_scope(q,t,v), "TRAJECTORY_MISMATCH")
    require(record.get("bindings") == bindings(), "VERSION_MISMATCH")
    review = record["review"]
    require(isinstance(review.get("reviewer"), str) and bool(review["reviewer"].strip()) and review.get("physical_tool_confirmed") is True, "PENDING_PHYSICAL_CONFIRMATION")
    dates = [datetime.fromisoformat(review[k]) for k in ("reviewed_at", "valid_until")]
    require(all(d.tzinfo is not None for d in dates) and dates[0] <= datetime.now(timezone.utc) <= dates[1], "EXPIRED_OR_INVALID_DATE")
    for key in ("physical_tool_and_capacity", "load_dependent_dynamics"):
        evidence = record["evidence"][key]
        path = Path(evidence["path"])
        require(path.is_absolute() and path.is_file() and path.stat().st_size > 0, "MISSING_EVIDENCE_" + key)
        require(path.suffix.lower() in (".pdf", ".png", ".jpg", ".jpeg", ".txt", ".md"), "INVALID_EVIDENCE_TYPE")
        require(file_hash(path) == evidence["sha256"], "EVIDENCE_CHANGED_" + key)
    sample = fresh_sample(live)
    require(sample is not None, "LIVE_PAYLOAD_UNKNOWN_OR_STALE")
    s = sample["snapshot"]
    require(s.get("robot_mode") == 7 and s.get("safety_mode") == 1, "ROBOT_NOT_READY")
    tool = record["verified_tool"]
    mass = float(tool["payload_kg"])
    cog = vector(tool["cog_m"], 3, "cog_m")
    require(tool.get("frame") == "controller_tool_output_flange" and np.isfinite(mass) and 0 < mass <= 5, "INVALID_TOOL")
    require(abs(mass-float(s["payload"])) <= 1e-6 and np.allclose(cog, vector(s["payload_cog"],3,"live cog"),rtol=0,atol=1e-6), "LIVE_PAYLOAD_MISMATCH")
    active = json.loads((ROOT/"real_tcp_calibration_active.json").read_text())
    require(np.allclose(vector(s["tcp_offset"],6,"live tcp"),vector(active["controller_tcp_offset"],6,"active tcp"),rtol=0,atol=1e-7), "LIVE_TCP_MISMATCH")
    for key, measured, cap in (("velocity_rad_s", "max_velocity_per_joint", "velocity_rad_s"),
                               ("acceleration_rad_s2", "max_acceleration_per_joint", "acceleration_rad_s2")):
        approved = vector(record["approved_limits"][key], 6, key)
        require(np.all(approved>0) and np.all(approved<=np.asarray(limits[cap])), "INVALID_APPROVED_LIMIT_"+key)
        require(np.all(np.asarray(metrics[measured])<=approved), "LIMIT_EXCEEDED_"+key)
    return dict(status="MATCHED_REVIEWED_RECORD", reviewer=review["reviewer"], payload_kg=mass,
                cog_m=cog.tolist(), executionReady=False,
                note="Evidence authenticity and physical suitability require human review; other execution checks remain mandatory")


def execution_reasons(q,t,v,live,metrics,limits,path=None):
    path = RECORD if path is None else Path(path)
    try:
        record = json.loads(path.read_text())
        validate(record,q,t,v,live,metrics,limits)
        return []
    except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
        return ["MOTION_LIMITS_PAYLOAD_ACCELERATION_PENDING：负载/重心及可用加速度验收尚未匹配："+str(error)]
