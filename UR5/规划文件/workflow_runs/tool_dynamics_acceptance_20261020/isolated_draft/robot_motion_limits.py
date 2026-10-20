"""Controller/URDF sourced offline limits and fail-closed execution checks.

Primary ConfigurationData is NOT the installation's safety configuration or
a load-dependent acceleration recommendation. No robot write API is used.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import socket
import struct
import time
import xml.etree.ElementTree as ET
import numpy as np
from safety.joint_limits import JOINT_NAMES

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "robot_motion_limits.json"
URDF = ROOT / "real_robot_description_calibrated.urdf"
LEGACY_V = float(np.deg2rad(5))
LEGACY_A = .261799


def decode_configuration(packet):
    if len(packet) < 445 or struct.unpack_from("!IB", packet) != (len(packet), 6):
        raise ValueError("MOTION_LIMITS_INVALID: ConfigurationData length/type")
    values = np.asarray(struct.unpack_from("!24d", packet, 5)).reshape(12, 2)
    positions, motion = values[:6], values[6:]
    if (not np.isfinite(values).all() or np.any(positions[:, 0] >= positions[:, 1])
            or np.any(motion <= 0)):
        raise ValueError("MOTION_LIMITS_INVALID: nonfinite/nonpositive controller limits")
    defaults = struct.unpack_from("!5d", packet, 197)
    return dict(position_limits_rad=positions.tolist(), velocity_rad_s=motion[:, 0].tolist(),
                acceleration_rad_s2=motion[:, 1].tolist(),
                default_joint_velocity_rad_s=defaults[0], default_joint_acceleration_rad_s2=defaults[1])


def read_configuration(host, timeout=3.):
    """Bounded recv-only connection to the UR read-only primary port."""
    deadline = time.monotonic() + timeout
    with socket.create_connection((host, 30011), timeout=min(timeout, 1.)) as connection:
        connection.settimeout(.25)
        buffer = b""
        while time.monotonic() < deadline:
            try: data = connection.recv(65536)
            except socket.timeout: continue
            if not data: break
            buffer += data
            while len(buffer) >= 5:
                length, kind = struct.unpack_from("!IB", buffer)
                if not 5 <= length <= 2_000_000:
                    raise ValueError("Invalid primary packet length")
                if len(buffer) < length: break
                packet, buffer = buffer[:length], buffer[length:]
                if kind != 16: continue
                offset = 5
                while offset + 5 <= length:
                    size, subkind = struct.unpack_from("!IB", packet, offset)
                    if size < 5 or offset + size > length:
                        raise ValueError("Invalid primary subpacket length")
                    if subkind == 6:
                        raw = packet[offset:offset+size]
                        return dict(decode_configuration(raw), raw_configuration_hex=raw.hex(),
                                    raw_configuration_sha256=hashlib.sha256(raw).hexdigest())
                    offset += size
    raise TimeoutError("MOTION_LIMITS_UNKNOWN: no ConfigurationData from read-only port30011")


def load_limits(path=PROFILE):
    content = Path(path).read_bytes(); profile = json.loads(content)
    if profile.get("schema_version") != 1 or profile.get("source") != "UR_PRIMARY_READ_ONLY_CONFIGURATION":
        raise ValueError("MOTION_LIMITS_INVALID: unsupported profile source/schema")
    if profile.get("joint_names") != JOINT_NAMES:
        raise ValueError("MOTION_LIMITS_INVALID: joint order")
    model = URDF.read_bytes()
    if hashlib.sha256(model).hexdigest() != profile.get("urdf_sha256"):
        raise ValueError("MOTION_LIMITS_STALE: URDF changed")
    raw = bytes.fromhex(profile["controller"]["raw_configuration_hex"])
    if hashlib.sha256(raw).hexdigest() != profile["controller"]["raw_configuration_sha256"]:
        raise ValueError("MOTION_LIMITS_INVALID: controller snapshot hash")
    reported = decode_configuration(raw)
    for field, value in reported.items():
        if profile["controller"].get(field) != value:
            raise ValueError("MOTION_LIMITS_INVALID: decoded snapshot differs: " + field)
    joints = {j.attrib["name"]: j for j in ET.fromstring(model).findall("joint")}
    rows = [joints[name].find("limit").attrib for name in JOINT_NAMES]
    qlo = np.maximum([float(r["lower"]) for r in rows], np.asarray(reported["position_limits_rad"])[:, 0])
    qhi = np.minimum([float(r["upper"]) for r in rows], np.asarray(reported["position_limits_rad"])[:, 1])
    velocity = np.minimum([float(r["velocity"]) for r in rows], reported["velocity_rad_s"])
    acceleration = np.asarray(reported["acceleration_rad_s2"])
    evidence = profile.get("pendant_evidence")
    normal_speed = None
    if evidence is not None:
        for kind in ("general", "joint"):
            photo = (ROOT / evidence[kind+"_photo"]).resolve(strict=True)
            if hashlib.sha256(photo.read_bytes()).hexdigest() != evidence[kind+"_photo_sha256"]:
                raise ValueError("MOTION_LIMITS_INVALID: pendant photo changed")
        speeds = np.deg2rad(np.asarray(evidence["normal_joint_velocity_deg_s"], dtype=float))
        normal_speed = float(evidence["normal_tcp_speed_m_s"])
        if speeds.shape != (6,) or not np.isfinite(speeds).all() or np.any(speeds <= 0) or not np.isfinite(normal_speed) or normal_speed <= 0:
            raise ValueError("MOTION_LIMITS_INVALID: pendant speed values")
        velocity = np.minimum(velocity, speeds)
    if np.any(qlo >= qhi) or not np.isfinite(velocity).all() or not np.isfinite(acceleration).all():
        raise ValueError("MOTION_LIMITS_INVALID: model/controller intersection")
    # Installation limits cannot be inferred from this configuration packet.
    # A future measured installation record must be independently implemented
    # and validated; flipping a JSON boolean must not grant authorization.
    return dict(profile_sha256=hashlib.sha256(content).hexdigest(), profile_path=str(Path(path).resolve()),
        source=profile["source"], joint_names=JOINT_NAMES, urdf_sha256=profile["urdf_sha256"],
        position_lower_rad=qlo.tolist(), position_upper_rad=qhi.tolist(),
        velocity_rad_s=velocity.tolist(), acceleration_rad_s2=acceleration.tolist(),
        scalar_velocity_rad_s=float(min(velocity)), scalar_acceleration_rad_s2=float(min(acceleration)),
        normal_joint_speed_evidence_present=evidence is not None, normal_tcp_speed_m_s=normal_speed,
        installation_safety_verified=False, payload_acceleration_verified=False,
        status="OFFLINE_LIMITS_AVAILABLE_EXECUTION_INCREASE_PENDING",
        note="Configuration ceilings, not safety settings or recommended operating speeds")


def validate_contract(contract):
    current = load_limits()
    if contract != current:
        raise ValueError("MOTION_LIMITS_STALE: profile/model/derived limits differ; revalidate trajectory")
    return current


def cubic_extrema(q, t, velocities):
    q = np.asarray(q, dtype=float); t = np.asarray(t, dtype=float).reshape(-1)
    v = np.asarray(velocities, dtype=float)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or v.shape != q.shape or len(t) != len(q):
        raise ValueError("MOTION_LIMITS_INVALID: P+V trajectory shape")
    if not np.isfinite(q).all() or not np.isfinite(v).all() or not np.isfinite(t).all() or t[0] < 0 or np.any(np.diff(t) <= 0):
        raise ValueError("MOTION_LIMITS_INVALID: nonfinite data or timestamps")
    h = np.diff(t)[:, None]; d = np.diff(q, axis=0)
    b = h*v[:-1]; c = 3*d-2*b-h*v[1:]; e = -2*d+b+h*v[1:]
    vmax = np.max(np.abs(v), axis=0)
    amax = np.max(np.maximum(np.abs(2*c/h**2), np.abs((2*c+6*e)/h**2)), axis=0)
    qmin, qmax = q.min(axis=0), q.max(axis=0)
    for j in range(6):
        for k in range(len(h)):
            if abs(e[k,j]) > 1e-16:
                u = -c[k,j]/(3*e[k,j])
                if 0 < u < 1:
                    vmax[j] = max(vmax[j], abs((b[k,j]+2*c[k,j]*u+3*e[k,j]*u*u)/h[k,0]))
            coefficients = [3*e[k,j], 2*c[k,j], b[k,j]]
            for u in np.roots(np.trim_zeros(coefficients, "f")):
                if abs(u.imag) < 1e-12 and 0 < u.real < 1:
                    u = u.real
                    value = q[k,j]+b[k,j]*u+c[k,j]*u*u+e[k,j]*u**3
                    qmin[j], qmax[j] = min(qmin[j], value), max(qmax[j], value)
    return dict(max_velocity_per_joint=vmax, max_acceleration_per_joint=amax,
                position_min=qmin, position_max=qmax)


def tcp_speed_upper_bound(max_joint_velocity):
    """Triangle-inequality bound valid for every pose, not sampled FK speed.

    Each joint's lever arm is bounded by the sum of downstream fixed lengths
    plus the active flange->TCP length. This can reject conservatively; it
    never uses finite differences as a proof of maximum Cartesian speed.
    """
    model = URDF.read_bytes()
    active_file = ROOT / "real_tcp_calibration_active.json"
    raw = active_file.read_bytes(); active = json.loads(raw)
    transform = np.asarray(active["T_flange_tcp"], dtype=float)
    if (active.get("status") != "ACTIVE" or active.get("valid") is not True
            or active.get("urdf_sha256") != hashlib.sha256(model).hexdigest()
            or transform.shape != (4,4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0,0,0,1], atol=1e-10)
            or not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(transform[:3,:3]),1,rtol=0,atol=1e-7)):
        raise ValueError("MOTION_LIMITS_TCP_UNKNOWN: active flange/TCP model invalid")
    joints = ET.fromstring(model).findall('joint')
    child = {j.find('child').get('link'):j for j in joints}
    link='flange'; radius=float(np.linalg.norm(transform[:3,3])); radii=np.zeros(6); found=set()
    for _ in range(len(joints)+1):
        if link not in child: break
        joint=child[link]; name=joint.get('name')
        if name in JOINT_NAMES:
            axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            if not np.isclose(np.linalg.norm(axis),1,rtol=0,atol=1e-8):
                raise ValueError('MOTION_LIMITS_TCP_UNKNOWN: nonunit URDF axis')
            index=JOINT_NAMES.index(name); radii[index]=radius; found.add(name)
        origin=joint.find('origin')
        if origin is not None: radius+=np.linalg.norm(np.fromstring(origin.get('xyz','0 0 0'),sep=' '))
        link=joint.find('parent').get('link')
    if found != set(JOINT_NAMES): raise ValueError('MOTION_LIMITS_TCP_UNKNOWN: incomplete flange chain')
    return float(radii @ np.asarray(max_joint_velocity)), radii, hashlib.sha256(raw).hexdigest()


def check_motion(q, t, velocities, *, execution=False, path=PROFILE, live_payload=None, acceptance_path=None):
    limits = load_limits(path); metrics = cubic_extrema(q, t, velocities); reasons = []
    for key, cap, label in (("max_velocity_per_joint", "velocity_rad_s", "VELOCITY_LIMIT"),
                           ("max_acceleration_per_joint", "acceleration_rad_s2", "ACCELERATION_LIMIT")):
        for j in np.flatnonzero(metrics[key] > np.asarray(limits[cap])):
            reasons.append(f"{label}: J{j+1} {metrics[key][j]:.9g} > {limits[cap][j]:.9g}")
    for j in np.flatnonzero((metrics["position_min"] < limits["position_lower_rad"]) |
                            (metrics["position_max"] > limits["position_upper_rad"])):
        reasons.append(f"POSITION_LIMIT: J{j+1} controller/URDF intersection violated")
    if limits['normal_tcp_speed_m_s'] is not None:
        bound, radii, tcp_hash = tcp_speed_upper_bound(metrics['max_velocity_per_joint'])
        metrics.update(tcp_linear_speed_upper_bound_m_s=bound,tcp_lever_arm_upper_bounds_m=radii,
                       active_tcp_sha256=tcp_hash)
        if bound>limits['normal_tcp_speed_m_s']:
            reasons.append(f"TCP_SPEED_BOUND_LIMIT: conservative bound {bound:.9g} > {limits['normal_tcp_speed_m_s']:.9g} m/s; dense FK-speed optimization required")
    if execution and (np.max(metrics["max_velocity_per_joint"]) > LEGACY_V or
                      np.max(metrics["max_acceleration_per_joint"]) > LEGACY_A):
        from safety.tool_dynamics_acceptance import execution_reasons
        reasons.extend(execution_reasons(q,t,velocities,live_payload,metrics,limits,acceptance_path))
    return reasons, metrics


def message_motion_reasons(message, live_payload=None):
    try:
        if message.joint_names != JOINT_NAMES or any(len(p.velocities) != 6 or p.accelerations for p in message.points):
            return ["MOTION_LIMITS_INVALID: standard positions+velocities cubic input required"]
        return check_motion([p.positions for p in message.points],
            [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in message.points],
            [p.velocities for p in message.points], execution=True, live_payload=live_payload)[0]
    except (OSError, ValueError, KeyError, TypeError, AttributeError, IndexError) as exc:
        return ["MOTION_LIMITS_UNKNOWN: " + str(exc)]
