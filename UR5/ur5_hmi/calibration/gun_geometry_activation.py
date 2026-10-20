"""Explicit operator-confirmed activation of the independent REAL gun."""
import hashlib
import json
import shutil
import time
from pathlib import Path
import numpy as np
from scipy.io import loadmat
from gui.robot_view import UrdfKinematicModel


def activate_real_gun(root: Path, record: dict) -> dict:
    if record.get('status') != 'CONFIRMED' or record.get('confirmed_by_operator') is not True:
        raise ValueError('需要人工确认安装变换')
    path = root / 'real_gun_geometry' / 'real_gun_geometry.json'
    manifest = json.loads(path.read_text())
    source_hash = hashlib.sha256(Path(manifest['source']['stl']).read_bytes()).hexdigest()
    if source_hash != manifest['source']['sha256'] or source_hash != record.get('source_sha256'):
        raise ValueError('STL版本不一致')
    T = np.asarray(record['T_flange_source'], dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or
        not np.allclose(T[3], [0, 0, 0, 1]) or
        not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6) or
        not np.isclose(np.linalg.det(T[:3, :3]), 1)):
        raise ValueError('安装变换不是有效刚体变换')
    collision = manifest['collision']
    if collision.get('method') != 'CoppeliaSim simConvex.vhacd' or not collision.get('components'):
        raise ValueError('缺少实际V-HACD组件')
    for c in collision['components']:
        v = np.asarray(c['vertices_m'], dtype=float)
        if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all() or np.linalg.matrix_rank(v-v[0]) != 3:
            raise ValueError('无效凸组件')
    model = UrdfKinematicModel(root / 'real_robot_description_calibrated.urdf')
    frames = model.link_transforms_in_base(np.zeros(6))
    Tf0 = np.linalg.inv(frames['flange']) @ frames['tool0']
    Tfm = Tf0 @ loadmat(root / 'ur5_tool_calibration.mat')['T_tool0_mount']
    manifest['visual'].update(T_flange_source=T.tolist(), transform_status='CONFIRMED')
    collision.update(status='CONFIRMED', T_flange_source=T.tolist(),
                     T_mount_source=(np.linalg.inv(Tfm) @ T).tolist(),
                     T_tool0_source=(np.linalg.inv(Tf0) @ T).tolist())
    manifest['mounting'] = {'status': 'CONFIRMED', 'coordinate_frame': 'URDF flange',
                            'T_flange_source': T.tolist(), 'source_sha256': source_hash,
                            'record_version': record.get('version'), 'confirmed_by_operator': True,
                            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    manifest.update(status='ACTIVE', version=f'real-gun-{time.time_ns()}', requires_replan=True)
    # Confirmed gun and wire must use the same installation and the current
    # controller-authoritative planning TCP. Never retain a PREVIEW wire tag
    # or claim success if its definition cannot be built.
    from calibration.model_axis_wire import build_model_axis_wire
    wire = build_model_axis_wire(root, manifest)
    manifest['wire'].update(wire)
    manifest['wire_axis_status'] = wire['status']
    if record.get('confirmation_source'):
        manifest['mounting']['confirmation_source'] = record['confirmation_source']
    backup = path.parent / 'mount_versions' / f'manifest_{time.time_ns()}.json'
    backup.parent.mkdir(exist_ok=True); shutil.copy2(path, backup)
    temp = path.with_suffix('.json.tmp')
    temp.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    temp.replace(path)
    return manifest
