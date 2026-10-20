#!/usr/bin/env python3
"""Stage a replacement CAD mesh and fit its mounting holes, without ROS.

Source vertices remain untouched. Four-hole symmetry is resolved using the
previous gun's installation orientation, not TCP proximity. A new CAD source
always needs renewed operator confirmation before REAL planning activation.
"""
import argparse
import hashlib
import itertools
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import vtk
from audit_new_gun_stl import audit
from align_gun_mount_holes import mesh, boundaries, UrdfKinematicModel

ROOT = Path(__file__).resolve().parents[2]


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--folder', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    folder = args.folder.resolve()
    if folder.exists():
        raise ValueError('Use a new package directory; existing results must not be overwritten')
    result = audit(source)
    if result['unit_assessment']['inferred_unit'] != 'mm':
        raise ValueError('Unknown units: operator must resolve before processing')
    if result['mesh_integrity']['nonfinite_triangle_count']:
        raise ValueError('Nonfinite STL')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    backup = ROOT / 'backups' / ('before_Areal_gun_' + stamp)
    backup.mkdir(parents=True)
    shutil.copytree(ROOT / 'real_gun_geometry', backup / 'real_gun_geometry')
    for name in ('real_gun_mount_calibration.json', 'real_tcp_calibration_active.json',
                 'real_tcp_calibration_applied.json', 'workpiece_pose_real.json'):
        if (ROOT / name).is_file():
            shutil.copy2(ROOT / name, backup / name)
    folder.mkdir(parents=True)
    stl = folder / 'welding_gun_source.STL'
    shutil.copy2(source, stl)
    result['original_source_file'] = str(source)
    result['source_file'] = str(stl)
    save(folder / 'stl_audit.json', result)
    v, f = mesh(stl)
    # This CAD's mounting back plane is Z=0; verify actual circular boundary
    # and four bolt holes rather than assuming the minimum face is a flange.
    loops = boundaries(v, f, 2, 0.0, .002)
    plate = next((l for l in loops if abs(l['radius'] - 31.5) < .05 and l['rms'] < .01), None)
    if plate is None:
        raise ValueError('Expected 63 mm flange boundary not identified at source Z=0')
    holes = np.array([l['center'] for l in loops if abs(l['radius'] - 3) < .01
                      and 24 < np.linalg.norm(l['center'] - plate['center']) < 26])
    if holes.shape != (4, 2):
        raise ValueError('Exactly four 6 mm mounting holes are required')
    model = UrdfKinematicModel(ROOT / 'real_robot_description_calibrated.urdf')
    frames = model.link_transforms_in_base(np.zeros(6))
    wv, wf = mesh(Path('/opt/ros/humble/share/ur_description/meshes/ur5/collision/wrist3.stl'))
    Tw = np.linalg.inv(frames['flange']) @ frames['wrist_3_link'] @ model.link_meshes['wrist_3_link']['origin']
    wv = (wv @ Tw[:3, :3].T + Tw[:3, 3]) * 1000
    face = float(wv[:, 0].max())
    robot_loops = boundaries(wv, wf, 0, face, .01)
    robot_holes = np.array([l['center'] for l in robot_loops if 2 < l['radius'] < 3])
    if robot_holes.shape != (4, 2):
        raise ValueError('UR5 four-hole pattern not identified')
    candidates = []
    for perm in itertools.permutations(range(4)):
        target = robot_holes[list(perm)]
        a, b = holes - holes.mean(0), target - target.mean(0)
        u, _, vt = np.linalg.svd(a.T @ b)
        R = vt.T @ u.T
        angle = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        # New source X/Y/Z corresponds to old -Z/Y/X. Consequently the
        # old confirmed +90 flange branch maps source X/Y to flange Y/Z.
        if np.linalg.det(R) > 0 and abs(angle) < 30:
            errors = np.linalg.norm(a @ R.T - b, axis=1)
            candidates.append((float(np.mean(errors**2)), R, target, errors, float(angle)))
    if not candidates:
        raise ValueError('No orientation-preserving mounting branch')
    _, R, target, errors, angle = min(candidates, key=lambda x: x[0])
    T = np.eye(4)
    T[:3, :3] = [[0, 0, 1], [R[0, 0], R[0, 1], 0], [R[1, 0], R[1, 1], 0]]
    T[:3, 3] = np.r_[face, target.mean(0) - R @ holes.mean(0)] * .001
    evidence = dict(status='DRAFT', method='planar boundary circles + four-hole rigid registration',
                    source_sha256=result['sha256'], source_contact_plane_axis='Z', source_contact_plane_mm=0,
                    source_plate_center_mm=[*plate['center'].tolist(), 0], source_plate_diameter_mm=63,
                    source_holes_xy_mm=holes.tolist(), robot_holes_flange_yz_mm=target.tolist(),
                    per_hole_error_mm=errors.tolist(), hole_center_rms_mm=float(np.sqrt(np.mean(errors**2))),
                    hole_center_max_mm=float(errors.max()), in_plane_rotation_deg=angle,
                    T_flange_source=T.tolist(), contact_gap_mm=0, normal_opposition_dot=-1,
                    orientation_basis='Previous +90 branch; source X/Y/Z = old -Z/Y/X',
                    note='Mesh fit only; CAD and robot hole pitches differ. Physical confirmation required.', robot_motion='NO')
    save(folder / 'mount_hole_alignment.json', evidence)
    # Generate the separate metre-unit visual only; never recentre/rotate STL.
    reader = vtk.vtkSTLReader(); reader.SetFileName(str(stl)); reader.Update()
    tr = vtk.vtkTransform(); tr.Scale(.001, .001, .001)
    filt = vtk.vtkTransformPolyDataFilter(); filt.SetInputConnection(reader.GetOutputPort()); filt.SetTransform(tr)
    writer = vtk.vtkSTLWriter(); writer.SetFileTypeToBinary(); writer.SetInputConnection(filt.GetOutputPort())
    writer.SetFileName(str(folder / 'welding_gun_visual_m.stl'))
    if not writer.Write():
        raise ValueError('Cannot write metre-unit visual')
    manifest = dict(schema_version='real_gun_geometry_v1', created_utc=datetime.now(timezone.utc).isoformat(),
                    version='Areal-' + result['sha256'][:16], status='STAGED_ALIGNMENT_REQUIRED',
                    activation_policy='New source requires manual mounting/collision confirmation; no automatic activation.',
                    source=dict(stl=str(stl), sha256=result['sha256'], units='mm', scale_to_m=.001,
                                source_frame='STL_SOURCE', audit_file=str(folder / 'stl_audit.json')),
                    visual=dict(path=str(stl), units='mm', source_frame='STL_SOURCE',
                                T_flange_source=T.tolist(), transform_status='DRAFT'),
                    preview=dict(T_flange_source=T.tolist()),
                    collision=dict(status='PENDING_VHACD', source_frame='STL_SOURCE', components=[]),
                    mounting=dict(status='DRAFT', coordinate_frame='URDF flange', T_flange_source=T.tolist(),
                                  confirmed_by_operator=False, evidence_file=str(folder / 'mount_hole_alignment.json')),
                    wire=dict(status='DISABLED_UNVERIFIED', reason='Old model reference cannot be copied to a changed CAD source.'),
                    requires_replan=True, backup_directory=str(backup), original_source=str(source))
    save(folder / 'real_gun_geometry.json', manifest)
    print(json.dumps(dict(folder=str(folder), backup=str(backup), triangles=result['triangle_count'],
                          hole_rms_mm=evidence['hole_center_rms_mm'], hole_max_mm=evidence['hole_center_max_mm']), ensure_ascii=False))


if __name__ == '__main__':
    main()
