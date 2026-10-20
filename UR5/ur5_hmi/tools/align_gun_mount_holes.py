"""Offline planar flange boundary inspection; never connects to ROS."""
from pathlib import Path
import sys
import json
import itertools
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ur5_hmi'))
from gui.robot_view import UrdfKinematicModel


def mesh(path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path)); reader.Update()
    p = reader.GetOutput()
    return (vtk_to_numpy(p.GetPoints().GetData()).astype(float),
            vtk_to_numpy(p.GetPolys().GetData()).reshape(-1, 4)[:, 1:])


def boundaries(v, f, axis, plane, tol):
    tri = v[f]
    selected = f[np.max(np.abs(tri[:, :, axis] - plane), axis=1) < tol]
    edges = np.sort(np.vstack([selected[:, [0, 1]], selected[:, [1, 2]], selected[:, [2, 0]]]), axis=1)
    edges, counts = np.unique(edges, axis=0, return_counts=True)
    edges = edges[counts == 1]
    neighbors = {}
    for a, b in edges:
        neighbors.setdefault(int(a), []).append(int(b))
        neighbors.setdefault(int(b), []).append(int(a))
    remaining = set(neighbors)
    components = []
    while remaining:
        stack = [remaining.pop()]; ids = []
        while stack:
            i = stack.pop(); ids.append(i)
            for j in neighbors[i]:
                if j in remaining:
                    remaining.remove(j); stack.append(j)
        points = v[ids]
        uv = np.delete(points, axis, axis=1)
        A = np.column_stack([2 * uv, np.ones(len(uv))])
        fit = np.linalg.lstsq(A, np.sum(uv**2, axis=1), rcond=None)[0]
        center = fit[:2]; radius = np.sqrt(max(0, fit[2] + center @ center))
        residual = np.linalg.norm(uv - center, axis=1) - radius
        components.append({'points': points, 'center': center, 'radius': float(radius),
                           'rms': float(np.sqrt(np.mean(residual**2))), 'count': len(points)})
    return sorted(components, key=lambda x: -x['radius'])


def main():
    folder = ROOT / 'real_gun_geometry'
    gun_v, gun_f = mesh(folder / 'welding_gun_source.STL')
    model = UrdfKinematicModel()
    frames = model.link_transforms_in_base(np.zeros(6))
    wrist_v, wrist_f = mesh('/opt/ros/humble/share/ur_description/meshes/ur5/collision/wrist3.stl')
    T = np.linalg.inv(frames['flange']) @ frames['wrist_3_link'] @ model.link_meshes['wrist_3_link']['origin']
    wrist_v = (wrist_v @ T[:3, :3].T + T[:3, 3]) * 1000
    print('wrist flange bounds', wrist_v.min(0), wrist_v.max(0))
    planes = [('gun back', gun_v, gun_f, 49.23116, .001),
              ('gun front', gun_v, gun_f, 59.23116, .001),
              ('robot face', wrist_v, wrist_f, float(wrist_v[:, 0].max()), .01)]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    report = {}
    loops_by_name = {}
    for ax, (name, v, f, plane, tol) in zip(axes, planes):
        loops = boundaries(v, f, 0, plane, tol)
        loops_by_name[name] = loops
        report[name] = []
        for i, loop in enumerate(loops):
            p = loop['points']
            ax.scatter(p[:, 1], p[:, 2], s=2)
            ax.text(*loop['center'], str(i))
            row = {k: (x.tolist() if isinstance(x, np.ndarray) else x) for k, x in loop.items() if k != 'points'}
            report[name].append(row)
        ax.set_aspect('equal'); ax.set_title(f'{name}: X={plane:.4f} mm'); ax.set_xlabel('Y mm'); ax.set_ylabel('Z mm')
    fig.tight_layout(); fig.savefig(folder / 'mount_hole_inspection.png', dpi=160)
    (folder / 'mount_hole_inspection.json').write_text(json.dumps(report, indent=2) + '\n')
    back = loops_by_name['gun back']
    center = back[0]['center']
    # Four bolt holes: radius 3 mm, at the square corners on PCD 50 mm.
    # The two additional axial holes are not treated as robot bolt holes.
    gun_holes = np.array([x['center'] for x in back if abs(x['radius'] - 3) < .01
                          and abs(abs(x['center'][0] - center[0]) - 25 / np.sqrt(2)) < .1])
    robot_holes = np.array([x['center'] for x in loops_by_name['robot face'] if 2 < x['radius'] < 3])
    if gun_holes.shape != (4, 2) or robot_holes.shape != (4, 2):
        raise RuntimeError('Four corresponding mounting holes were not identified')
    candidates = []
    for permutation in itertools.permutations(range(4)):
        target = robot_holes[list(permutation)]
        a, b = gun_holes - gun_holes.mean(0), target - target.mean(0)
        u, _, vt = np.linalg.svd(a.T @ b)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            continue
        angle = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
        errors = np.linalg.norm(a @ rotation.T - b, axis=1)
        # The four-hole pattern has 90-degree symmetry. Select the branch
        # nearest the user-confirmed +90-degree installation, not the TCP.
        if abs(angle - 90) < 30:
            candidates.append((float(np.mean(errors**2)), angle, rotation, target, errors))
    _, angle, rotation, target, errors = min(candidates, key=lambda x: x[0])
    total = np.eye(4)
    total[1:3, 1:3] = rotation
    total[0, 3] = wrist_v[:, 0].max() - 49.23116
    total[1:3, 3] = target.mean(0) - rotation @ gun_holes.mean(0)
    total[:3, 3] *= .001
    adjust = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
    nominal = np.linalg.inv(adjust) @ total
    evidence = {
        'method': 'planar boundary circle fits + four-hole rigid registration',
        'gun_contact_plane_source_mm': 49.23116,
        'gun_plate_center_source_mm': [49.23116, *center.tolist()],
        'gun_plate_diameter_mm': back[0]['radius'] * 2,
        'gun_bolt_hole_diameter_mm': 6,
        'gun_bolt_pcd_mm': 50,
        'robot_contact_plane_flange_mm': float(wrist_v[:, 0].max()),
        'gun_holes_source_yz_mm': gun_holes.tolist(),
        'robot_holes_flange_yz_mm': target.tolist(),
        'per_hole_center_error_mm': errors.tolist(),
        'hole_center_rms_mm': float(np.sqrt(np.mean(errors**2))),
        'hole_center_max_mm': float(errors.max()),
        'effective_roll_deg': angle,
        'T_flange_source': total.tolist(),
        'T_flange_source_nominal': nominal.tolist(),
        'contact_plane_gap_mm': 0,
        'normal_opposition_dot': -1,
        'azimuth_ambiguity': 'fourfold bolt symmetry; +90 branch from user field confirmation',
        'robot_mesh_limit': 'low polygon wrist3 collision mesh; fit errors are mesh errors, not physical measurements',
        'status': 'DRAFT', 'robot_motion': 'NO',
    }
    (folder / 'mount_hole_alignment.json').write_text(json.dumps(evidence, indent=2) + '\n')
    fig2, ax = plt.subplots(figsize=(6, 6))
    moved = gun_holes @ rotation.T + total[1:3, 3] * 1000
    ax.scatter(target[:, 0], target[:, 1], s=150, facecolors='none', edgecolors='blue', label='UR5 hole centres')
    ax.scatter(moved[:, 0], moved[:, 1], s=35, c='orange', marker='+', label='Gun holes after alignment')
    for i, (point, error) in enumerate(zip(target, errors)):
        ax.annotate(f'{i+1}: {error:.3f} mm', point, xytext=(5, 8), textcoords='offset points')
    ax.set_aspect('equal'); ax.set_xlabel('Flange Y (mm)'); ax.set_ylabel('Flange Z (mm)')
    ax.set_title('Four mounting holes aligned (mesh inspection)'); ax.legend(); ax.grid(alpha=.2)
    fig2.tight_layout(); fig2.savefig(folder / 'mount_holes_aligned.png', dpi=160)
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main()
