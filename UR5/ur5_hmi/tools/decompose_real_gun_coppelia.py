#!/usr/bin/env python3
"""Offline V-HACD via a dedicated stopped CoppeliaSim instance (no ROS).

Start CoppeliaSim with separate RPC/WS ports and NO -s flag. Source coordinates
are retained; only the explicit mm->m conversion is used. This tool never
confirms a flange mounting transform or activates planning geometry.
"""
import argparse
import hashlib
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, '/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from export_scene_geometry import mesh_record
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=23060)
    p.add_argument('--folder', type=Path, default=ROOT / 'real_gun_geometry',
                   help='Independent geometry package; never overwrite a frozen scene')
    p.add_argument('--resolution', type=int, default=1000000)
    p.add_argument('--concavity', type=float, default=0.0025)
    p.add_argument('--recover-completed', action='store_true', help='Export completed convex parts after a client timeout; do not rerun decomposition')
    p.add_argument('--collision-input', type=Path, help='Explicit audited collision-only STL derivative in mm')
    p.add_argument('--lightweight-scene', action='store_true', help='Use collision derivative as Coppelia preview visual; HMI still renders the full source STL')
    args = p.parse_args()
    folder = args.folder.resolve()
    source = folder / 'welding_gun_source.STL'
    client = RemoteAPIClient(port=args.port)
    sim = client.require('sim')
    convex = client.require('simConvex')
    client.socket.RCVTIMEO = 3600000
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Dedicated instance must be STOPPED; refusing simulation changes')
    existing = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0)
    allowed = ('Floor', 'box', 'welding_gun_source', 'WeldingGun_visual')
    if not args.recover_completed and any(sim.getObjectAlias(h) not in allowed for h in existing):
        raise RuntimeError('Expected empty dedicated scene; refusing to replace existing shapes')
    # ignore any file up-vector, no bounding-box alignment option. Relocating
    # only the reference frame keeps imported mesh world coordinates intact.
    pending = [h for h in existing if sim.getObjectAlias(h) in ('welding_gun_source', 'WeldingGun_visual')]
    scene_source=args.collision_input.resolve() if args.lightweight_scene and args.collision_input else source
    print(f'Importing Coppelia preview mesh: {scene_source}',flush=True)
    visual = pending[0] if pending else sim.importShape(0, str(scene_source), 128, 0, 0.001)
    print('Import done, preserving source frame',flush=True)
    sim.relocateShapeFrame(visual, [0, 0, 0, 0, 0, 0, 1])
    sim.setObjectAlias(visual, 'WeldingGun_visual')
    print(f'Imported source shape {visual}', flush=True)
    print('Reading source bounds',flush=True)
    # Do not transfer millions of visual mesh coordinates through Lua/CBOR.
    # The box pose is relative to the identity source shape frame; inspecting
    # its eight corners does not move, rotate or rescale the source mesh.
    from itertools import product
    from scipy.spatial.transform import Rotation
    # This changes bounding-box metadata ONLY, not shape frame or vertices.
    sim.alignShapeBB(visual,[0,0,0,0,0,0,1])
    size, pose = sim.getShapeBB(visual)
    corners = np.asarray(list(product((-0.5,0.5),repeat=3)))*np.asarray(size)
    points = corners@Rotation.from_quat(pose[3:7]).as_matrix().T+np.asarray(pose[:3])
    audit = json.loads((folder / 'stl_audit.json').read_text())
    # Verify source origin was not silently recentred/reoriented.
    bounds = np.stack([points.min(0), points.max(0)])
    expected = np.asarray([audit['raw_coordinate_bounds']['min'], audit['raw_coordinate_bounds']['max']])*0.001
    tolerance=0.001 if args.lightweight_scene else 1e-7
    if not np.allclose(bounds, expected, atol=tolerance):
        raise RuntimeError(f'Import changed source bounds: {bounds} != {expected}')
    params = dict(resolution=args.resolution, concavity=args.concavity,
                  plane_downsampling=4, hull_downsampling=4, alpha=0.05,
                  beta=0.05, max_vertices=64, min_volume=0.00001,
                  pca=False, voxels=True)
    decomposition_source = visual
    if args.collision_input and not args.lightweight_scene:
        print('Importing collision-only input',flush=True)
        decomposition_source=sim.importShape(0,str(args.collision_input.resolve()),128,0,0.001)
        sim.relocateShapeFrame(decomposition_source,[0,0,0,0,0,0,1])
        sim.setObjectAlias(decomposition_source,'V_HACD_COLLISION_INPUT_DERIVATIVE')
    print(f'Calling actual V-HACD with {params}',flush=True)
    start = time.perf_counter()
    if args.recover_completed:
        parts = [h for h in existing if sim.getObjectAlias(h) not in allowed
                 and int(sim.getShapeGeomInfo(h)[0]) & 4]
        elapsed = None # no fabricated duration after client timeout
    else:
        parts = convex.vhacd(decomposition_source, params)
        elapsed = time.perf_counter() - start
    if not parts:
        raise RuntimeError('V-HACD returned no convex shapes')
    print(f'V-HACD returned {len(parts)} components',flush=True)
    identity = [0, 0, 0, 0, 0, 0, 1]
    part_vertices = []
    for i, part in enumerate(parts, 1):
        if not int(sim.getShapeGeomInfo(part)[0]) & 4:
            raise RuntimeError(f'V-HACD part {i} is not convex')
        sim.relocateShapeFrame(part, identity)
        v, _, _ = sim.getShapeMesh(part)
        part_vertices.append(np.asarray(v).reshape(-1, 3))
    hull = sim.groupShapes(parts) if len(parts) > 1 else parts[0]
    sim.relocateShapeFrame(hull, identity)
    sim.setObjectAlias(hull, 'GunCollisionHull')
    anchor = sim.createDummy(0.01)
    sim.setObjectAlias(anchor, 'STL_SOURCE_UNCONFIRMED')
    sim.setObjectParent(visual, anchor, True)
    sim.setObjectParent(hull, anchor, True)
    record = mesh_record(sim, hull, anchor, anchor)
    if record['component_count'] != len(parts):
        raise RuntimeError('Compound exporter lost convex components')
    # Match property-exported vertices against pre-group source-frame vertices.
    before = np.concatenate(part_vertices)
    after = np.concatenate([np.asarray(c['vertices_m']) for c in record['components']])
    from scipy.spatial import cKDTree
    component_frame_error = max(cKDTree(before).query(after)[0].max(),
                                cKDTree(after).query(before)[0].max())
    if component_frame_error > 1e-7:
        raise RuntimeError(f'Component frame mismatch: {component_frame_error} m')
    for i, c in enumerate(record['components'], 1):
        c.update(name=f'GunCollisionHull_{i:02d}', type='convex_mesh', source_frame='STL_SOURCE')
    record.update(method='CoppeliaSim simConvex.vhacd', parameters=params,
                  elapsed_seconds=elapsed, source_frame='STL_SOURCE',
                  source_bounds_m=bounds.tolist(),
                  component_frame_error_m=float(component_frame_error),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  collision_input=str(args.collision_input.resolve()) if args.collision_input else str(source),
                  coppelia_preview_visual='collision input derivative' if args.lightweight_scene else 'full source STL',
                  coppeliasim_version='4.10.0 rev0', generated_utc=datetime.now(timezone.utc).isoformat())
    (folder / 'vhacd_collision_export.json').write_text(json.dumps(record, indent=2)+'\n')
    manifest = json.loads((folder / 'real_gun_geometry.json').read_text())
    manifest['collision'].update(status='GENERATED_ALIGNMENT_REQUIRED',
        method=record['method'], parameters=params, components=record['components'],
        component_count=len(parts), decomposition_time_sec=elapsed,
        export_file=str(folder / 'vhacd_collision_export.json'))
    # Preserve a model-defined wire record when the source STL has not
    # changed.  Re-decomposition must not silently erase an operator-approved
    # CAD tip/axis definition; a changed source is explicitly downgraded.
    wire = manifest.get('wire', {})
    if (str(wire.get('status', '')).upper() in {'MODEL_DEFINED_ACTIVE', 'MODEL_AXIS_TO_TCP_ACTIVE'}
            and manifest.get('source', {}).get('sha256') == record['source_sha256']):
        manifest['wire'] = wire
    else:
        manifest['wire'].update(status='DISABLED_UNVERIFIED',
            reason='Wire geometry requires a matching active model-defined CAD reference record.')
    manifest['status'] = 'STAGED_ALIGNMENT_REQUIRED'
    (folder / 'real_gun_geometry.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    scene = folder / 'new_gun_vhacd_source_preview.ttt'
    sim.saveScene(str(scene))
    print(json.dumps(dict(component_count=len(parts), vertices=record['vertex_count'],
        faces=record['face_count'], decomposition_seconds=elapsed,
        component_frame_error_m=float(component_frame_error), scene=str(scene))), flush=True)


if __name__ == '__main__':
    main()
