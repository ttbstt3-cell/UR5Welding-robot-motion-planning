#!/usr/bin/env python3
"""Read existing fixed CoppeliaSim environment geometry without scene mutations.

The frozen tool/workpiece export and scene files are never written.  Meshes are
kept per original component: a nonconvex fence is never replaced by its overall
convex hull or bounding box.  All exported poses use Coppelia UR5_base and world;
the planning/HMI loaders perform their existing calibrated frame conversion.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

sys.path.insert(0, '/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

ROOT = Path(__file__).resolve().parent


def pose(sim, handle, reference, frame):
    p = sim.getObjectPose(handle, reference)
    return dict(frame=frame, position=list(map(float, p[:3])),
                quaternion=list(map(float, p[3:7])))


def matrix(sim, handle, reference):
    return np.vstack((np.asarray(sim.getObjectMatrix(handle, reference), float)
                      .reshape(3, 4), [0., 0., 0., 1.])).tolist()


def category(path):
    text = path.lower()
    if any(k in text for k in ('floor', 'ground', '地面')):
        return 'floor'
    if any(k in text for k in ('fence', '栅栏', '围栏')):
        return 'safety_fence'
    if any(k in text for k in ('table', 'bench', 'desk', '工作台')):
        return 'workbench'
    if any(k in text for k in ('fixture', 'clamp', '夹具')):
        return 'fixture'
    return 'surrounding_obstacle'


def get_optional(fn, fallback):
    try:
        return fn()
    except Exception:
        return fallback


def record(sim, handle, base):
    path = sim.getObjectAlias(handle, 2)
    alias = sim.getObjectAlias(handle)
    flags, primitive, params = sim.getShapeGeomInfo(handle)
    special = int(sim.getObjectSpecialProperty(handle))
    type_names = {int(sim.primitiveshape_cuboid): 'box',
                  int(sim.primitiveshape_cylinder): 'cylinder',
                  int(sim.primitiveshape_spheroid): 'spheroid',
                  int(sim.primitiveshape_capsule): 'capsule',
                  int(sim.primitiveshape_cone): 'cone',
                  int(sim.primitiveshape_disc): 'disc',
                  int(sim.primitiveshape_plane): 'plane'}
    size = list(map(float, params[:3]))
    compound = bool(int(flags) & 1)
    convex = bool(int(flags) & 4)
    shape_type = type_names.get(int(primitive), 'mesh') if not compound else 'compound'
    bbox_size, bbox_pose = sim.getShapeBB(handle)
    rec = dict(name='EnvironmentCollision_' + re.sub(r'\W+', '_', alias),
               source_alias=alias, path=path, category=category(path),
               type=shape_type, size_m=size, is_convex=convex,
               primitive_type_code=int(primitive), geometry_flags=int(flags),
               special_property=special,
               collidable_source=bool(special & int(sim.objectspecialproperty_collidable)),
               collision_enabled=True,
               static=bool(sim.getObjectInt32Param(handle, sim.shapeintparam_static)),
               visibility_layer=int(sim.getObjectInt32Param(handle, sim.objintparam_visibility_layer)),
               pose_base=pose(sim, handle, base, 'UR5_base'),
               pose_world=pose(sim, handle, -1, 'world'),
               T_UR5base_object=matrix(sim, handle, base),
               bbox_size_m=list(map(float, bbox_size)),
               bbox_pose_object=list(map(float, bbox_pose)),
               faces_index_base=1)
    vertices, indices, _ = sim.getShapeMesh(handle)
    rec.update(vertices_m=np.asarray(vertices, float).reshape(-1, 3).tolist(),
               faces=(np.asarray(indices, int).reshape(-1, 3)+1).tolist())
    if shape_type in ('cylinder', 'capsule'):
        rec.update(radius_m=size[0]/2, length_m=size[2])
    components = []
    for n, mesh in enumerate(get_optional(lambda: sim.getIntArrayProperty(handle, 'meshes'), []), 1):
        components.append(dict(index=n, name=f'{rec["name"]}_component_{n:03d}',
            type='mesh', is_convex=bool(sim.getBoolProperty(mesh, 'convex')),
            vertices_m=np.asarray(sim.getFloatArrayProperty(mesh, 'vertices'), float).reshape(-1, 3).tolist(),
            faces=(np.asarray(sim.getIntArrayProperty(mesh, 'indices'), int).reshape(-1, 3)+1).tolist(),
            faces_index_base=1,
            pose_object=dict(frame=alias, position=[0., 0., 0.], quaternion=[0., 0., 0., 1.])))
    rec['components'] = components
    # Property vertices follow the shape-local coordinates.  Check rather than
    # relying on an undocumented component pose or modifying source frames.
    if components:
        from scipy.spatial import cKDTree
        a=np.asarray(rec['vertices_m']); b=np.concatenate([np.asarray(c['vertices_m']) for c in components])
        rec['component_frame_error_m']=float(max(cKDTree(a).query(b)[0].max(), cKDTree(b).query(a)[0].max()))
        if rec['component_frame_error_m'] > 1e-7:
            raise RuntimeError(f'Component coordinates do not match shape frame: {path}')
    return rec


def export_scene(output, host='localhost', port=23000, source_scene=None, sim_backend=None):
    if sim_backend is None:
        client=RemoteAPIClient(host=host, port=port)
        client.socket.RCVTIMEO=15000
        sim=client.require('sim')
    else:
        sim=sim_backend
    before_state=int(sim.getSimulationState())
    if before_state != int(sim.simulation_stopped):
        raise RuntimeError('Environment snapshot requires stopped scene; no stop command is sent.')
    base=sim.getObject('/UR5')
    scene_path=sim.getStringParam(sim.stringparam_scene_path_and_name)
    inventory=[]; objects=[]; warnings=[]
    shape_handles=sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0)
    for handle in shape_handles:
        path=sim.getObjectAlias(handle, 2)
        if path.startswith('/UR5/') or path == '/UR5':
            reason='Existing robot/tool collision model is retained separately'
        elif path.startswith('/Workpiece'):
            reason='Existing calibrated workpiece collision model is retained separately'
        else:
            reason=''
        if reason:
            inventory.append(dict(path=path, source_alias=sim.getObjectAlias(handle), included=False, reason=reason))
            continue
        rec=record(sim, handle, base)
        inventory.append(dict(path=path, source_alias=rec['source_alias'], included=True,
                              category=rec['category'], type=rec['type'], size_m=rec['size_m'],
                              static=rec['static'], collidable_source=rec['collidable_source'],
                              is_convex=rec['is_convex'], component_count=len(rec['components'])))
        if not rec['static']:
            warnings.append(f'Dynamic source exported at stopped pose: {path}; no motion prediction.')
        if not rec['collidable_source']:
            rec['collision_policy_reason']='Physical environment geometry included despite disabled Coppelia collidable flag'
        if rec['type'] in ('mesh', 'compound') and not rec['is_convex']:
            warnings.append(f'Nonconvex geometry requires exact component loading, not overall collisionMesh convexification: {path}')
        rec['pose_UR5_base']=rec.pop('pose_base')
        geometry_keys=('type','size_m','radius_m','length_m','vertices_m','faces',
                       'is_convex','components','faces_index_base','component_frame_error_m')
        rec['geometry']={key:rec.pop(key) for key in geometry_keys if key in rec}
        objects.append(rec)
    source=Path(source_scene or scene_path)
    data=dict(schema_version=1, units='m', frame_id='UR5_base',
              faces_index_base=1, status='EXPORTED_REVIEW_REQUIRED',
              generated_utc=datetime.now(timezone.utc).isoformat(),
              source=dict(scene=str(source), scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest()
                          if source.is_file() else None,
                          active_scene_path=scene_path, remote_api=dict(host=host, port=port),
                          simulation_state=before_state,
                          method='CoppeliaSim existing primitives/meshes; read-only; no convexification'),
              frames=dict(base_path='/UR5', base_pose_world=pose(sim, base, -1, 'world'),
                          T_world_UR5base=matrix(sim, base, -1)),
              objects=objects, object_count=len(objects), inventory=inventory,
              inventory_complete=True, source_shape_count=len(shape_handles),
              warnings=warnings,
              scene_coverage=dict(floor=any(o['category']=='floor' for o in objects),
                    safety_fence=any(o['category']=='safety_fence' for o in objects),
                    workbench=any(o['category']=='workbench' for o in objects),
                    fixture=any(o['category']=='fixture' for o in objects)))
    if int(sim.getSimulationState()) != before_state:
        raise RuntimeError('Simulation state changed while reading geometry; snapshot rejected')
    return finalize_manifest(data, output)


def finalize_manifest(data, output):
    """Add calibrated registration and prove redundant floor decoration exclusion."""
    from scipy.io import loadmat
    objects=data['objects']
    physical_floor=next((o for o in objects if o['path']=='/Floor'
                         and o['geometry']['type']=='box'), None)
    if physical_floor:
        from scipy.spatial.transform import Rotation
        fp=physical_floor['pose_UR5_base']; fg=physical_floor['geometry']
        floor_r=Rotation.from_quat(fp['quaternion']).as_matrix()
        half=np.asarray(fg['size_m'])/2
        for o in objects:
            if o['path'] != '/Floor/box' or o['collidable_source']:
                continue
            # This specific source child is a duplicate slab plus a planar
            # checkered top.  Verify the actual vertices lie in the native box.
            p=o['pose_UR5_base']; r=Rotation.from_quat(p['quaternion']).as_matrix()
            v=np.asarray(o['geometry']['vertices_m'])
            local=(v@r.T+np.asarray(p['position'])-np.asarray(fp['position']))@floor_r
            excess=float(np.max(np.abs(local)-half))
            if excess <= 1e-7:
                o['collision_enabled']=False
                o['collision_policy_reason']='Non-collidable child duplicates native /Floor slab and planar checkered top; no independent physical obstacle'
                o['duplicate_floor_max_bound_excess_m']=excess
                data['warnings']=[w for w in data['warnings'] if o['path'] not in w]
                for item in data['inventory']:
                    if item['path']==o['path']:
                        item.update(collision_enabled=False, reason=o['collision_policy_reason'])
    data['collision_object_count']=sum(bool(o['collision_enabled']) for o in objects)
    data['missing_categories']=[k for k,v in data['scene_coverage'].items() if not v]
    data['environment_status']='SOURCE_SCENE_PARTIAL_COVERAGE'
    data['coverage_note']='Floor exported; safety fence/workbench/fixtures absent from the source scene. This is not complete registration of the real workplace.'
    calibration=ROOT/'ur5_tool_calibration.mat'
    urdf=ROOT/'real_robot_description_calibrated.urdf'
    if not urdf.exists():
        urdf=ROOT/'ur5_hmi'/'config'/'real_robot_description_calibrated.urdf'
    # Derive the actual root-to-base fixed chain; no visual alignment offsets.
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    tree=ET.parse(urdf).getroot()
    parents={j.find('child').get('link'):j for j in tree.findall('joint')}
    transform=np.eye(4); link='base'
    while link in parents:
        joint=parents[link]
        if joint.get('type')!='fixed':
            raise RuntimeError('URDF root-to-base chain is not fixed')
        origin=joint.find('origin'); local=np.eye(4)
        if origin is not None:
            local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        transform=local@transform; link=joint.find('parent').get('link')
    cop_mat=np.asarray(loadmat(calibration)['T_CopBase_MatBase'],float)
    data['frames']['T_base_UR5base']=(np.linalg.inv(transform)@np.linalg.inv(cop_mat)).tolist()
    data['frames']['registration_source']=dict(calibration_path=str(calibration),
        calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
        urdf_path=str(urdf),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        urdf_root=link,T_URDFroot_base=transform.tolist(),T_CopBase_MatBase=cop_mat.tolist(),
        derivation='inv(T_URDFroot_base) @ inv(T_CopBase_MatBase)')
    data['version']='environment_'+hashlib.sha256(json.dumps(dict(objects=objects,frames=data['frames']),
                                       sort_keys=True).encode()).hexdigest()[:16]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    return data


def export_recorded_inventory(raw_path, output, shape_inventory_path=None):
    """Convert actual read-only API snapshot, retaining original source coordinates."""
    raw=json.loads(raw_path.read_text())
    if raw['simulation_state'] != 0:
        raise RuntimeError('Recorded source scene was not stopped')
    objects=[]; inventory=[]; warnings=[]
    for s in raw['shapes']:
        if s.get('excluded'):
            inventory.append(dict(path=s['path'],included=False,reason='Existing robot/tool/workpiece model retained separately'))
            continue
        flags,primitive,size=s['geom_info']
        # The stored source dump retains actual API flags/type codes. Cuboid is
        # the only primitive in this snapshot; unknown codes remain mesh.
        kind='compound' if int(flags)&1 else ('box' if int(primitive)==3 else 'mesh')
        alias=s['alias']; name='EnvironmentCollision_'+re.sub(r'\W+','_',alias)
        vertices,indices,_=s['vertices_indices']
        components=[]
        for n,c in enumerate(s['components'],1):
            components.append(dict(index=n,name=f'{name}_component_{n:03d}',type='mesh',
                is_convex=bool(c['convex']),vertices_m=np.asarray(c['vertices'],float).reshape(-1,3).tolist(),
                faces=(np.asarray(c['indices'],int).reshape(-1,3)+1).tolist(),faces_index_base=1,
                pose_object=dict(frame=alias,position=[0.,0.,0.],quaternion=[0.,0.,0.,1.])))
        def p(record,frame):
            return dict(frame=frame,position=list(map(float,record[:3])),quaternion=list(map(float,record[3:7])))
        o=dict(name=name,path=s['path'],source_alias=alias,category=category(s['path']),collision_enabled=True,
            collidable_source=bool(s['special_property']&1),special_property=s['special_property'],static=bool(s['static']),
            primitive_type_code=primitive,geometry_flags=flags,pose_UR5_base=p(s['pose_base'],'UR5_base'),
            pose_world=p(s['pose_world'],'world'),T_UR5base_object=np.vstack((np.asarray(s['matrix_base']).reshape(3,4),[0,0,0,1])).tolist(),
            bbox_size_m=s['bbox'][0],bbox_pose_object=s['bbox'][1],
            geometry=dict(type=kind,size_m=list(map(float,size[:3])),is_convex=bool(flags&4),
                vertices_m=np.asarray(vertices,float).reshape(-1,3).tolist(),
                faces=(np.asarray(indices,int).reshape(-1,3)+1).tolist(),faces_index_base=1,components=components))
        if not o['collidable_source']:
            o['collision_policy_reason']='Physical environment geometry included despite disabled Coppelia collidable flag'
        if kind in ('mesh','compound') and not o['geometry']['is_convex']:
            warnings.append(f'Nonconvex geometry requires exact component loading, not overall collisionMesh convexification: {s["path"]}')
        objects.append(o)
        inventory.append(dict(path=s['path'],source_alias=alias,included=True,type=kind,category=o['category'],
            static=o['static'],collidable_source=o['collidable_source'],collision_enabled=True,
            size_m=o['geometry']['size_m'],component_count=len(components)))
    if shape_inventory_path:
        all_shapes=json.loads(shape_inventory_path.read_text())
        if sorted(s['handle'] for s in all_shapes) != sorted(raw['all_shape_handles']):
            raise RuntimeError('Full shape inventory differs from raw snapshot shape handles')
        names={s['path'] for s in raw['shapes']}
        for s in all_shapes:
            if s['path'] in names:
                continue
            if not s.get('excluded'):
                raise RuntimeError('Environment shape was not exported: '+s['path'])
            inventory.append(dict(path=s['path'],source_alias=s['alias'],included=False,
                reason='Existing calibrated workpiece collision model is retained separately'
                       if s['path'].startswith('/Workpiece') else 'Existing robot/tool collision model is retained separately'))
    source=Path(raw['scene_path'])
    data=dict(schema_version=1,units='m',frame_id='UR5_base',faces_index_base=1,status='EXPORTED_REVIEW_REQUIRED',
        generated_utc=datetime.now(timezone.utc).isoformat(),
        source=dict(scene=str(source),scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            active_scene_path=str(source),simulation_state=0,remote_api=dict(host='localhost',port=23070),
            raw_inventory_path=str(raw_path.resolve()),raw_inventory_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            method='CoppeliaSim read-only API snapshot; no convexification'),
        frames=dict(base_path='/UR5',base_pose_world=p(raw['base_pose_world'],'world'),
            T_world_UR5base=np.vstack((np.asarray(raw['T_world_UR5base']).reshape(3,4),[0,0,0,1])).tolist()),
        objects=objects,object_count=len(objects),inventory=inventory,warnings=warnings,
        inventory_complete=bool(shape_inventory_path),
        source_shape_count=len(raw.get('all_shape_handles',[])),
        scene_coverage={k:any(o['category']==k for o in objects) for k in ('floor','safety_fence','workbench','fixture')})
    return finalize_manifest(data,output)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=23000)
    p.add_argument('--source-scene', type=Path)
    p.add_argument('--from-inventory', type=Path, help='Offline normalize a recorded read-only Coppelia API snapshot')
    p.add_argument('--shape-inventory', type=Path, help='Optional full object-name inventory from the same stopped source scene')
    p.add_argument('--output', type=Path, default=ROOT/'environment_geometry_real.json')
    a=p.parse_args()
    data=export_recorded_inventory(a.from_inventory,a.output,a.shape_inventory) if a.from_inventory else export_scene(a.output,a.host,a.port,a.source_scene)
    print(json.dumps(dict(output=str(a.output), object_count=data['object_count'],
                          coverage=data['scene_coverage'], inventory=data['inventory'], warnings=data['warnings']),
                     ensure_ascii=False, indent=2), flush=True)


if __name__=='__main__':
    main()
