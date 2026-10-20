#!/usr/bin/env python3
"""Install a backed-up V-HACD package for HMI preview, NOT activation."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from prepare_replacement_gun import ROOT, save, UrdfKinematicModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    folder=args.folder.resolve()
    manifest=json.loads((folder/'real_gun_geometry.json').read_text())
    backup=Path(manifest['backup_directory'])
    target=ROOT/'real_gun_geometry'
    if not (backup/'real_gun_geometry'/'real_gun_geometry.json').is_file():
        raise ValueError('Rollback package missing')
    source=folder/'welding_gun_source.STL'
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    export=json.loads((folder/'vhacd_collision_export.json').read_text())
    if source_hash!=manifest['source']['sha256'] or source_hash!=export['source_sha256']:
        raise ValueError('Source and decomposition hashes disagree')
    if export['method']!='CoppeliaSim simConvex.vhacd' or not export['components']:
        raise ValueError('Actual V-HACD required')
    evidence=json.loads((folder/'mount_hole_alignment.json').read_text())
    T=np.asarray(evidence['T_flange_source'],dtype=float)
    model=UrdfKinematicModel(ROOT/'real_robot_description_calibrated.urdf')
    frames=model.link_transforms_in_base(np.zeros(6))
    Tf0=np.linalg.inv(frames['flange'])@frames['tool0']
    Tfm=Tf0@loadmat(ROOT/'ur5_tool_calibration.mat')['T_tool0_mount']
    manifest['source'].update(stl=str(target/'welding_gun_source.STL'),audit_file=str(target/'stl_audit.json'))
    manifest['visual'].update(path=str(target/'welding_gun_source.STL'),T_flange_source=T.tolist(),transform_status='DRAFT')
    manifest['collision'].update(T_flange_source=T.tolist(),T_mount_source=(np.linalg.inv(Tfm)@T).tolist(),
                                  T_tool0_source=(np.linalg.inv(Tf0)@T).tolist(),export_file=str(target/'vhacd_collision_export.json'))
    manifest['mounting']['evidence_file']=str(target/'mount_hole_alignment.json')
    manifest.update(status='STAGED_ALIGNMENT_REQUIRED',requires_replan=True,installed_package=str(folder))
    manifest['preview']={'T_flange_source':T.tolist()}
    # Do not claim the previous CAD's nozzle point belongs to this STL.
    manifest['wire']={'status':'DISABLED_UNVERIFIED','reason':'Changed CAD source: previous wire datum is not transferred.'}
    manifest['wire_axis_status']='SOURCE_CHANGED_REVIEW_REQUIRED'
    for name in ('welding_gun_source.STL','welding_gun_visual_m.stl','stl_audit.json',
                 'vhacd_collision_export.json','mount_hole_alignment.json','geometry_review.json',
                 'new_gun_vhacd_source_preview.ttt','decomposition.log'):
        if (folder/name).is_file():
            shutil.copy2(folder/name,target/name)
    # Draft has zero incremental roll: the fitted source basis already
    # includes the old +90 physical branch. Never apply it twice.
    draft=dict(schema_version=1,calibration_type='real_gun_mount_adjustment',
               coordinate_frame='URDF flange',source_sha256=source_hash,status='DRAFT',
               confirmed_by_operator=False,applied_to_planning_model=False,
               T_flange_source=T.tolist(),T_flange_source_nominal=T.tolist(),
               roll_deg=0.0,translation_flange_mm=[0,0,0],rotation_axis_flange=[1,0,0],
               version='Areal-HOLES-DRAFT-'+source_hash[:16],requires_replan=True,
               created_utc=manifest['created_utc'],alignment_evidence=str(target/'mount_hole_alignment.json'))
    save(ROOT/'real_gun_mount_calibration.json',draft)
    # Replace the manifest last; changed hash/source makes old trajectories STALE.
    save(target/'real_gun_geometry.json',manifest)
    print(json.dumps(dict(status=manifest['status'],manifest=str(target/'real_gun_geometry.json'),
                          component_count=len(export['components']),backup=str(backup)),ensure_ascii=False))


if __name__=='__main__': main()
