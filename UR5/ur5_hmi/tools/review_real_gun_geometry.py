#!/usr/bin/env python3
"""Generate reproducible offline geometry review inputs, never robot commands."""
import hashlib
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import vtk
import yaml
from scipy.spatial import ConvexHull
from vtk.util.numpy_support import vtk_to_numpy

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ur5_hmi'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,default=ROOT/'real_gun_geometry')
    args=parser.parse_args()
    folder=args.folder.resolve()
    manifest=json.loads((folder/'real_gun_geometry.json').read_text())
    if manifest['collision']['method']!='CoppeliaSim simConvex.vhacd':
        raise RuntimeError('Actual V-HACD export has not completed')
    source=folder/'welding_gun_source.STL'
    reader=vtk.vtkSTLReader(); reader.SetFileName(str(source)); reader.Update()
    poly=reader.GetOutput()
    points=vtk_to_numpy(poly.GetPoints().GetData()).astype(float)*0.001
    edges=vtk.vtkFeatureEdges(); edges.SetInputData(poly)
    edges.BoundaryEdgesOn(); edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff(); edges.ManifoldEdgesOff(); edges.Update()
    # Unit-converted derived visual for MATLAB addVisual; original untouched.
    tr=vtk.vtkTransform(); tr.Scale(0.001,0.001,0.001)
    filt=vtk.vtkTransformPolyDataFilter(); filt.SetInputData(poly); filt.SetTransform(tr); filt.Update()
    writer=vtk.vtkSTLWriter(); writer.SetFileTypeToBinary()
    writer.SetFileName(str(folder/'welding_gun_visual_m.stl'))
    writer.SetInputConnection(filt.GetOutputPort()); assert writer.Write()
    sample=points[np.linspace(0,len(points)-1,min(len(points),20000)).astype(int)]
    violations=np.full(len(sample),np.inf)
    comps=manifest['collision']['components']
    volumes=[]
    for c in comps:
        hull=ConvexHull(np.asarray(c['vertices_m']))
        eq=hull.equations
        # Normalized convex half-space violation is a coverage diagnostic,
        # not exact Euclidean distance and not collision safety acceptance.
        violations=np.minimum(violations,np.max(sample@eq[:,:3].T+eq[:,3],axis=1))
        volumes.append(float(hull.volume))
    config=ROOT/'ur5_hmi/config/hmi_config.yaml'
    home=np.asarray(yaml.safe_load(config.read_text())['real_home'],dtype=float)
    poses=np.tile(home,(10,1))
    for i in range(1,10):
        poses[i]+=np.deg2rad(10)*np.sin(np.arange(6)+i)
    from gui.robot_view import UrdfKinematicModel
    model=UrdfKinematicModel()
    samples=[]
    # Identity source-at-flange is explicitly NOT a measured install pose.
    # ±90 previews are for operator inspection, not minimum-distance selection.
    from gui.robot_view import _axis_rotation
    T0=model.link_transforms_in_base(np.zeros(6))
    j=model.joints['wrist_3_joint']
    axis=T0['flange'][:3,:3].T@T0[j['parent']][:3,:3]@j['origin'][:3,:3]@np.asarray(j['axis'])
    for angle in (0,90,-90):
        T=_axis_rotation(axis,np.deg2rad(angle))
        request={'q_rad':poses.tolist(),'qHome':home.tolist(),
                 'qHome_source':str(config),'T_flange_source':T.tolist(),
                 'mount_status':'UNCONFIRMED_SOURCE_AT_FLANGE',
                 'angle_deg':angle,'source_sha256':manifest['source']['sha256']}
        (folder/f'review_request_{angle:+d}.json').write_text(json.dumps(request,indent=2)+'\n')
        if angle==0:
            for q in poses:
                Tf=model.link_transforms_in_base(q)['flange']
                Ts=Tf@T
                centers=[(Ts@np.r_[np.mean(c['vertices_m'],axis=0),1])[:3].tolist() for c in comps]
                samples.append({'q_rad':q.tolist(),'T_base_flange':Tf.tolist(),
                                'T_base_source':Ts.tolist(),'component_centers_base_m':centers})
    audit={'status':'DECOMPOSED_ALIGNMENT_REVIEW_REQUIRED',
           'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
           'source_vertex_count':len(points),'boundary_or_nonmanifold_edge_count':edges.GetOutput().GetNumberOfCells(),
           'component_count':len(comps),'component_volume_sum_m3_overlap_not_removed':sum(volumes),
           'surface_vertex_sample_count':len(sample),
           'max_convex_halfspace_violation_mm':float(max(0,violations.max())*1000),
           'sample_fraction_inside_union':float(np.mean(violations<=1e-8)),
           'coverage_note':'Sampled normalized plane violation only; not Euclidean distance, no automatic collision PASS.',
           'wire_status':str(manifest.get('wire',{}).get('status','UNAVAILABLE')),
           'physical_mount_status':'NOT_CONFIRMED','robot_motion':'NO',
           'hmi_samples':samples}
    (folder/'geometry_review.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps({k:v for k,v in audit.items() if k!='hmi_samples'},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
