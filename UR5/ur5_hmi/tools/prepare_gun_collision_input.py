#!/usr/bin/env python3
"""Topology-preserving COLLISION input derivative; original visual unchanged."""
import json
from pathlib import Path
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

ROOT=Path(__file__).resolve().parents[2]/'real_gun_geometry'
reader=vtk.vtkSTLReader(); reader.SetFileName(str(ROOT/'welding_gun_source.STL')); reader.Update()
source=reader.GetOutput()
dec=vtk.vtkDecimatePro(); dec.SetInputData(source)
dec.PreserveTopologyOn(); dec.SplittingOff(); dec.BoundaryVertexDeletionOff()
dec.SetTargetReduction(0.98); dec.SetErrorIsAbsolute(1); dec.SetAbsoluteError(0.1) # source units mm
dec.Update(); result=dec.GetOutput()
writer=vtk.vtkSTLWriter(); writer.SetFileTypeToBinary()
writer.SetFileName(str(ROOT/'welding_gun_collision_input_mm.stl'))
writer.SetInputConnection(dec.GetOutputPort()); assert writer.Write()
distance=vtk.vtkImplicitPolyDataDistance(); distance.SetInput(result)
pts=vtk_to_numpy(source.GetPoints().GetData())
idx=np.linspace(0,len(pts)-1,min(len(pts),10000)).astype(int)
errors=np.array([abs(distance.EvaluateFunction(pts[i])) for i in idx])
report={'method':'vtkDecimatePro topology-preserving','target_reduction':0.98,
        'absolute_error_setting_mm':0.1,'source_faces':source.GetNumberOfCells(),
        'collision_input_faces':result.GetNumberOfCells(),'sample_count':len(errors),
        'sampled_max_surface_distance_mm':float(errors.max()),
        'sampled_rms_surface_distance_mm':float(np.sqrt(np.mean(errors**2))),
        'source_bounds_mm':list(source.GetBounds()),'derived_bounds_mm':list(result.GetBounds()),
        'source_modified':False,'recenter_rotate_applied':False,
        'note':'Sampled distance, not a certified continuous Hausdorff bound; original full mesh is visual only.'}
(ROOT/'collision_input_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
