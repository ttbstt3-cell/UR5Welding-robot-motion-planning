from pathlib import Path
import json,numpy as np
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[1]
d=json.loads((root/'scene_geometry.json').read_text())
for s in d['shapes']:
 s['robot']=s['robot'] or s['name'] in ['world_visual','CR7']
 v=np.array(s['vertices']).reshape(-1,3)
 hull=ConvexHull(v)
 s['collision_vertices']=v[hull.vertices].tolist()
 s['collision_components']=[]
 for component in s.pop('components',[]):
  points=np.array(component).reshape(-1,3)
  try: s['collision_components'].append({'vertices':points[ConvexHull(points).vertices].tolist()})
  except Exception: pass
 # Keep display mesh in separate files, avoid decoding a giant JSON in MATLAB.
 name=s['name']; p=root/'simulation'/f'{name}.mat'
 from scipy.io import savemat
 savemat(p,{'vertices':v,'faces':np.array(s.pop('indices')).reshape(-1,3)+1},do_compression=True)
 del s['vertices'];s['mesh_file']=str(p)
source_path=root/'simulation/workpiece_source.json'
d['ur5_workpiece']=json.loads(source_path.read_text())
(root/'scene_geometry_matlab.json').write_text(json.dumps(d,indent=2))
