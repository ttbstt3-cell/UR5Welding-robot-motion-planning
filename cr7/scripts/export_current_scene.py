"""Read the stopped main scene in-process, preserving full mesh components."""
import sys,time,subprocess
from pathlib import Path
sys.path.insert(0,'/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
root=Path(__file__).resolve().parents[1]
s=RemoteAPIClient(port=23010).require('sim');assert s.getSimulationState()==s.simulation_stopped
assert s.getObject('/CR7/WeldingGunMount/WeldTCP')>=0
p=root/'scene_geometry.json';old=p.stat().st_mtime_ns
code="function sysCall_init() dofile('"+str(root/'scripts/export_scene.lua')+"') end"
h=s.createScript(s.scripttype_customization,code,0,'lua')
try:
 for _ in range(240):
  time.sleep(.25)
  if p.stat().st_mtime_ns!=old:
   # Removing the script waits for its in-process callback to finish.
   s.removeObjects([h]);h=None;break
 else:raise TimeoutError('Geometry export did not complete')
finally:
 if h is not None:s.removeObjects([h])
subprocess.run([sys.executable,str(root/'scripts/prepare_geometry.py')],check=True)
print('Exported current CR7 geometry; run validate_fk.py and validate_cr7_fk next')
