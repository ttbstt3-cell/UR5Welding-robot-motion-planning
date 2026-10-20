import sys,json,numpy as np
from pathlib import Path
sys.path.insert(0,'/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
s=RemoteAPIClient(port=23010).require('sim');root=Path(__file__).resolve().parents[1]
assert s.getSimulationState()==s.simulation_stopped
j=[s.getObject('/CR7/joint'+str(i)) for i in range(1,7)]
q0=[s.getJointPosition(h) for h in j];samples=[]
try:
 for q in [np.zeros(6),np.deg2rad([10,-30,45,-30,20,10]),np.deg2rad([-25,15,-20,35,-15,25]),q0]:
  for h,v in zip(j,q):s.setJointPosition(h,float(v))
  samples.append({'q':list(q),'tcp_matrix':s.getObjectMatrix(s.getObject('/CR7/WeldingGunMount/WeldTCP'),s.getObject('/CR7'))})
finally:
 for h,v in zip(j,q0):s.setJointPosition(h,v)
(root/'simulation/fk_samples.json').write_text(json.dumps(samples,indent=2))
print('Exported',len(samples),'FK configurations; initial state restored')
