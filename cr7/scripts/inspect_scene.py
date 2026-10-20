import sys,json
sys.path.insert(0,'/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
s=RemoteAPIClient(port=23010).require('sim')
out=[]
for h in s.getObjectsInTree(s.handle_scene,s.handle_all,0):
 t=s.getObjectType(h)
 if t in [s.sceneobject_joint,s.sceneobject_dummy,s.sceneobject_script] or s.getObjectParent(h)==-1:
  out.append({'handle':h,'type':t,'path':s.getObjectAlias(h,1),'parent':s.getObjectParent(h)})
print(json.dumps(out,indent=2))
