import sys,time,json
from pathlib import Path
sys.path.insert(0,'/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
root=Path(__file__).resolve().parents[1];s=RemoteAPIClient(port=23010).require('sim')
assert s.getSimulationState()==s.simulation_stopped
robot=s.getObject('/CR7')
# Save the arm only; source and validated native scene remain on disk.
for path in ['/CR7/hanqiang','/CR7/TCP']:
 h=s.getObject(path,{'noError':True})
 if h!=-1:s.removeObjects(s.getObjectsInTree(h,s.handle_all,0))
for h in s.getObjectsInTree(robot,s.sceneobject_script,0):s.removeObjects([h])
s.setModelProperty(robot,0);s.saveModel(robot,str(root/'simulation/cr7_arm.ttm'))
s.loadScene('/home/ttbstt/jixiebi/UR5/ur5.ttt')
ur5=s.getObject('/UR5');pose=s.getObjectPose(ur5,s.handle_world)
mount=s.getObject('/UR5/connection/WeldingGunMount')
s.setObjectParent(mount,-1,True)
s.removeObjects(s.getObjectsInTree(ur5,s.handle_all,0))
robot=s.loadModel(str(root/'simulation/cr7_arm.ttm'));s.setObjectPose(robot,s.handle_world,pose)
s.setObjectParent(mount,s.getObject('/CR7/joint6'),False);s.setObjectPose(mount,s.getObject('/CR7/joint6'),[0,0,0,0,0,0,1])
for h in s.getObjectsInTree(s.handle_scene,s.sceneobject_script,0):
 # Stop inherited simulation controllers; archive their text.
 (root/'simulation'/f'ur5_source_script_{h}.txt').write_text(s.getScriptStringParam(h,s.scriptstringparam_text))
 s.removeObjects([h])
for h in s.getObjectsInTree(s.handle_scene,s.sceneobject_shape,0):
 s.setObjectInt32Param(h,s.shapeintparam_static,1);s.setObjectInt32Param(h,s.shapeintparam_respondable,0)
script=s.createScript(s.scripttype_simulation,(root/'simulation/joint_states.lua').read_text(),0,'lua');s.setObjectParent(script,robot,True);s.setObjectAlias(script,'joint_states_publisher')
s.saveScene(str(root/'simulation/cr7.ttt'))
print('Migrated UR5 scene, mount and frozen WeldTCP; CR7 base occupies former UR5 base pose')
print('TCP local',s.getObjectPose(s.getObject('/CR7/WeldingGunMount/WeldTCP'),mount))
