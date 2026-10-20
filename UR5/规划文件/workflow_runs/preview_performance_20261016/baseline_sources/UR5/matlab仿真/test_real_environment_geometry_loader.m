function report=test_real_environment_geometry_loader(savedSceneFile,outDir)
% Independent SYNTHETIC regression. Never replaces real environment data.
% Checks frame mapping, object names and robot/gun/wire collision attribution.
if ~isfolder(outDir), mkdir(outDir); end
data=load(savedSceneFile,'scene'); scene=data.scene; original=scene;
q=scene.qMat0; Trootbase=getTransform(scene.robot,zeros(1,6),'base');
Trootcop=scene.T_CopBase_MatBase\eye(4); Tcoproot=Trootcop\eye(4);
names=hmi_environment_names(scene);
template=struct('name','','path','','category','synthetic_test_only',...
    'collision_enabled',true,'pose_UR5_base',struct('position',[],'quaternion',[]),...
    'geometry',struct('type','sphere','radius_m',0.001));
specs=repmat(template,1,3);
Tbody=getTransform(scene.robot,q,'base_link_inertia');
if iscell(scene.gunProxyInfo), gunComponent=scene.gunProxyInfo{1};
else, gunComponent=scene.gunProxyInfo(1); end
gunCenter=mean(double(gunComponent.vertices_m),1);
Tgun=getTransform(scene.robot,q,char(scene.mountBodyName))*scene.T_mount_source;
gunPoint=Tgun*[gunCenter(:);1];
Twire=getTransform(scene.robot,q,char(scene.wireBodyName));
positions={Tbody(1:3,4),gunPoint(1:3),Twire(1:3,4)};
labels={'EnvironmentTest_robot','EnvironmentTest_gun','EnvironmentTest_wire'};
for k=1:3
    specs(k).name=labels{k}; specs(k).path=['/SYNTHETIC_ONLY/' labels{k}];
    T=eye(4); T(1:3,4)=positions{k}; Tcop=Tcoproot*T;
    specs(k).pose_UR5_base=struct('position',Tcop(1:3,4).',...
        'quaternion',matrixQuaternion(Tcop(1:3,1:3)));
end
specs(1).geometry=struct('type','sphere','radius_m',0.020);
payload=struct('schema_version','SYNTHETIC_ENVIRONMENT_LOADER_TEST',...
    'units','m','frame_id','UR5_base','version','SYNTHETIC_NOT_FOR_PLANNING',...
    'frames',struct('T_base_UR5base',Trootbase\Trootcop),'objects',specs);
fixture=fullfile(outDir,'synthetic_environment_fixture.json'); writeJson(fixture,payload);
[colliders,info,loadReport]=load_real_environment_geometry(fixture,Trootcop,Trootbase);
scene.environment=[scene.environment colliders]; scene.environmentNames=[names {info.name}];
assert(isequal(scene.selfCollisionSkipPairs,original.selfCollisionSkipPairs));
assert(isequal(scene.qOffset,original.qOffset));
for k=1:numel(original.environment)
    assert(isequal(scene.environment{k}.Pose,original.environment{k}.Pose));
end
expected={'RobotEnvironmentCollision','GunEnvironmentCollision','WireEnvironmentCollision'};
bodies={'base_link_inertia',char(scene.mountBodyName),char(scene.wireBodyName)};
endpoint=validate_hmi_endpoint(scene,q,'SYNTHETIC ENVIRONMENT TEST');
for k=1:3
    pair=[bodies{k} ' <-> ' labels{k}]; assert(any(strcmp(endpoint.collision_pairs,pair)),...
        'REAL_ENVIRONMENT:Test','Expected exact pair absent: %s',pair);
    assert(endpoint.collision_categories.(expected{k}).pair_count>0);
    assert(norm(colliders{k}.Pose(1:3,4)-positions{k})<1e-12);
end
bad=payload; bad.frames.T_base_UR5base(1,4)=0.001;
badFile=fullfile(outDir,'synthetic_stale_mapping.json'); writeJson(badFile,bad);
mappingRejected=false;
try, load_real_environment_geometry(badFile,Trootcop,Trootbase);
catch err, mappingRejected=strcmp(err.identifier,'REAL_ENVIRONMENT:STALE'); end
assert(mappingRejected,'REAL_ENVIRONMENT:Test','Stale frame mapping was accepted');
report=struct('pass',true,'scope','SYNTHETIC regression only; not real environment approval',...
    'source_scene',savedSceneFile,'synthetic_fixture',fixture,'loader',loadReport,...
    'endpoint',endpoint,'exact_pairs_pass',true,'pose_consistency_pass',true,...
    'stale_frame_mapping_rejected',mappingRejected,'old_environment_unchanged',true,...
    'self_collision_skips_unchanged',true,'qOffset_unchanged',true,'robot_motion','NO');
writeJson(fullfile(outDir,'synthetic_environment_regression.json'),report);
fprintf('SYNTHETIC environment regression PASS: %s\n',outDir);
end
function q=matrixQuaternion(R)
value=rotm2quat(R); q=[value(2:4) value(1)];
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
