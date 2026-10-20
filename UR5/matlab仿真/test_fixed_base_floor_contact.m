function report=test_fixed_base_floor_contact(sceneFile,outDir)
% Offline regression: exactly one matrix cell changes; moving collisions stay.
if ~isfolder(outDir), mkdir(outDir); end
s=load(sceneFile,'scene'); scene=s.scene;
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(fullfile(fileparts(root),'RRT')));
[scene.environmentCollisionPolicy,scene.fixedEnvironmentContact]=load_fixed_environment_contacts(scene,root);
policy=scene.environmentCollisionPolicy; assert(~isempty(policy));
originalRobot=copy(scene.robot); originalEnvironment=scene.environment;
strict=scene; strict.environmentCollisionPolicy=[];
before=validate_hmi_endpoint(strict,scene.qMat0,'STRICT HOME');
after=validate_hmi_endpoint(scene,scene.qMat0,'FIXED SUPPORT HOME');
assert(~before.valid && after.valid);
samples=[scene.qMat0;zeros(1,6);deg2rad([180 -90 0 -90 0 0]);sin((1:7)'*(1:6))*0.7];
checks=cell(1,size(samples,1)); names=hmi_environment_names(scene);
for k=1:size(samples,1)
    q=samples(k,:);
    [raw,D]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
    [filtered,F,~,contact]=check_scene_environment_collision(scene,q);
    expected=D; expected(policy.body_index,policy.environment_index)=Inf;
    assert(isequaln(F,expected),'FIXED_SUPPORT:Test','A non-exempt distance cell changed');
    assert(any(filtered(:))==any(isnan(expected(:))));
    [ii,jj]=find(isnan(F)); pairs=cell(1,numel(ii));
    for j=1:numel(ii), pairs{j}=[scene.robot.BodyNames{ii(j)} ' <-> ' names{jj(j)}]; end
    checks{k}=struct('q_rad',q,'raw_collision',any(raw(:)),...
        'filtered_collision',any(filtered(:)),'remaining_pairs',{pairs},'contact',contact);
end
assert(any(checks{2}.filtered_collision),'FIXED_SUPPORT:Test','Moving body/tool floor collisions were hidden');
bad=scene; floor=collisionBox(originalEnvironment{policy.environment_index}.X,...
    originalEnvironment{policy.environment_index}.Y,originalEnvironment{policy.environment_index}.Z);
floor.Pose=originalEnvironment{policy.environment_index}.Pose; floor.Pose(3,4)=floor.Pose(3,4)+0.001;
bad.environment{policy.environment_index}=floor;
shiftRejected=false;
try, check_scene_environment_collision(bad,bad.qMat0);
catch err, shiftRejected=strcmp(err.identifier,'FIXED_SUPPORT:STALE'); end
assert(shiftRejected);
bad=scene; bad.environmentNames{policy.environment_index}='EnvironmentCollision_OTHER';
renameRejected=false;
try, check_scene_environment_collision(bad,bad.qMat0);
catch err, renameRejected=strcmp(err.identifier,'FIXED_SUPPORT:STALE'); end
assert(renameRejected);
bad=scene; bad.robot=copy(scene.robot); base=getBody(bad.robot,'base_link_inertia');
T=getTransform(bad.robot,zeros(1,6),base.Name,base.Parent.Name); T(3,4)=T(3,4)+0.001;
joint=rigidBodyJoint('SYNTHETIC_MOVED_BASE_FIXED','fixed'); setFixedTransform(joint,T);
replaceJoint(bad.robot,base.Name,joint);
movedBaseRejected=false;
try, check_scene_environment_collision(bad,bad.qMat0);
catch err, movedBaseRejected=strcmp(err.identifier,'FIXED_SUPPORT:STALE'); end
assert(movedBaseRejected);
bad=scene; bad.environmentCollisionPolicy.pair{1}='shoulder_link';
arbitraryPairRejected=false;
try, check_scene_environment_collision(bad,bad.qMat0);
catch err, arbitraryPairRejected=strcmp(err.identifier,'FIXED_SUPPORT:Policy'); end
assert(arbitraryPairRejected);
% Base versus a DIFFERENT obstacle stays hard, using a temporary fixture.
fixture=scene; obstacle=collisionSphere(0.02); T=getTransform(scene.robot,scene.qMat0,'base_link_inertia');
obstacle.Pose=T; obstacle.Pose(3,4)=obstacle.Pose(3,4)+0.04;
fixture.environment{1}=obstacle;
[extra,E]=check_scene_environment_collision(fixture,fixture.qMat0);
assert(any(extra(:)) && isnan(E(policy.body_index,1)));
% FastConnect's common backend receives the policy; no search code is changed.
[free,detail]=collisionFreeJointState(scene.robot,scene.qMat0,scene.environment,...
    scene.selfCollisionSkipPairs,policy); assert(free && detail.fixedEnvironmentContact.raw_contact);
[strictFree,~]=collisionFreeJointState(scene.robot,scene.qMat0,scene.environment,...
    scene.selfCollisionSkipPairs); assert(~strictFree);
for k=1:numel(originalRobot.BodyNames)
    name=originalRobot.BodyNames{k};
    assert(isequal(getBody(originalRobot,name).Collisions,getBody(scene.robot,name).Collisions));
    assert(isequal(getTransform(originalRobot,scene.qMat0,name),getTransform(scene.robot,scene.qMat0,name)));
end
for k=1:numel(originalEnvironment), assert(isequal(originalEnvironment{k}.Pose,scene.environment{k}.Pose)); end
report=struct('pass',true,'strict_endpoint',before,'filtered_endpoint',after,...
    'matrix_checks',{checks},'exactly_one_pair_masked',true,'moving_collision_preserved',true,...
    'base_other_obstacle_collision_preserved',true,'shifted_floor_rejected',shiftRejected,...
    'renamed_floor_rejected',renameRejected,'moved_base_rejected',movedBaseRejected,...
    'arbitrary_pair_rejected',arbitraryPairRejected,...
    'rrt_backend_pass',free,'no_policy_stays_strict',~strictFree,...
    'geometry_and_FK_unchanged',true,'executionReady',false,'robot_motion','NO');
fid=fopen(fullfile(outDir,'fixed_contact_regression.json'),'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('FIXED SUPPORT REGRESSION PASS; one pair only, all moving/tool/wire pairs retained\n');
end
