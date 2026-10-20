function result=validate_hmi_endpoint(scene,q,label,binding)
% Read-only endpoint gate using the complete saved scene and original skips.
% Does not alter q, frames, geometry, limits, or self-collision exclusions.
if nargin<3, label='endpoint'; end
if nargin<4, binding=struct(); end
result=struct('label',char(label),'status','FAIL','valid',false,...
    'q_rad',[],'q_deg',[],'collision_pairs',{{}},'collision_details',{{}},...
    'collision_coverage_pass',false,'missing_robot_collision_bodies',{{}},...
    'joint_limits_pass',false,'joint_limit_violations',[],...
    'calibration_binding',binding,'executionReady',false,'robot_motion','NO');
required={'base_link_inertia','shoulder_link','upper_arm_link','forearm_link',...
    'wrist_1_link','wrist_2_link','wrist_3_link'};
for k=1:numel(required)
    if ~any(strcmp(scene.robot.BodyNames,required{k})) || ...
            isempty(getBody(scene.robot,required{k}).Collisions)
        result.missing_robot_collision_bodies{end+1}=required{k}; %#ok<AGROW>
    end
end
result.collision_coverage_pass=isempty(result.missing_robot_collision_bodies);
if ~isnumeric(q) || numel(q)~=6 || ~isreal(q) || any(~isfinite(q(:)))
    result.failure_reason='INVALID_JOINT_VECTOR'; return;
end
q=reshape(double(q),1,6); result.q_rad=q; result.q_deg=rad2deg(q);
limits=scene.jointLimits;
result.joint_limit_violations=find(q(:)<limits(:,1) | q(:)>limits(:,2)).';
result.joint_limits_pass=isempty(result.joint_limit_violations);
result.joint_limits_rad=limits;
if ~result.joint_limits_pass
    result.failure_reason='JOINT_LIMIT_VIOLATION'; return;
end
robot=scene.robot; names=robot.BodyNames;
Tbase=getTransform(robot,q,'base'); Ttcp=getTransform(robot,q,scene.tcpBodyName);
result.TCP_pose_base=Tbase\Ttcp;
result.flange_pose_base=Tbase\getTransform(robot,q,'flange');
result.TCP_pose_URDFroot=Ttcp;
[envHit,envDistance,~,result.fixed_environment_contact]=check_scene_environment_collision(scene,q);
[selfHit,selfDistance]=checkCollision(robot,q,'IgnoreSelfCollision','off',...
    'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
result.self_collision=any(selfHit(:)); result.environment_collision=any(envHit(:));
% Enumerate actual self pairs individually. NaN in the full matrix alone
% cannot distinguish skipped/empty pairs from colliding pairs reliably.
allPairs=cell(0,2);
for a=1:numel(names)-1
    for b=a+1:numel(names), allPairs(end+1,:)={names{a},names{b}}; end %#ok<AGROW>
end
for a=1:numel(names)-1
    if isempty(getBody(robot,names{a}).Collisions), continue; end
    for b=a+1:numel(names)
        if isempty(getBody(robot,names{b}).Collisions), continue; end
        pair={names{a},names{b}};
        if skipped(pair,scene.selfCollisionSkipPairs), continue; end
        keep=strcmp(allPairs(:,1),pair{1})&strcmp(allPairs(:,2),pair{2});
        hit=checkCollision(robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',allPairs(~keep,:),'Exhaustive','on');
        if any(hit(:)), record(pair{1},pair{2},classify(pair{1},pair{2})); end
    end
end
envNames=hmi_environment_names(scene);
if any(envHit(:))
    for a=1:numel(names)
        if isempty(getBody(robot,names{a}).Collisions), continue; end
        for b=1:numel(scene.environment)
            if isnan(envDistance(a,b))
                record(names{a},envNames{b},classify(names{a},envNames{b},true));
            end
        end
    end
end
categories={'RobotSelfCollision','RobotGunCollision','RobotWorkpieceCollision',...
    'RobotWireCollision','GunWorkpieceCollision','WireWorkpieceCollision','ToolInternalCollision',...
    'RobotEnvironmentCollision','GunEnvironmentCollision','WireEnvironmentCollision'};
summary=struct();
for k=1:numel(categories)
    count=0;
    for j=1:numel(result.collision_details)
        count=count+strcmp(result.collision_details{j}.category,categories{k});
    end
    summary.(categories{k})=struct('status',ternary(count==0,'PASS','FAIL'),'pair_count',count);
end
result.collision_categories=summary;
result.self_collision_skip_pairs=scene.selfCollisionSkipPairs;
result.attached_gun_wire_collision_policy=struct('status','SKIPPED_FIXED_ATTACHMENT',...
    'pair',{{'WeldingGunMount','WeldingGunWire'}},'checked',false,...
    'reason','Existing fixed internal attachment exclusion is retained; external wire/gun/robot/environment pairs remain checked');
if summary.ToolInternalCollision.pair_count==0
    result.collision_categories.ToolInternalCollision.status='SKIPPED_FIXED_ATTACHMENT';
end
result.wire_collision_pass=summary.RobotWireCollision.pair_count==0 && ...
    summary.WireWorkpieceCollision.pair_count==0 && summary.WireEnvironmentCollision.pair_count==0;
result.environment_object_names=envNames;
result.environment_object_count=numel(scene.environment);
if isfield(scene,'realEnvironmentGeometryLoad')
    result.environment_geometry_load=scene.realEnvironmentGeometryLoad;
    result.environment_collision_scope='Only objects in the saved exported environment; missing source categories are not certified';
end
positive=[envDistance(isfinite(envDistance)&envDistance>0);...
    selfDistance(isfinite(selfDistance)&selfDistance>0)];
result.minimum_positive_separation_mm=[];
if ~isempty(positive), result.minimum_positive_separation_mm=min(positive)*1000; end
result.minimum_positive_separation_scope='Non-colliding pairs only; not clearance of a colliding endpoint';
result.valid=result.collision_coverage_pass && result.joint_limits_pass && ...
    ~result.self_collision && ~result.environment_collision;
if result.valid, result.status='PASS'; result.failure_reason='';
elseif ~result.collision_coverage_pass, result.failure_reason='ROBOT_COLLISION_GEOMETRY_MISSING';
else, result.failure_reason='COLLISION'; end
fprintf('%s COLLISION CHECK: %s\n',upper(char(label)),ternary(result.valid,'FREE','COLLISION/INVALID'));
for k=1:numel(result.collision_pairs), fprintf('  %s\n',result.collision_pairs{k}); end

    function record(a,b,category)
        result.collision_pairs{end+1}=[a ' <-> ' b];
        result.collision_details{end+1}=struct('object_a',a,'object_b',b,...
            'category',category,'separation_mm',[],'penetration_depth_mm',[],...
            'distance_status','Collision; native signed penetration unavailable');
    end
end
function yes=skipped(pair,skip)
yes=any((strcmp(skip(:,1),pair{1})&strcmp(skip(:,2),pair{2}))|...
    (strcmp(skip(:,1),pair{2})&strcmp(skip(:,2),pair{1})));
end
function category=classify(a,b,isEnvironment)
if nargin<3, isEnvironment=false; end
wire=contains(string({a,b}),'Wire'); gun=contains(string({a,b}),'Gun') & ~wire;
env=startsWith(string({a,b}),'WorkpieceCollision');
if any(env)
    if any(wire), category='WireWorkpieceCollision';
    elseif any(gun), category='GunWorkpieceCollision';
    else, category='RobotWorkpieceCollision'; end
elseif isEnvironment
    if any(wire), category='WireEnvironmentCollision';
    elseif any(gun), category='GunEnvironmentCollision';
    else, category='RobotEnvironmentCollision'; end
elseif any(wire) && any(gun), category='ToolInternalCollision';
elseif any(wire), category='RobotWireCollision';
elseif any(gun), category='RobotGunCollision';
else, category='RobotSelfCollision'; end
end
function value=ternary(condition,a,b)
if condition, value=a; else, value=b; end
end
