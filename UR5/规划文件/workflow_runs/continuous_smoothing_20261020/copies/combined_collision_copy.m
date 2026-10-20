function [ok,gap,envHit,selfHit,distEnv,distSelf]=combined_collision_copy(scene,q)
% Public exhaustive API, same pair set, same one fixed-support exception.
% No temporal/coarse cache and no change to geometry or distance thresholds.
robot=scene.robot; n=numel(robot.BodyNames)+1; env={};
if isfield(scene,'environment'), env=scene.environment; end
if isfield(scene,'selfCollisionSkipPairs')
    [raw,dist]=checkCollision(robot,q,env,'IgnoreSelfCollision','off',...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
else
    [raw,dist]=checkCollision(robot,q,env,'IgnoreSelfCollision','off','Exhaustive','on');
end
assert(isequal(size(dist),[n,n+numel(env)]),'AUDIT:NativeMatrix','Unexpected combined distance matrix');
distSelf=dist(:,1:n); distEnv=dist(:,n+1:end);
selfHit=any(isnan(distSelf(:))); envHit=any(isnan(distEnv(:)));
assert((numel(raw)==2 && isequal(logical(raw(:)),logical([selfHit;envHit]))) ||...
    (isempty(env) && numel(raw)==1 && logical(raw)==selfHit),...
    'AUDIT:NativeFlags','Native exhaustive flags disagree with pair matrix');
if isfield(scene,'environmentCollisionPolicy') && ~isempty(scene.environmentCollisionPolicy)
    policy=scene.environmentCollisionPolicy; names=hmi_environment_names(scene);
    assert(strcmp(names{policy.environment_index},policy.pair{2}),...
        'FIXED_SUPPORT:STALE','Certified environment identity/order changed');
    assert(isstruct(policy) && isscalar(policy) && strcmp(policy.status,'MODEL_VERIFIED_FIXED_SUPPORT') &&...
        isequal(policy.pair,{'base_link_inertia','EnvironmentCollision_Floor'}),...
        'FIXED_SUPPORT:Policy','Unsupported environment exception');
    i=policy.body_index; j=policy.environment_index;
    assert(i<=numel(robot.BodyNames) && strcmp(robot.BodyNames{i},policy.pair{1}) &&...
        j<=numel(env) && isequal(env{j}.Pose,policy.floor_pose) && isa(env{j},'collisionBox') &&...
        isequal([env{j}.X env{j}.Y env{j}.Z],policy.floor_size_m),...
        'FIXED_SUPPORT:STALE','Certified body/floor geometry changed');
    assert(isequal(getTransform(robot,q,policy.pair{1}),policy.T_root_base_body),...
        'FIXED_SUPPORT:STALE','Certified base is not fixed');
    distEnv(i,j)=Inf; envHit=any(isnan(distEnv(:)));
end
d=[distSelf(:);distEnv(:)]; d=d(isfinite(d)&d>0); gap=NaN;
if ~isempty(d), gap=min(d)*1000; end
ok=~envHit && ~selfHit;
end
