function [hit,distance,witness,contact]=check_scene_environment_collision(scene,q)
% Scene-level entry for the single explicitly certified fixed support pair.
policy=[];
if isfield(scene,'environmentCollisionPolicy'), policy=scene.environmentCollisionPolicy; end
if ~isempty(policy)
    names=hmi_environment_names(scene);
    assert(strcmp(names{policy.environment_index},policy.pair{2}),...
        'FIXED_SUPPORT:STALE','Certified environment object name/order changed');
end
[hit,distance,witness,contact]=checkEnvironmentCollision(scene.robot,q,scene.environment,policy);
end
