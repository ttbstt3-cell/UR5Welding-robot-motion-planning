function [free,gap]=hmi_workflow_state_free(scene,q)
% Use the scene's existing limit and self-collision exemptions without edits.
free=false; gap=NaN; q=double(q(:).');
if isfield(scene,'jointLimits'), lim=scene.jointLimits;
else, lim=deriveUR5JointLimits(scene.robot); end
if numel(q)~=6 || any(~isfinite(q)) || any(q<lim(:,1).') || any(q>lim(:,2).'), return; end
[env,de]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
[self,ds]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off',...
    'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
free=~any(env(:)) && ~any(self(:));
d=[de(:);ds(:)]; d=d(isfinite(d)&d>0); if ~isempty(d), gap=1000*min(d); end
end
