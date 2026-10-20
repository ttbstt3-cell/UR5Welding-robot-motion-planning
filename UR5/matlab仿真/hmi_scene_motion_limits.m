function limits=hmi_scene_motion_limits(scene)
% New HMI requests carry a sourced profile; legacy experiments stay frozen.
limits=struct('velocityLimit',deg2rad(5),'accelerationLimit',0.261799);
if ~isfield(scene,'hmiMotionLimits'), return; end
contract=scene.hmiMotionLimits;
assert(all(isfinite([contract.velocity_rad_s(:);contract.acceleration_rad_s2(:)])) &&...
    numel(contract.velocity_rad_s)==6 && numel(contract.acceleration_rad_s2)==6 &&...
    all(contract.velocity_rad_s>0) && all(contract.acceleration_rad_s2>0),...
    'LIMITS:Invalid','Invalid sourced scene motion limits');
% Existing scalar optimizers use the most restrictive joint. The independent
% execution audit additionally checks every joint's exact cubic extrema.
limits.velocityLimit=min(contract.velocity_rad_s); limits.accelerationLimit=min(contract.acceleration_rad_s2);
end
