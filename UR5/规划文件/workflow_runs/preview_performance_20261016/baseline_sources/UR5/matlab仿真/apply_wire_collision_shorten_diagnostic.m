function scene = apply_wire_collision_shorten_diagnostic(scene,shortenMm)
% Apply a diagnostic-only shortening to the REAL wire collision cylinder.
% The visual WeldingGunWire geometry, TCP, wire manifest, collision pose and
% planner constraints remain unchanged. Only the cylinder length changes.
if nargin < 2 || isempty(shortenMm), shortenMm = 0; end
shortenMm = double(shortenMm);
if ~isscalar(shortenMm) || ~isfinite(shortenMm) || shortenMm < 0
    error('WIRE_DIAGNOSTIC:Shorten','wire_collision_shorten_mm must be a finite nonnegative scalar');
end
if ~isfield(scene,'wireLengthM') || ~isfinite(scene.wireLengthM) || scene.wireLengthM <= 0
    error('WIRE_DIAGNOSTIC:Wire','A valid REAL wire length is required');
end
fullMm = scene.wireLengthM*1000;
if shortenMm >= fullMm
    error('WIRE_DIAGNOSTIC:Shorten','wire collision shortening %.6g mm must be less than %.6g mm',shortenMm,fullMm);
end
scene.wireCollisionShortenMm = shortenMm;
scene.wireCollisionLengthM = scene.wireLengthM - shortenMm*1e-3;
if shortenMm == 0
    fprintf('Wire collision diagnostic: unchanged length %.6f mm\n',fullMm);
    return;
end
if ~isfield(scene,'wireBodyName') || ~any(strcmp(scene.robot.BodyNames,scene.wireBodyName))
    error('WIRE_DIAGNOSTIC:Body','REAL WeldingGunWire body is unavailable');
end
wireBody = getBody(scene.robot,scene.wireBodyName);
clearCollision(wireBody);
% Keep the original collision shape transform exactly unchanged: this
% diagnostic changes only the collisionCylinder height argument.
addCollision(wireBody,collisionCylinder(scene.wireDiameterM/2,scene.wireCollisionLengthM),eye(4));
replaceBody(scene.robot,scene.wireBodyName,wireBody);
fprintf('Wire collision diagnostic: shorten=%.3f mm, collision length=%.6f mm, visual length=%.6f mm\n', ...
    shortenMm,scene.wireCollisionLengthM*1000,scene.wireLengthM*1000);
end
