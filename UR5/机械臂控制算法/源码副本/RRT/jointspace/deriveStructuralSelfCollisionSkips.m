function pairs = deriveStructuralSelfCollisionSkips(robot)
%deriveStructuralSelfCollisionSkips Keep only attachment interfaces ignored.

names = robot.BodyNames;
pairs = cell(0,2);
for i = 1:min(10,numel(names)-1)
    pairs(end+1,:) = {names{i},names{i+1}}; %#ok<AGROW>
end
extra = {{'WeldingGunMount','wrist_3_link'}; ...
         {'WeldingGunMount','ee_link'}; ...
         {'WeldingGunMount','WeldTCP'}};
% The model-defined wire is a fixed child of the physical gun mount.  The
% cylinder starts at the CAD gun-tip surface, so a contact with its parent
% mount is an attachment-interface contact, not an obstacle collision.  Keep
% the wire in all other self/environment collision checks (robot links,
% workpiece, etc.); skip only this rigid parent-child interface when both
% bodies are present in the REAL scene.
if any(strcmp(names,'WeldingGunMount')) && any(strcmp(names,'WeldingGunWire'))
    extra(end+1,:) = {{'WeldingGunMount','WeldingGunWire'}};
end
for i = 1:size(extra,1)
    pairs(end+1,:) = extra{i}; %#ok<AGROW>
end
end
