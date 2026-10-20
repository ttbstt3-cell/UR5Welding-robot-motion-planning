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
% The model-defined wire is a fixed child of the physical gun mount.  Its
% CAD-defined cylinder starts at the mount surface, so contact with that
% parent is an attachment-interface contact rather than an obstacle hit.
% Keep wire-vs-robot and wire-vs-workpiece checks active; skip only the
% fixed parent-child interface when the REAL scene contains both bodies.
if any(strcmp(names,'WeldingGunMount')) && any(strcmp(names,'WeldingGunWire'))
    extra(end+1,:) = {{'WeldingGunMount','WeldingGunWire'}};
end
for i = 1:size(extra,1)
    pairs(end+1,:) = extra{i}; %#ok<AGROW>
end
end
