function pairs=ignore_fixed_gun_wire_self_collision(robot,pairs)
% Only the fixed gun/wire attachment. Never exclude external body pairs.
names=robot.BodyNames;
if ~all(ismember({'WeldingGunMount','WeldingGunWire'},names)), return; end
pair={'WeldingGunMount','WeldingGunWire'};
exists=false;
for i=1:size(pairs,1)
    if isequal(sort(string(pairs(i,:))),sort(string(pair))), exists=true; break; end
end
if ~exists, pairs(end+1,:)=pair; end
end
