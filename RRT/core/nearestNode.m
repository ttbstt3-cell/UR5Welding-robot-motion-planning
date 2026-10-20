function [index,distance] = nearestNode(nodes,point)
% nearestNode  Return the closest row of an N-by-2 node matrix.

if size(nodes,2) ~= 2
    error('nodes must be an N-by-2 matrix.');
end

point = reshape(point,1,[]);
if numel(point) ~= 2
    error('point must contain exactly two coordinates.');
end

differences = nodes-point;
distancesSquared = sum(differences.^2,2);
[~,index] = min(distancesSquared);
distance = sqrt(distancesSquared(index));

end
