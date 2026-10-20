function indices = nearNodes(nodes,point,radius)
% nearNodes  Return all node indices within a Euclidean radius.

if size(nodes,2) ~= 2
    error('nodes must be an N-by-2 matrix.');
end

point = reshape(point,1,[]);
if numel(point) ~= 2
    error('point must contain exactly two coordinates.');
end

if ~isscalar(radius) || ~isfinite(radius) || radius < 0
    error('radius must be a finite non-negative scalar.');
end

differences = nodes-point;
indices = find(sum(differences.^2,2) <= radius^2+1e-12);
indices = reshape(indices,[],1);

end
