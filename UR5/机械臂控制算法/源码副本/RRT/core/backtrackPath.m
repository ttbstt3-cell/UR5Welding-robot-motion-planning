function path = backtrackPath(nodes,parent,index)
% backtrackPath  Return a root-to-index path from a parent array.

if size(nodes,2) ~= 2 || size(parent,2) ~= 1 || size(nodes,1) ~= numel(parent)
    error('nodes must be N-by-2 and parent must be an N-by-1 vector.');
end

if ~isscalar(index) || index < 1 || index > size(nodes,1) || index ~= floor(index)
    error('index is not a valid node index.');
end

path = zeros(0,2);
visited = false(size(parent));

while true
    if visited(index)
        error('parent array contains a cycle.');
    end

    visited(index) = true;
    path = [nodes(index,:); path]; %#ok<AGROW>

    if parent(index) == 0
        break;
    end

    if parent(index) < 1 || parent(index) > size(nodes,1) || ...
            parent(index) ~= floor(parent(index))
        error('parent array contains an invalid index.');
    end

    index = parent(index);
end

end
