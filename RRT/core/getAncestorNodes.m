function ancestorIndices = getAncestorNodes(tree,nodeIndex,depth)
% getAncestorNodes  Return up to depth ancestors nearest-first.
%
% Example: for 1 <- 2 <- 3 <- 4 <- 5,
% getAncestorNodes(tree,5,2) returns [4; 3].

validateTree(tree);

if ~isscalar(nodeIndex) || nodeIndex < 1 || ...
        nodeIndex > size(tree.nodes,1) || nodeIndex ~= floor(nodeIndex)
    error('nodeIndex is not a valid tree node index.');
end

if ~isscalar(depth) || ~isfinite(depth) || depth < 0 || depth ~= floor(depth)
    error('depth must be a non-negative integer.');
end

ancestorIndices = zeros(0,1);
currentIndex = nodeIndex;

for level = 1:depth
    parentIndex = tree.parent(currentIndex);

    if parentIndex == 0
        break;
    end

    if parentIndex < 1 || parentIndex > size(tree.nodes,1) || ...
            parentIndex ~= floor(parentIndex)
        error('parent chain contains an invalid index at node %d.',currentIndex);
    end

    if any(ancestorIndices == parentIndex)
        error('parent chain contains a cycle at node %d.',parentIndex);
    end

    ancestorIndices(end+1,1) = parentIndex; %#ok<AGROW>
    currentIndex = parentIndex;
end

end

function validateTree(tree)
if ~isstruct(tree) || ~isfield(tree,'nodes') || ...
        ~isfield(tree,'parent')
    error('tree must contain nodes and parent fields.');
end

if size(tree.nodes,2) ~= 2 || size(tree.parent,2) ~= 1 || ...
        numel(tree.parent) ~= size(tree.nodes,1)
    error('tree.nodes must be N-by-2 and tree.parent must be N-by-1.');
end
end
