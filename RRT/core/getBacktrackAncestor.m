function ancestorIndex = getBacktrackAncestor(tree,nodeIndex,depth)
% getBacktrackAncestor  Return the exact ancestor at a requested depth.
%
% depth=0 returns nodeIndex.  If the root is reached before the requested
% depth, the function returns 0 instead of fabricating an invalid index.

validateTree(tree);
validateIndex(nodeIndex,size(tree.nodes,1),'nodeIndex');
if ~isscalar(depth) || ~isfinite(depth) || depth < 0 || depth ~= floor(depth)
    error('depth must be a non-negative integer.');
end

ancestorIndex = nodeIndex;
visited = false(size(tree.parent));
for level = 1:depth
    if visited(ancestorIndex)
        error('parent chain contains a cycle at node %d.',ancestorIndex);
    end
    visited(ancestorIndex) = true;
    parentIndex = tree.parent(ancestorIndex);
    if parentIndex == 0
        ancestorIndex = 0;
        return;
    end
    validateIndex(parentIndex,size(tree.nodes,1),'parentIndex');
    ancestorIndex = parentIndex;
end

end

function validateTree(tree)
if ~isstruct(tree) || ~isfield(tree,'nodes') || ...
        ~isfield(tree,'parent') || size(tree.nodes,2) ~= 2 || ...
        size(tree.parent,2) ~= 1 || ...
        numel(tree.parent) ~= size(tree.nodes,1)
    error('tree must contain an N-by-2 nodes matrix and parent vector.');
end
end

function validateIndex(index,numberOfNodes,name)
if ~isscalar(index) || index < 1 || index > numberOfNodes || ...
        index ~= floor(index)
    error('%s is not a valid tree node index.',name);
end
end
