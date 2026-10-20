function descendant = isDescendant(tree,ancestorIndex,nodeIndex)
% isDescendant  True when nodeIndex is below ancestorIndex in the tree.
%
% The ancestor node itself is not considered its own descendant.

if size(tree.nodes,2) ~= 2 || size(tree.parent,2) ~= 1 || ...
        numel(tree.parent) ~= size(tree.nodes,1)
    error('tree.nodes must be N-by-2 and tree.parent must be N-by-1.');
end

validateIndex(ancestorIndex,size(tree.nodes,1),'ancestorIndex');
validateIndex(nodeIndex,size(tree.nodes,1),'nodeIndex');

descendant = false;
visited = false(size(tree.parent));
currentIndex = nodeIndex;

while tree.parent(currentIndex) ~= 0
    if visited(currentIndex)
        error('parent chain contains a cycle at node %d.',currentIndex);
    end
    visited(currentIndex) = true;

    parentIndex = tree.parent(currentIndex);
    if parentIndex < 1 || parentIndex > size(tree.nodes,1) || ...
            parentIndex ~= floor(parentIndex)
        error('parent chain contains an invalid index at node %d.',currentIndex);
    end

    if parentIndex == ancestorIndex
        descendant = true;
        return;
    end

    currentIndex = parentIndex;
end

end

function validateIndex(index,numberOfNodes,name)
if ~isscalar(index) || index < 1 || index > numberOfNodes || ...
        index ~= floor(index)
    error('%s is not a valid tree node index.',name);
end
end
