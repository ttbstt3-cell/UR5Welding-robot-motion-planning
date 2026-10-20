function tree = propagateDescendantCosts(tree,rootIndex)
% propagateDescendantCosts  Update all costs below a rewired node.

if rootIndex < 1 || rootIndex > size(tree.nodes,1) || ...
        rootIndex ~= floor(rootIndex)
    error('rootIndex is not a valid tree node index.');
end

queue = rootIndex;
cursor = 1;

while cursor <= numel(queue)
    currentIndex = queue(cursor);
    cursor = cursor+1;
    children = find(tree.parent == currentIndex);

    for childIndex = reshape(children,1,[])
        tree.cost(childIndex) = tree.cost(currentIndex)+ ...
            norm(tree.nodes(childIndex,:)-tree.nodes(currentIndex,:));
    end

    queue = [queue; children]; %#ok<AGROW>
end

end
