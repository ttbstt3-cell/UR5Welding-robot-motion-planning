function lengthValue = pathLength(path)
% pathLength  Sum Euclidean lengths of consecutive 2-D path points.

if isempty(path)
    lengthValue = NaN;
    return;
end

if size(path,2) ~= 2
    error('path must be an N-by-2 matrix.');
end

if size(path,1) < 2
    lengthValue = 0;
    return;
end

segments = diff(path,1,1);
lengthValue = sum(sqrt(sum(segments.^2,2)));

end
