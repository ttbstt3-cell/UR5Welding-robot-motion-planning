function [smoothedPath,actualDegree] = generateBSplinePath( ...
    controlPoints,degree,sampleCount)
% generateBSplinePath  Generate a clamped uniform B-spline curve.

controlPoints = reshape(controlPoints,[],2);
if isempty(controlPoints)
    smoothedPath = zeros(0,2);
    actualDegree = 0;
    return;
end
if ~isscalar(degree) || ~isfinite(degree) || degree < 1 || ...
        degree ~= floor(degree)
    error('degree must be a positive integer.');
end
if ~isscalar(sampleCount) || ~isfinite(sampleCount) || sampleCount < 2 || ...
        sampleCount ~= floor(sampleCount)
    error('sampleCount must be an integer of at least 2.');
end

numberOfPoints = size(controlPoints,1);
actualDegree = min(degree,numberOfPoints-1);
if actualDegree < 1
    smoothedPath = repmat(controlPoints(1,:),sampleCount,1);
    return;
end

interiorCount = numberOfPoints-actualDegree-1;
if interiorCount > 0
    interiorKnots = (1:interiorCount)/(interiorCount+1);
else
    interiorKnots = zeros(1,0);
end
knotVector = [zeros(1,actualDegree+1),interiorKnots, ...
    ones(1,actualDegree+1)];
parameters = linspace(0,1,sampleCount).';
smoothedPath = zeros(sampleCount,2);

for sampleIndex = 1:sampleCount
    u = parameters(sampleIndex);
    basisValues = zeros(1,numberOfPoints);
    for controlIndex = 1:numberOfPoints
        basisValues(controlIndex) = bsplineBasis( ...
            controlIndex,actualDegree,u,knotVector);
    end
    smoothedPath(sampleIndex,:) = basisValues*controlPoints;
end

% Explicit endpoint assignment removes any endpoint round-off and enforces
% the path contract required by the planner.
smoothedPath(1,:) = controlPoints(1,:);
smoothedPath(end,:) = controlPoints(end,:);

end

function value = bsplineBasis(index,degree,u,knots)
if degree == 0
    value = double((knots(index) <= u && u < knots(index+1)) || ...
        (u == 1 && knots(index+1) == 1 && knots(index) <= 1));
    return;
end

leftDenominator = knots(index+degree)-knots(index);
rightDenominator = knots(index+degree+1)-knots(index+1);
leftTerm = 0;
rightTerm = 0;
if leftDenominator > 0
    leftTerm = (u-knots(index))/leftDenominator* ...
        bsplineBasis(index,degree-1,u,knots);
end
if rightDenominator > 0
    rightTerm = (knots(index+degree+1)-u)/rightDenominator* ...
        bsplineBasis(index+1,degree-1,u,knots);
end
value = leftTerm+rightTerm;
end
