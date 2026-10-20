function q = sampleJointConfig(jointLimits,goal,goalBias)
%sampleJointConfig Uniform joint-limit sample with optional goal bias.

if nargin < 2, goal = []; end
if nargin < 3 || isempty(goalBias), goalBias = 0; end
if ~isequal(size(jointLimits),[6 2])
    error('jointLimits must be 6-by-2.');
end
if ~isscalar(goalBias) || ~isfinite(goalBias) || goalBias < 0 || goalBias > 1
    error('goalBias must be in [0,1].');
end
if ~isempty(goal) && rand < goalBias
    q = reshape(double(goal),1,[]);
    return;
end
q = jointLimits(:,1).' + rand(1,6).*(jointLimits(:,2)-jointLimits(:,1)).';
end
