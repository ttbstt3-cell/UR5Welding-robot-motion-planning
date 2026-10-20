function config = normalizeJointPlannerConfig(config)
%normalizeJointPlannerConfig Fill and validate common Phase 12 config.

if nargin < 1 || isempty(config), config = struct(); end
hasStart = isfield(config,'start') && ~isempty(config.start);
hasGoal = isfield(config,'goal') && ~isempty(config.goal);
if ~isstruct(config) || ~isscalar(config)
    error('config must be a scalar structure.');
end
defaults = defaultJointPlannerConfig();
names = fieldnames(defaults);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(config,name) || isempty(config.(name))
        config.(name) = defaults.(name);
    end
end

if isfield(config,'scene') && ~isempty(config.scene)
    scene = config.scene;
    if isempty(config.robot) && isfield(scene,'robot'), config.robot=scene.robot; end
    if isempty(config.environment) && isfield(scene,'environment'), config.environment=scene.environment; end
    if (isempty(config.selfCollisionSkipPairs) || ...
            isequal(config.selfCollisionSkipPairs,{})) && ...
            isfield(scene,'selfCollisionSkipPairs')
        config.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
    end
    if ~hasStart && isfield(scene,'qMat0'), config.start=scene.qMat0; end
    if isfield(scene,'environmentCollisionPolicy')
        config.environmentCollisionPolicy=scene.environmentCollisionPolicy;
    end
end
if isfield(config,'qStart') && ~isempty(config.qStart), config.start=config.qStart; end
if isfield(config,'qGoal') && ~isempty(config.qGoal), config.goal=config.qGoal; end
if ~hasGoal && isfield(config,'goalState') && ~isempty(config.goalState)
    config.goal=config.goalState;
end

config.start = reshape(double(config.start),1,[]);
config.goal = reshape(double(config.goal),1,[]);
if numel(config.start) ~= 6 || numel(config.goal) ~= 6 || ...
        any(~isfinite([config.start config.goal]))
    error('config.start and config.goal must be finite 1-by-6 rad states.');
end
if isempty(config.robot) || isempty(config.environment)
    error('config.robot and config.environment are required.');
end
if isempty(config.jointLimits), config.jointLimits=deriveUR5JointLimits(config.robot); end
config.jointLimits=double(config.jointLimits);
if ~isequal(size(config.jointLimits),[6 2])
    error('jointLimits must be 6-by-2.');
end
if ~jointWithinLimits(config.start,config.jointLimits) || ...
        ~jointWithinLimits(config.goal,config.jointLimits)
    error('Start/goal violates UR5 joint limits.');
end

config.jointWeights=reshape(double(config.jointWeights),1,[]);
if numel(config.jointWeights)~=6 || any(~isfinite(config.jointWeights)) || ...
        any(config.jointWeights<0)
    error('jointWeights must be a non-negative 1-by-6 vector.');
end
validatePositive(config.maxJointStepRad,'maxJointStepRad');
validatePositive(config.jointEdgeResolutionRad,'jointEdgeResolutionRad');
validatePositiveInteger(config.maxIterations,'maxIterations');
if ~isscalar(config.maxPlanningTimeSec) || ...
        (~isfinite(config.maxPlanningTimeSec) && ~isinf(config.maxPlanningTimeSec)) || ...
        config.maxPlanningTimeSec<=0
    error('maxPlanningTimeSec must be positive or Inf.');
end
validatePositiveInteger(config.seed,'seed',true);
validateRange(config.goalBias,0,1,'goalBias');
config.anytime=logical(config.anytime);
config.continueAfterFirstSolution=logical(config.continueAfterFirstSolution);
config.checkpointTimesSec=reshape(double(config.checkpointTimesSec),1,[]);
if any(~isfinite(config.checkpointTimesSec)) || any(config.checkpointTimesSec<=0) || ...
        any(diff(config.checkpointTimesSec)<=0)
    error('checkpointTimesSec must be a strictly increasing positive vector.');
end
validatePositive(config.goalThresholdRad,'goalThresholdRad');
validatePositive(config.nearGamma,'nearGamma');
validatePositive(config.nearRadiusMaxRad,'nearRadiusMaxRad');
validatePositiveInteger(config.offsetAttempts,'offsetAttempts');
validatePositive(config.offsetRadiusScale,'offsetRadiusScale');
validatePositiveInteger(config.backtrackDepth,'backtrackDepth',true);
if ~isempty(config.candidateValidatorFcn) && ~isa(config.candidateValidatorFcn,'function_handle')
    error('candidateValidatorFcn must be empty or a function handle.');
end
config.rewire=logical(config.rewire);
config.classificationGoalTree=logical(config.classificationGoalTree);
config.useOffset=logical(config.useOffset);
config.useBacktracking=logical(config.useBacktracking);
config.useSequentialRewire=logical(config.useSequentialRewire);
end

function validatePositive(x,name)
if ~isscalar(x) || ~isfinite(x) || x<=0, error('%s must be positive.',name); end
end
function validatePositiveInteger(x,name,allowZero)
if nargin<3, allowZero=false; end
if ~isscalar(x) || ~isfinite(x) || x~=floor(x) || ...
        (allowZero && x<0) || (~allowZero && x<1)
    error('%s must be a valid integer.',name);
end
end
function validateRange(x,lo,hi,name)
if ~isscalar(x) || ~isfinite(x) || x<lo || x>hi, error('%s out of range.',name); end
end
