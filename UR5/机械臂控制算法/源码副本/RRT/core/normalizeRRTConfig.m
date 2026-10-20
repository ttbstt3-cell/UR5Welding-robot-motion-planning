function config = normalizeRRTConfig(config)
% normalizeRRTConfig  Fill defaults and validate a planner configuration.

defaults = defaultRRTConfig();

if nargin < 1 || isempty(config)
    config = struct();
end

if ~isstruct(config) || ~isscalar(config)
    error('config must be a scalar structure.');
end

defaultNames = fieldnames(defaults);
for i = 1:numel(defaultNames)
    name = defaultNames{i};
    if ~isfield(config,name) || isempty(config.(name))
        % An explicitly supplied empty obstacle matrix means an obstacle-free
        % map.  Other empty fields continue to receive shared defaults.
        if strcmp(name,'obstacles') && isfield(config,name)
            config.(name) = zeros(0,4);
        else
            config.(name) = defaults.(name);
        end
    end
end

config.start = reshape(config.start,1,[]);
config.goal = reshape(config.goal,1,[]);

if numel(config.start) ~= 2 || numel(config.goal) ~= 2 || ...
        any(~isfinite([config.start config.goal]))
    error('config.start and config.goal must be finite 1-by-2 points.');
end

if isempty(config.obstacles)
    config.obstacles = zeros(0,4);
end

if size(config.obstacles,2) ~= 4 || any(~isfinite(config.obstacles(:))) || ...
        any(config.obstacles(:,3:4) < 0,'all')
    error('config.obstacles must be an N-by-4 finite matrix [x y width height].');
end

if ~isnumeric(config.mapBounds) || numel(config.mapBounds) ~= 4
    error('config.mapBounds must be [xmin xmax ymin ymax].');
end

config.mapBounds = reshape(config.mapBounds,1,4);
if any(~isfinite(config.mapBounds)) || ...
        config.mapBounds(1) >= config.mapBounds(2) || ...
        config.mapBounds(3) >= config.mapBounds(4)
    error('config.mapBounds must contain increasing finite limits.');
end

validateNonnegativeScalar(config.clearance,'config.clearance');
validatePositiveScalar(config.stepSize,'config.stepSize');
validatePositiveInteger(config.maxIterations,'config.maxIterations');
validateNonnegativeInteger(config.optimizeIterations,'config.optimizeIterations');
validateNonnegativeScalar(config.goalThreshold,'config.goalThreshold');
validatePositiveInteger(config.qDepth,'config.qDepth');
if ~isscalar(config.gdPaperMode) || ...
        ~(islogical(config.gdPaperMode) || isnumeric(config.gdPaperMode))
    error('config.gdPaperMode must be a scalar logical-like value.');
end
config.gdPaperMode = logical(config.gdPaperMode);

if isstring(config.gdSamplingInterpretation) && ...
        isscalar(config.gdSamplingInterpretation)
    config.gdSamplingInterpretation = char(config.gdSamplingInterpretation);
end
if ~ischar(config.gdSamplingInterpretation) || ...
        ~isrow(config.gdSamplingInterpretation) || ...
        ~strcmp(config.gdSamplingInterpretation,'paper_literal')
    error(['config.gdSamplingInterpretation must be ', ...
        '''paper_literal'' in Phase 7.']);
end
validatePositiveInteger(config.gdGaussianMaxRetries, ...
    'config.gdGaussianMaxRetries');

validateNonnegativeInteger(config.crrtBacktrackDepth, ...
    'config.crrtBacktrackDepth');
if config.crrtBacktrackDepth > 4
    error('config.crrtBacktrackDepth must be in the range 0 to 4.');
end
validatePositiveInteger(config.crrtOffsetAttempts, ...
    'config.crrtOffsetAttempts');
validatePositiveInteger(config.crrtPaperTotalIterations, ...
    'config.crrtPaperTotalIterations');
if ~isscalar(config.crrtPaperMode) || ...
        ~(islogical(config.crrtPaperMode) || isnumeric(config.crrtPaperMode))
    error('config.crrtPaperMode must be a scalar logical-like value.');
end
config.crrtPaperMode = logical(config.crrtPaperMode);
if ~isscalar(config.crrtStoreOffsetPoints) || ...
        ~(islogical(config.crrtStoreOffsetPoints) || ...
        isnumeric(config.crrtStoreOffsetPoints))
    error('config.crrtStoreOffsetPoints must be a scalar logical-like value.');
end
config.crrtStoreOffsetPoints = logical(config.crrtStoreOffsetPoints);

if ~isscalar(config.apfPaperMode) || ...
        ~(islogical(config.apfPaperMode) || isnumeric(config.apfPaperMode))
    error('config.apfPaperMode must be a scalar logical-like value.');
end
config.apfPaperMode = logical(config.apfPaperMode);
validateNonnegativeScalar(config.apfGoalBiasMin,'config.apfGoalBiasMin');
validateNonnegativeScalar(config.apfGoalBiasMax,'config.apfGoalBiasMax');
if config.apfGoalBiasMin > config.apfGoalBiasMax || ...
        config.apfGoalBiasMax > 1
    error('APF goal-bias limits must satisfy 0 <= min <= max <= 1.');
end
validatePositiveScalar(config.apfGoalBiasExponent, ...
    'config.apfGoalBiasExponent');
validatePositiveScalar(config.apfKattMin,'config.apfKattMin');
validateNonnegativeScalar(config.apfLambda,'config.apfLambda');
validatePositiveScalar(config.apfKrep,'config.apfKrep');
validatePositiveScalar(config.apfInfluenceDistance, ...
    'config.apfInfluenceDistance');
validatePositiveScalar(config.apfRepulsiveExponent, ...
    'config.apfRepulsiveExponent');
validatePositiveScalar(config.apfForceTolerance,'config.apfForceTolerance');
if isstring(config.apfPaperConnectionMode) && ...
        isscalar(config.apfPaperConnectionMode)
    config.apfPaperConnectionMode = char(config.apfPaperConnectionMode);
end
if ~ischar(config.apfPaperConnectionMode) || ...
        ~isrow(config.apfPaperConnectionMode) || ...
        ~(strcmp(config.apfPaperConnectionMode,'nearest') || ...
        strcmp(config.apfPaperConnectionMode,'best_near'))
    error('config.apfPaperConnectionMode must be nearest or best_near.');
end
validatePositiveInteger(config.bsplineDegree,'config.bsplineDegree');
validatePositiveInteger(config.bsplineSamples,'config.bsplineSamples');

if ~isscalar(config.seed) || ~isfinite(config.seed) || ~isnumeric(config.seed) || ...
        config.seed < 0 || config.seed ~= floor(config.seed)
    error('config.seed must be a finite non-negative integer.');
end

if ~isscalar(config.showAnimation)
    error('config.showAnimation must be scalar logical-like value.');
end

validateNonnegativeScalar(config.animationPause,'config.animationPause');

if ~isscalar(config.goalBias) || ~isfinite(config.goalBias) || ...
        config.goalBias < 0 || config.goalBias > 1
    error('config.goalBias must be in [0, 1].');
end

validatePositiveScalar(config.nearGamma,'config.nearGamma');
validatePositiveScalar(config.nearRadiusMax,'config.nearRadiusMax');

end

function validatePositiveScalar(value,name)
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('%s must be a finite positive scalar.',name);
end
end

function validateNonnegativeScalar(value,name)
if ~isscalar(value) || ~isfinite(value) || value < 0
    error('%s must be a finite non-negative scalar.',name);
end
end

function validatePositiveInteger(value,name)
if ~isscalar(value) || ~isfinite(value) || value < 1 || value ~= floor(value)
    error('%s must be a positive integer.',name);
end
end

function validateNonnegativeInteger(value,name)
if ~isscalar(value) || ~isfinite(value) || value < 0 || value ~= floor(value)
    error('%s must be a non-negative integer.',name);
end
end
