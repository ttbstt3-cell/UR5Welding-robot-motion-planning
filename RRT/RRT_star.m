function result = RRT_star(config)
% RRT_star  Direct project-root launcher for RRT*.
%
% Run without arguments for an animated demo:
%     >> RRT_star
%
% Pass a configuration structure for benchmark mode:
%     result = RRT_star(config)

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    % Direct interactive runs stop at the first feasible path.  Benchmark
    % runs can pass a config with optimizeIterations > 0 to continue the
    % required RRT* optimization phase.
    config.optimizeIterations = 0;
end

result = RRT_star_impl(config);

end
