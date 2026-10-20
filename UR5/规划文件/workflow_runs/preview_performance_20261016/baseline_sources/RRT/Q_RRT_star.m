function result = Q_RRT_star(config)
% Q_RRT_star  Direct project-root launcher for Quick-RRT*.
%
% Run without arguments for an animated first-solution demo:
%     >> Q_RRT_star
%
% Pass a configuration structure for benchmark mode:
%     result = Q_RRT_star(config)

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    config.optimizeIterations = 0;
end

result = Q_RRT_star_impl(config);

end
