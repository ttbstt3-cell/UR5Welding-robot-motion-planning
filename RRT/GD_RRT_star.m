function result = GD_RRT_star(config)
% GD_RRT_star  Direct project-root launcher for GD-RRT*.
%
% Run without arguments for the paper-literal animated demo:
%     >> GD_RRT_star
%
% Pass a configuration structure for paper or extended benchmark mode:
%     result = GD_RRT_star(config)

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    config.gdPaperMode = true;
    config.optimizeIterations = 0;
end

result = GD_RRT_star_impl(config);

end
