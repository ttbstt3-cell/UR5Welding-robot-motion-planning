function result = Bi_RRT_star(config)
% Bi_RRT_star  Direct project-root launcher for Bi-RRT*.
%
% Run without arguments for an animated first-solution demo:
%     >> Bi_RRT_star
%
% Pass a configuration structure for benchmark mode:
%     result = Bi_RRT_star(config)

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    config.optimizeIterations = 0;
end

result = Bi_RRT_star_impl(config);

end
