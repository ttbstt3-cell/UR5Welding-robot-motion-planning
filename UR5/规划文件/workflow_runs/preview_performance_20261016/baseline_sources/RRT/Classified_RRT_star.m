function result = Classified_RRT_star(config)
% Classified_RRT_star  Direct project-root launcher for C-RRT*.
%
% Run without arguments for an animated first-solution demo.  Set
% config.crrtPaperMode=true for the paper's fixed total-iteration study, or
% pass a normal configuration for the project's extended benchmark mode.

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    config.crrtPaperMode = false;
    config.optimizeIterations = 0;
end

result = Classified_RRT_star_impl(config);

end
