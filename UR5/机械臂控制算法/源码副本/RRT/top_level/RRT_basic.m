function result = RRT_basic(config)
% RRT_basic  Unified-config launcher for the original Basic RRT.

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
end

result = RRT_basic_impl(config);

end
