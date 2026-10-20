function result = Bi_RRT(config)
% Bi_RRT  Unified-config launcher for the original bidirectional RRT.

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
end

result = Bi_RRT_impl(config);

end
