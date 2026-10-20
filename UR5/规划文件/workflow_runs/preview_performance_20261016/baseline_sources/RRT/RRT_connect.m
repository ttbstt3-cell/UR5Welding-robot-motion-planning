function result = RRT_connect(config)
% RRT_connect  Direct project-root launcher for RRT-Connect.
%
% Run without arguments for an animated demo:
%     >> RRT_connect
%
% Pass a configuration structure for benchmark mode:
%     result = RRT_connect(config)

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
end

result = RRT_connect_impl(config);

end
