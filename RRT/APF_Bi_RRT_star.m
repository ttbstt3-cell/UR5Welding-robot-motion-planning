function result = APF_Bi_RRT_star(config)
% APF_Bi_RRT_star  Direct project-root launcher for improved APF-Bi-RRT*.
%
% No arguments run the paper-mode animated first-connection demo.  Pass a
% configuration structure for paper total-horizon or extended benchmark
% experiments.

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'core'));
addpath(fullfile(projectRoot,'planners'));

if nargin < 1 || isempty(config)
    config = defaultRRTConfig();
    config.showAnimation = true;
    config.apfPaperMode = true;
    config.optimizeIterations = 0;
    % The shared 2-D demo map contains long wall-like obstacles. A smaller
    % APF step lets the force field slide around those walls in the direct
    % animation demo. Paper-scale experiments should set stepSize=8
    % explicitly together with a paper-scale map.
    config.stepSize = 1;
end

result = APF_Bi_RRT_star_impl(config);

end
