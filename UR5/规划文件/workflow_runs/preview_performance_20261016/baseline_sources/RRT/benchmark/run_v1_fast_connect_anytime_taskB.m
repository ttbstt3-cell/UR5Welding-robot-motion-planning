function report = run_v1_fast_connect_anytime_taskB(options)
%run_v1_fast_connect_anytime_taskB Run CBi-RRT* on Seam A Task B.
%
% Task B is the frozen free-space transition qPostWeld -> qHome.  The
% deterministic local Point80 -> PostWeld connection is kept outside this
% planner comparison, exactly as in the original welding task.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'task','B', ...
    'outputFile',fullfile(root,'results','v1_fast_connect_anytime_taskB_seed20260917.mat'), ...
    'csvFile',fullfile(root,'results','v1_fast_connect_anytime_taskB_seed20260917.csv'), ...
    'eventsCsvFile',fullfile(root,'results','v1_fast_connect_anytime_taskB_seed20260917_improvement_events.csv'), ...
    'figureFile',fullfile(root,'results','v1_fast_connect_anytime_taskB_time_quality.png'));
names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(options,names{i}) && ~isempty(options.(names{i}))
        defaults.(names{i})=options.(names{i});
    end
end
% Preserve runner options such as maxPlanningTimeSec/checkpointTimesSec.
% The wrapper only supplies Task-B defaults; all other caller options must
% pass through unchanged to the shared Task-A/Task-B runner.
extraNames=fieldnames(options);
for i=1:numel(extraNames)
    if ~isfield(defaults,extraNames{i}) || ~isempty(options.(extraNames{i}))
        defaults.(extraNames{i})=options.(extraNames{i});
    end
end
report=run_v1_fast_connect_anytime_taskA(defaults);
end
