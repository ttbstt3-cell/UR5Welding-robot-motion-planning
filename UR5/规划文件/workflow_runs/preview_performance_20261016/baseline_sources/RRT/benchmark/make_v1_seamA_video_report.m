function outputFile = make_v1_seamA_video_report(outputFile)
%make_v1_seamA_video_report Adapt the separate CBi-RRT* task reports for replay.
%
% The animation script expects one report containing approach and exit paths
% in report.paths{segment,algorithm}.  Planning remains untouched; this file
% only packages the already saved Task A and Task B qPaths for visualization.

root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1 || isempty(outputFile)
    outputFile=fullfile(root,'results','v1_seamA_video_report.mat');
end
taskA=load(fullfile(root,'results', ...
    'v1_fast_connect_anytime_taskA_seed20260917.mat'),'report');
taskB=load(fullfile(root,'results', ...
    'v1_fast_connect_anytime_taskB_seed20260917.mat'),'report');
taskA=taskA.report; taskB=taskB.report;
if ~isequal(string(taskA.algorithmLabels(:)),string(taskB.algorithmLabels(:)))
    error('Task A and Task B algorithm labels do not match.');
end
videoReport=struct();
videoReport.algorithmLabels=taskA.algorithmLabels;
videoReport.paths=cell(2,numel(taskA.algorithmLabels));
videoReport.paths(1,:)=reshape(taskA.paths,1,[]);
videoReport.paths(2,:)=reshape(taskB.paths,1,[]);
videoReport.taskA=taskA.task;
videoReport.taskB=taskB.task;
videoReport.sourceTaskAFile=fullfile(root,'results', ...
    'v1_fast_connect_anytime_taskA_seed20260917.mat');
videoReport.sourceTaskBFile=fullfile(root,'results', ...
    'v1_fast_connect_anytime_taskB_seed20260917.mat');
videoReport.note="Visualization-only adapter; no replanning.";
report=videoReport; %#ok<NASGU>
save(outputFile,'report','-v7.3');
fprintf('CBi-RRT* video report saved: %s\n',outputFile);
end
