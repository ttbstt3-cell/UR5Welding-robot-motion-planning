function report = profile_weld_actual_axis_baseline()
%PROFILE_WELD_ACTUAL_AXIS_BASELINE Profile the existing 80-point baseline.
% This does not publish ROS commands and does not overwrite the validated
% weld result. It uses the same algorithm options, but disables animation.

here = fileparts(mfilename("fullpath"));
profileFile = fullfile(here,"weld_actual_axis_baseline_profile.mat");

% Keep all planning defaults unchanged; only suppress side effects that are
% irrelevant to timing diagnosis.
options = struct( ...
    "animate",false, ...
    "publish",false, ...
    "outputFile","weld_actual_axis_replan_profile.mat");

fprintf("Starting MATLAB Profiler for existing 80-point baseline...\n");
fprintf("No ROS publish and no CoppeliaSim command will be issued.\n");

profile clear;
profile on -history;
wallTimer = tic;
result = replan_weld_actual_axis(options);
wallTimeSec = toc(wallTimer);
profile off;

profileData = profile("info");
report = struct();
report.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
report.wallTimeSec = wallTimeSec;
report.algorithmTimeSec = result.algorithmElapsedSec;
report.resultFile = result.outputFile;
report.profileData = profileData;
report.stageSummary = localSummarizeResult(result);
save(profileFile,"report","-v7.3");

fprintf("\nProfiler complete.\n");
fprintf("Wall time       : %.3f s\n",wallTimeSec);
fprintf("Algorithm time  : %.3f s\n",result.algorithmElapsedSec);
fprintf("Saved profile   : %s\n",profileFile);
fprintf("Saved result    : %s\n",result.outputFile);
fprintf("State count     : %d\n",sum(cellfun(@numel,result.stateBank)));
fprintf("Graph edge tests: %d\n",result.graph.edgeTests);
end

function summary = localSummarizeResult(result)
summary = struct();
summary.numPoints = result.seam.numPoints;
summary.coarsePoseCount = result.seam.numPoints * ...
    numel(result.options.coarseTravelDeg) * ...
    numel(result.options.coarseRollDeg);
summary.finePoseCountUpperBound = result.options.maxRefineRounds * 6 * ...
    numel(result.options.fineTravelDeg) * ...
    numel(result.options.fineRollDeg);
summary.stateCount = sum(cellfun(@numel,result.stateBank));
summary.graphEdgeTests = result.graph.edgeTests;
summary.graphCollisionFreeEdges = result.graph.collisionFreeEdges;
summary.complete = result.complete;
end
