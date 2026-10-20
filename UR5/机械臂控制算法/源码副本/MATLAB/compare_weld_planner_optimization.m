function report = compare_weld_planner_optimization()
%COMPARE_WELD_PLANNER_OPTIMIZATION Compare the validated weld planner runs.
% This is read-only: it loads saved MAT files and prints the common metrics.

here = fileparts(mfilename("fullpath"));
baselineFile = fullfile(here,"weld_actual_axis_replan_timed_20260917.mat");
fastFile = fullfile(here,"weld_actual_axis_replan_fast.mat");

baseline = load(baselineFile);
fast = load(fastFile);
rb = baseline.result;
rf = fast.result;

report = struct();
report.baselineFile = baselineFile;
report.fastFile = fastFile;
report.baselineAlgorithmSec = rb.algorithmElapsedSec;
report.fastAlgorithmSec = rf.algorithmElapsedSec;
report.speedupPercent = 100*(1-rf.algorithmElapsedSec/rb.algorithmElapsedSec);
report.baselineComplete = rb.complete;
report.fastComplete = rf.complete;
report.baselineStateCount = sum(cellfun(@numel,rb.stateBank));
report.fastStateCount = sum(cellfun(@numel,rf.stateBank));
report.baselineEdgeTests = rb.graph.edgeTests;
report.fastEdgeTests = rf.graph.edgeTests;
report.baselineCollisionFreeEdges = rb.graph.collisionFreeEdges;
report.fastCollisionFreeEdges = rf.graph.collisionFreeEdges;
report.baselineMaxJointJumpDeg = rb.maxJointJumpDeg;
report.fastMaxJointJumpDeg = rf.maxJointJumpDeg;
[report.baselineMaxPositionErrorMm,report.baselineMaxOrientationErrorDeg] = ...
    localMaxErrors(rb);
[report.fastMaxPositionErrorMm,report.fastMaxOrientationErrorDeg] = ...
    localMaxErrors(rf);

fprintf("\n========================================\n");
fprintf("WELD PLANNER OPTIMIZATION COMPARISON\n");
fprintf("========================================\n");
fprintf("Baseline: %s\n",baselineFile);
fprintf("Fast    : %s\n\n",fastFile);
fprintf("                         baseline       fast\n");
fprintf("Algorithm time (s)       %10.3f %10.3f\n", ...
    report.baselineAlgorithmSec,report.fastAlgorithmSec);
fprintf("Speedup                  %10.2f %%\n",report.speedupPercent);
fprintf("Complete path            %10d %10d\n", ...
    report.baselineComplete,report.fastComplete);
fprintf("Accepted states          %10d %10d\n", ...
    report.baselineStateCount,report.fastStateCount);
fprintf("Graph edge tests         %10d %10d\n", ...
    report.baselineEdgeTests,report.fastEdgeTests);
fprintf("Collision-free edges     %10d %10d\n", ...
    report.baselineCollisionFreeEdges,report.fastCollisionFreeEdges);
fprintf("Max joint jump (deg)     %10.3f %10.3f\n", ...
    report.baselineMaxJointJumpDeg,report.fastMaxJointJumpDeg);
fprintf("Max TCP position err(mm) %10.6f %10.6f\n", ...
    report.baselineMaxPositionErrorMm,report.fastMaxPositionErrorMm);
fprintf("Max TCP orient err(deg)  %10.6f %10.6f\n", ...
    report.baselineMaxOrientationErrorDeg,report.fastMaxOrientationErrorDeg);
fprintf("Node collision free      %10d %10d\n", ...
    rb.nodeCollisionFree,rf.nodeCollisionFree);
fprintf("Edge collision free      %10d %10d\n", ...
    rb.edgeCollisionFree,rf.edgeCollisionFree);
fprintf("========================================\n");

end

function [maxPosMm,maxOriDeg] = localMaxErrors(result)
pos = zeros(0,1);
ori = zeros(0,1);
for i = 1:numel(result.stateBank)
    states = result.stateBank{i};
    if isempty(states), continue; end
    pos = [pos; vertcat(states.positionErrorM)]; %#ok<AGROW>
    ori = [ori; vertcat(states.orientationErrorDeg)]; %#ok<AGROW>
end
if isempty(pos)
    maxPosMm = NaN;
else
    maxPosMm = max(pos)*1000;
end
if isempty(ori)
    maxOriDeg = NaN;
else
    maxOriDeg = max(ori);
end
end
