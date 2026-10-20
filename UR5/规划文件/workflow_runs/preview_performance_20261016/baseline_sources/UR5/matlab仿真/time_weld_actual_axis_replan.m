function timing = time_weld_actual_axis_replan()
%TIME_WELD_ACTUAL_AXIS_REPLAN Measure a fresh 80-point weld replanning run.
%
% This is a read-only timing wrapper around replan_weld_actual_axis.  It
% disables animation and ROS2 publishing, writes the rerun result to a new
% MAT file, and never overwrites weld_actual_axis_replan.mat.

here = fileparts(mfilename("fullpath"));
timedResultName = "weld_actual_axis_replan_timed_20260917.mat";
timedReportName = "weld_actual_axis_replan_timing_report.txt";
timedResultFile = fullfile(here,timedResultName);
timedReportFile = fullfile(here,timedReportName);

fprintf("\nTiming a fresh measured-wire-axis weld planning run...\n");
fprintf("Animation: off | ROS2 publish: off\n");
fprintf("Existing weld_actual_axis_replan.mat will not be overwritten.\n\n");

result = replan_weld_actual_axis(struct( ...
    "animate",false, ...
    "publish",false, ...
    "outputFile",timedResultName));
if ~isfield(result,"algorithmElapsedSec")
    error("规划结果没有 algorithmElapsedSec；请确认 replan_weld_actual_axis.m 已更新。");
end
elapsedSec = result.algorithmElapsedSec;

timing = struct();
timing.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
timing.planningElapsedSec = elapsedSec;
timing.resultFile = result.outputFile;
timing.weldPoints = size(result.qPath,1);
timing.complete = result.complete;
timing.nodeCollisionFree = result.nodeCollisionFree;
timing.edgeCollisionFree = result.edgeCollisionFree;
timing.maxJointJumpDeg = result.maxJointJumpDeg;
timing.randomSeed = result.options.randomSeed;
timing.animation = false;
timing.publish = false;
timing.reportFile = timedReportFile;

save(fullfile(here,"weld_actual_axis_replan_timing.mat"),"timing");

fid = fopen(timedReportFile,"w");
if fid < 0
    error("无法写入计时报告：%s",timedReportFile);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,"Measured-wire-axis weld planning timing report\n");
fprintf(fid,"Created: %s\n",timing.created);
fprintf(fid,"Source algorithm: replan_weld_actual_axis.m\n");
fprintf(fid,"Existing result was not overwritten: 1\n");
fprintf(fid,"Animation: 0\n");
fprintf(fid,"ROS2 publish: 0\n");
fprintf(fid,"Random seed: %d\n",timing.randomSeed);
fprintf(fid,"Weld points: %d\n",timing.weldPoints);
fprintf(fid,"Planning elapsed time (s): %.6f\n",timing.planningElapsedSec);
fprintf(fid,"Complete: %d\n",timing.complete);
fprintf(fid,"Node collision free: %d\n",timing.nodeCollisionFree);
fprintf(fid,"Edge collision free: %d\n",timing.edgeCollisionFree);
fprintf(fid,"Max joint jump (deg): %.6f\n",timing.maxJointJumpDeg);
fprintf(fid,"Timed rerun result: %s\n",timing.resultFile);

fprintf("\nAlgorithm computation time (seam + IK + collision + graph; scene excluded): %.6f s\n",elapsedSec);
fprintf("Timed rerun complete=%d, node-free=%d, edge-free=%d\n", ...
    timing.complete,timing.nodeCollisionFree,timing.edgeCollisionFree);
fprintf("Saved timing MAT: %s\n",fullfile(here,"weld_actual_axis_replan_timing.mat"));
fprintf("Saved timing report: %s\n",timedReportFile);
fprintf("Saved timed rerun result: %s\n",timing.resultFile);
end
