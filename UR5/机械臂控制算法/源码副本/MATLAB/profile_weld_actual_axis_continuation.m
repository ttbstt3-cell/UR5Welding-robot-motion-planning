function report = profile_weld_actual_axis_continuation()
%PROFILE_WELD_ACTUAL_AXIS_CONTINUATION Fair cold-start continuation benchmark.
% No previous qPath or state graph is loaded.  No ROS2 command is published.

here = fileparts(mfilename("fullpath"));
outputFile = "weld_actual_axis_replan_continuation_beam1.mat";
opts = struct( ...
    "animate",false,"publish",false,"enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "outputFile",outputFile);

t = tic;
result = replan_weld_actual_axis_continuation(opts);
wallTimeSec = toc(t);

report = struct();
report.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
report.wallTimeSec = wallTimeSec;
report.algorithmTimeSec = result.algorithmElapsedSec;
report.ikElapsedSec = result.continuationStats.ikElapsedSec;
report.resultFile = result.outputFile;
report.result = result;
save(fullfile(here,"weld_actual_axis_continuation_profile.mat"),"report","-v7.3");

fprintf("\nContinuation profile complete.\n");
fprintf("Wall time       : %.3f s\n",wallTimeSec);
fprintf("Algorithm time  : %.3f s\n",result.algorithmElapsedSec);
fprintf("IK time         : %.3f s\n",result.continuationStats.ikElapsedSec);
fprintf("PASS             : %d\n",result.complete);
fprintf("Saved profile    : %s\n",fullfile(here,"weld_actual_axis_continuation_profile.mat"));
end
