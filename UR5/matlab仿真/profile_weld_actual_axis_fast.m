function report = profile_weld_actual_axis_fast()
%PROFILE_WELD_ACTUAL_AXIS_FAST Profile the optimized baseline variant.
% Reuses the IK solver and exact pose candidates, and defers collision
% checks until the layered graph reaches a node/edge. No ROS publish.

here = fileparts(mfilename("fullpath"));
profileFile = fullfile(here,"weld_actual_axis_fast_profile.mat");
% Continuation-seed reuse was benchmarked separately. It increases the
% number of retained IK states during fine refinement and can break the
% layered graph, so this stable fast profile keeps the original seed bank.
options = struct( ...
    "animate",false, ...
    "publish",false, ...
    "reuseIKSolver",true, ...
    "reuseCandidateCache",true, ...
    "reusePoseCandidates",false, ...
    "lazyCollision",true, ...
    "outputFile","weld_actual_axis_replan_fast.mat");

fprintf("Starting optimized 80-point baseline profile...\n");
fprintf("IK solver/cache reuse: ON; lazy collision: ON.\n");

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
report.stateCount = sum(cellfun(@numel,result.stateBank));
report.graphEdgeTests = result.graph.edgeTests;
report.graphCollisionFreeEdges = result.graph.collisionFreeEdges;
report.complete = result.complete;
save(profileFile,"report","-v7.3");

fprintf("\nOptimized profile complete.\n");
fprintf("Wall time       : %.3f s\n",wallTimeSec);
fprintf("Algorithm time  : %.3f s\n",result.algorithmElapsedSec);
fprintf("Saved profile   : %s\n",profileFile);
fprintf("Saved result    : %s\n",result.outputFile);
fprintf("State count     : %d\n",report.stateCount);
fprintf("Graph edge tests: %d\n",report.graphEdgeTests);
end
