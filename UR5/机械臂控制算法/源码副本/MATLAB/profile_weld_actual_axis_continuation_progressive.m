function report = profile_weld_actual_axis_continuation_progressive()
%PROFILE_WELD_ACTUAL_AXIS_CONTINUATION_PROGRESSIVE
% Paper-inspired adaptive local pose expansion benchmark.
% Normal layers use five pose neighbors; a failed layer is expanded to nine.
% This is a cold-start run and does not load a previous qPath/state graph.

here = fileparts(mfilename("fullpath"));
outputFile = "weld_actual_axis_replan_continuation_progressive.mat";
opts = struct( ...
    "animate",false,"publish",false,"enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "maxPoseNeighbors",9,"numRandomSeedsFirstPoint",3, ...
    "coarseTravelDeg",0,"coarseRollDeg",0:45:315, ...
    "progressivePoseNeighbors",true, ...
    "progressiveInitialPoseNeighbors",5, ...
    "progressiveMaxPoseNeighbors",9, ...
    "outputFile",outputFile);

t = tic;
result = replan_weld_actual_axis_continuation(opts);
wallTimeSec = toc(t);

layer = result.layerStats;
layerTime = [layer.elapsedSec];
[~,order] = sort(layerTime,"descend");
topCount = min(10,numel(order));
expanded = find(([layer.ikCalls] > opts.maxPoseNeighbors*opts.maxPoseSeeds) & ...
    ((1:numel(layer)) > 1));

report = struct();
report.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
report.options = opts;
report.wallTimeSec = wallTimeSec;
report.algorithmTimeSec = result.algorithmElapsedSec;
report.ikElapsedSec = result.continuationStats.ikElapsedSec;
report.ikCalls = result.continuationStats.ikCalls;
report.resultFile = result.outputFile;
report.expandedLayerIndices = expanded;
report.layerStats = layer;
report.topLayerIndices = order(1:topCount);
report.topLayerTimeSec = layerTime(order(1:topCount));
report.result = result;
save(fullfile(here,"weld_actual_axis_continuation_progressive_profile.mat"), ...
    "report","-v7.3");

reportText = fullfile(here,"weld_actual_axis_continuation_progressive_profile.txt");
fid = fopen(reportText,"w");
if fid >= 0
    fprintf(fid,"Progressive local pose expansion cold-start profile\n");
    fprintf(fid,"Created: %s\n\n",report.created);
    fprintf(fid,"Algorithm time (s): %.6f\n",report.algorithmTimeSec);
    fprintf(fid,"Wall time (s): %.6f\n",report.wallTimeSec);
    fprintf(fid,"IK time (s): %.6f\n",report.ikElapsedSec);
    fprintf(fid,"IK calls: %d\n",report.ikCalls);
    fprintf(fid,"Complete: %d\n",result.complete);
    fprintf(fid,"Node collision free: %d\n",result.nodeCollisionFree);
    fprintf(fid,"Edge collision free: %d\n",result.edgeCollisionFree);
    fprintf(fid,"Max joint jump (deg): %.6f\n",result.maxJointJumpDeg);
    fprintf(fid,"Max node TCP error (mm): %.6f\n",result.tcpMetrics.maxNodePositionErrorMm);
    fprintf(fid,"Max segment TCP error (mm): %.6f\n",result.tcpMetrics.maxSegmentPositionErrorMm);
    fprintf(fid,"Max segment TCP orientation (deg): %.6f\n", ...
        result.tcpMetrics.maxSegmentOrientationErrorDeg);
    fprintf(fid,"Expanded layer indices: ");
    fprintf(fid,"%d ",expanded); fprintf(fid,"\n\n");
    fprintf(fid,"Top layer timings:\n");
    fprintf(fid,"layer,elapsed_s,ik_elapsed_s,ik_calls,edge_tests\n");
    for k=1:topCount
        i=order(k);
        fprintf(fid,"%d,%.6f,%.6f,%d,%d\n",i,layer(i).elapsedSec, ...
            layer(i).ikElapsedSec,layer(i).ikCalls,layer(i).edgeTests);
    end
    fclose(fid);
end

fprintf("\nProgressive continuation profile complete.\n");
fprintf("Wall time       : %.3f s\n",wallTimeSec);
fprintf("Algorithm time  : %.3f s\n",result.algorithmElapsedSec);
fprintf("IK time         : %.3f s\n",result.continuationStats.ikElapsedSec);
fprintf("IK calls        : %d\n",result.continuationStats.ikCalls);
fprintf("Expanded layers : "); fprintf("%d ",expanded); fprintf("\n");
fprintf("PASS             : %d\n",result.complete);
fprintf("Saved profile    : %s\n",fullfile(here,"weld_actual_axis_continuation_progressive_profile.mat"));
fprintf("Saved timing     : %s\n",reportText);
end
