function report = profile_weld_actual_axis_continuation_reduced()
%PROFILE_WELD_ACTUAL_AXIS_CONTINUATION_REDUCED
% Cold-start benchmark for the reduced beam=1 continuation configuration.
% This is independent of all previous qPath/state-graph results.

here = fileparts(mfilename("fullpath"));
outputFile = "weld_actual_axis_replan_continuation_reduced.mat";
opts = struct( ...
    "animate",false,"publish",false,"enableFallback",false, ...
    "beamWidth",1,"firstPointMaxStates",2,"maxPoseSeeds",3, ...
    "maxPoseNeighbors",9,"numRandomSeedsFirstPoint",3, ...
    "coarseTravelDeg",0,"coarseRollDeg",0:45:315, ...
    "outputFile",outputFile);

t = tic;
result = replan_weld_actual_axis_continuation(opts);
wallTimeSec = toc(t);

layer = result.layerStats;
layerTime = [layer.elapsedSec];
layerIkTime = [layer.ikElapsedSec];
[~,order] = sort(layerTime,"descend");
topCount = min(10,numel(order));

report = struct();
report.created = char(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
report.options = opts;
report.wallTimeSec = wallTimeSec;
report.algorithmTimeSec = result.algorithmElapsedSec;
report.ikElapsedSec = result.continuationStats.ikElapsedSec;
report.ikCalls = result.continuationStats.ikCalls;
report.resultFile = result.outputFile;
report.layerStats = layer;
% layer i is the continuation/edge expansion into weld point i.  For
% i>=2 it is therefore the timing record for segment (i-1)->i.
segmentStats = repmat(struct("segmentIndex",0,"fromPoint",0,"toPoint",0, ...
    "elapsedSec",0,"ikElapsedSec",0,"ikCalls",0,"edgeTests",0),1,max(0,numel(layer)-1));
for i=2:numel(layer)
    segmentStats(i-1).segmentIndex = i-1;
    segmentStats(i-1).fromPoint = i-1;
    segmentStats(i-1).toPoint = i;
    segmentStats(i-1).elapsedSec = layer(i).elapsedSec;
    segmentStats(i-1).ikElapsedSec = layer(i).ikElapsedSec;
    segmentStats(i-1).ikCalls = layer(i).ikCalls;
    segmentStats(i-1).edgeTests = layer(i).edgeTests;
end
report.segmentStats = segmentStats;
report.topLayerIndices = order(1:topCount);
report.topLayerTimeSec = layerTime(order(1:topCount));
report.topLayerIkTimeSec = layerIkTime(order(1:topCount));
report.result = result;

save(fullfile(here,"weld_actual_axis_continuation_reduced_profile.mat"), ...
    "report","-v7.3");

reportText = fullfile(here,"weld_actual_axis_continuation_reduced_profile.txt");
fid = fopen(reportText,"w");
if fid >= 0
    fprintf(fid,"Reduced continuation cold-start profile\n");
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
    fprintf(fid,"\nTop segment times (segment i means point i -> i+1):\n");
    fprintf(fid,"segment,elapsed_s,ik_elapsed_s,ik_calls,edge_tests\n");
    for k=1:topCount
        i=order(k);
        if i==1, continue; end
        s=segmentStats(i-1);
        fprintf(fid,"%d,%.6f,%.6f,%d,%d\n",s.segmentIndex,s.elapsedSec, ...
            s.ikElapsedSec,s.ikCalls,s.edgeTests);
    end
    fclose(fid);
end

fprintf("\nReduced continuation profile complete.\n");
fprintf("Wall time       : %.3f s\n",wallTimeSec);
fprintf("Algorithm time  : %.3f s\n",result.algorithmElapsedSec);
fprintf("IK time         : %.3f s\n",result.continuationStats.ikElapsedSec);
fprintf("IK calls        : %d\n",result.continuationStats.ikCalls);
fprintf("PASS             : %d\n",result.complete);
fprintf("Saved profile    : %s\n",fullfile(here,"weld_actual_axis_continuation_reduced_profile.mat"));
fprintf("Saved timing     : %s\n",reportText);
end
