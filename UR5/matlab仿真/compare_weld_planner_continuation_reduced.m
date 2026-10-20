function report = compare_weld_planner_continuation_reduced()
%COMPARE_WELD_PLANNER_CONTINUATION_REDUCED
% Compare the historical stable planner, the reduced cold-start continuation,
% and its adaptive TCP execution-waypoint correction.

here = fileparts(mfilename("fullpath"));
stable = load(fullfile(here,"weld_actual_axis_replan_fast.mat")).result;
reduced = load(fullfile(here,"weld_actual_axis_replan_continuation_reduced.mat")).result;
adaptive = load(fullfile(here,"weld_actual_axis_replan_adaptive_reduced.mat")).result;
profile = load(fullfile(here,"weld_actual_axis_continuation_reduced_profile.mat")).report;

report = struct();
report.stableAlgorithmSec = stable.algorithmElapsedSec;
report.reducedAlgorithmSec = reduced.algorithmElapsedSec;
report.adaptiveAlgorithmSec = adaptive.algorithmElapsedSec;
report.reducedWallSec = profile.wallTimeSec;
report.adaptiveWallSec = adaptive.totalPlanningTimeSec;
report.reducedSpeedupVsStable = 100*(1-report.reducedAlgorithmSec/report.stableAlgorithmSec);
report.reduced = reduced;
report.adaptive = adaptive;

fprintf("\n========================================\n");
fprintf("REDUCED CONTINUATION COMPARISON\n");
fprintf("========================================\n");
fprintf("                         stable    reduced    adaptive\n");
fprintf("Algorithm time (s)       %8.3f  %8.3f  %8.3f\n", ...
    stable.algorithmElapsedSec,reduced.algorithmElapsedSec,adaptive.algorithmElapsedSec);
fprintf("Wall time (s)            %8s  %8.3f  %8.3f\n","n/a",report.reducedWallSec,report.adaptiveWallSec);
fprintf("IK time (s)              %8s  %8.3f  %8.3f\n","n/a", ...
    reduced.continuationStats.ikElapsedSec,adaptive.continuationStats.ikElapsedSec);
fprintf("IK calls                 %8s  %8d  %8d\n","n/a", ...
    reduced.continuationStats.ikCalls,adaptive.continuationStats.ikCalls);
fprintf("Complete path            %8d  %8d  %8d\n", ...
    stable.complete,reduced.complete,adaptive.complete);
fprintf("Node collision free      %8d  %8d  %8d\n", ...
    stable.nodeCollisionFree,reduced.nodeCollisionFree,adaptive.nodeCollisionFree);
fprintf("Edge collision free      %8d  %8d  %8d\n", ...
    stable.edgeCollisionFree,reduced.edgeCollisionFree,adaptive.edgeCollisionFree);
stableSeg = localStableSegmentTcpMax(stable,3);
fprintf("Segment TCP max (mm)     %8.3f  %8.3f  %8.3f\n", ...
    stableSeg, ...
    reduced.tcpMetrics.maxSegmentPositionErrorMm, ...
    max(adaptive.finalTcpMetrics.segmentPositionErrorMm));
fprintf("========================================\n");
end

function maxMm = localStableSegmentTcpMax(r,edgeResolutionDeg)
if isfield(r,"tcpMetrics") && isfield(r.tcpMetrics,"maxSegmentPositionErrorMm")
    maxMm = r.tcpMetrics.maxSegmentPositionErrorMm;
    return;
end
q = r.qPath; N=size(q,1); Ttar=cell(N,1);
for i=1:N
    s=r.stateBank{i}; Q=vertcat(s.q);
    D=rad2deg(atan2(sin(Q-q(i,:)),cos(Q-q(i,:))));
    [~,j]=min(vecnorm(D,2,2)); Ttar{i}=s(j).Ttarget;
end
err=zeros(N-1,1);
for i=1:N-1
    dq=atan2(sin(q(i+1,:)-q(i,:)),cos(q(i+1,:)-q(i,:)));
    n=max(2,ceil(max(abs(rad2deg(dq)))/edgeResolutionDeg)+1);
    for k=0:n-1
        s=k/(n-1); qq=q(i,:)+s*dq;
        T=getTransform(r.scene.robot,qq,r.scene.tcpBodyName);
        p=(1-s)*Ttar{i}(1:3,4)+s*Ttar{i+1}(1:3,4);
        err(i)=max(err(i),norm(T(1:3,4)-p));
    end
end
maxMm=max(err)*1000;
end
