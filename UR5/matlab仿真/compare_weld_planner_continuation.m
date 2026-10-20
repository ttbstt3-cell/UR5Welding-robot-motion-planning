function report = compare_weld_planner_continuation()
%COMPARE_WELD_PLANNER_CONTINUATION Compare 346 s and cold-start beam results.

here = fileparts(mfilename("fullpath"));
base = load(fullfile(here,"weld_actual_axis_replan_fast.mat"));
cont = load(fullfile(here,"weld_actual_axis_replan_continuation_beam1.mat"));
rb = base.result; rc = cont.result;

% The historical stable result predates tcpMetrics; compute its segment
% deviation from the saved state labels without loading a previous qPath as
% an input to the continuation planner.
baseTcp = localTcpMetrics(rb,3);
report = struct();
report.baselineAlgorithmSec = rb.algorithmElapsedSec;
report.continuationAlgorithmSec = rc.algorithmElapsedSec;
report.continuationIkSec = rc.continuationStats.ikElapsedSec;
report.speedupPercent = 100*(1-rc.algorithmElapsedSec/rb.algorithmElapsedSec);
report.baselineComplete = rb.complete;
report.continuationComplete = rc.complete;
report.baselineStateCount = sum(cellfun(@numel,rb.stateBank));
report.continuationStateCount = sum(cellfun(@numel,rc.stateBank));
report.baselineEdgeTests = rb.graph.edgeTests;
report.continuationEdgeTests = rc.graph.edgeTests;
report.baselineMaxJointJumpDeg = rb.maxJointJumpDeg;
report.continuationMaxJointJumpDeg = rc.maxJointJumpDeg;
report.baselineTcp = baseTcp;
report.continuationTcp = rc.tcpMetrics;

fprintf("\n========================================\n");
fprintf("WELD PLANNER CONTINUATION COMPARISON\n");
fprintf("========================================\n");
fprintf("                         stable 346 s   continuation\n");
fprintf("Algorithm time (s)       %10.3f %10.3f\n", ...
    rb.algorithmElapsedSec,rc.algorithmElapsedSec);
fprintf("Speedup                  %10.2f %%\n",report.speedupPercent);
fprintf("IK time (s)              %10s %10.3f\n","n/a",rc.continuationStats.ikElapsedSec);
fprintf("IK calls                 %10s %10d\n","n/a",rc.continuationStats.ikCalls);
fprintf("Complete path            %10d %10d\n",rb.complete,rc.complete);
fprintf("States                   %10d %10d\n", ...
    report.baselineStateCount,report.continuationStateCount);
fprintf("Graph edge tests         %10d %10d\n", ...
    rb.graph.edgeTests,rc.graph.edgeTests);
fprintf("Max joint jump (deg)     %10.3f %10.3f\n", ...
    rb.maxJointJumpDeg,rc.maxJointJumpDeg);
fprintf("Node TCP error (mm)      %10.6f %10.6f\n", ...
    baseTcp.maxNodePositionErrorMm,rc.tcpMetrics.maxNodePositionErrorMm);
fprintf("Segment TCP error (mm)   %10.6f %10.6f\n", ...
    baseTcp.maxSegmentPositionErrorMm,rc.tcpMetrics.maxSegmentPositionErrorMm);
fprintf("Segment TCP orient (deg) %10.6f %10.6f\n", ...
    baseTcp.maxSegmentOrientationErrorDeg,rc.tcpMetrics.maxSegmentOrientationErrorDeg);
fprintf("Node collision free      %10d %10d\n", ...
    rb.nodeCollisionFree,rc.nodeCollisionFree);
fprintf("Edge collision free      %10d %10d\n", ...
    rb.edgeCollisionFree,rc.edgeCollisionFree);
fprintf("========================================\n");
end

function metrics = localTcpMetrics(r,edgeResolutionDeg)
metrics = struct("maxNodePositionErrorMm",NaN,"maxNodeOrientationErrorDeg",NaN, ...
    "maxSegmentPositionErrorMm",NaN,"maxSegmentOrientationErrorDeg",NaN, ...
    "checkedSegments",0,"allSegmentsChecked",false);
q = r.qPath;
if isempty(q), return; end
N=size(q,1); Ttar=cell(N,1);
for i=1:N
    s=r.stateBank{i}; Q=vertcat(s.q);
    D=rad2deg(atan2(sin(Q-q(i,:)),cos(Q-q(i,:))));
    [~,j]=min(vecnorm(D,2,2)); Ttar{i}=s(j).Ttarget;
end
nodePos=zeros(N,1); nodeOri=zeros(N,1);
for i=1:N
    T=getTransform(r.scene.robot,q(i,:),r.scene.tcpBodyName);
    nodePos(i)=norm(T(1:3,4)-Ttar{i}(1:3,4));
    c=(trace(Ttar{i}(1:3,1:3).'*T(1:3,1:3))-1)/2;
    nodeOri(i)=acos(max(-1,min(1,c)));
end
sp=zeros(N-1,1); so=sp;
for i=1:N-1
    dq=atan2(sin(q(i+1,:)-q(i,:)),cos(q(i+1,:)-q(i,:)));
    n=max(2,ceil(max(abs(rad2deg(dq)))/edgeResolutionDeg)+1);
    for k=0:n-1
        s=k/(n-1); qq=q(i,:)+s*dq;
        T=getTransform(r.scene.robot,qq,r.scene.tcpBodyName);
        p=(1-s)*Ttar{i}(1:3,4)+s*Ttar{i+1}(1:3,4);
        sp(i)=max(sp(i),norm(T(1:3,4)-p));
        q1=rotm2quat(Ttar{i}(1:3,1:3)); q2=rotm2quat(Ttar{i+1}(1:3,1:3));
        if dot(q1,q2)<0,q2=-q2;end
        qr=((1-s)*q1+s*q2); qr=qr/norm(qr); R=quat2rotm(qr);
        c=(trace(R.'*T(1:3,1:3))-1)/2;
        so(i)=max(so(i),acos(max(-1,min(1,c))));
    end
end
metrics.maxNodePositionErrorMm=max(nodePos)*1000;
metrics.maxNodeOrientationErrorDeg=max(rad2deg(nodeOri));
metrics.maxSegmentPositionErrorMm=max(sp)*1000;
metrics.maxSegmentOrientationErrorDeg=max(rad2deg(so));
metrics.checkedSegments=N-1; metrics.allSegmentsChecked=(N-1)==79;
end
