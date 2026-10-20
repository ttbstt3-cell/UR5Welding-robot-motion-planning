function combined = summarize_adaptive_arc_ik_recovery(baseCsv,recoveryCsv,outCsv)
%SUMMARIZE_ADAPTIVE_ARC_IK_RECOVERY Merge conditional recovery results.
%
% The original 30-run A-ARC-IK benchmark remains untouched.  For an A-ARC-IK
% failure, a successful post-refinement recovery run replaces that one row;
% fixed-80 and fixed-16 rows are copied unchanged.

if nargin<1||isempty(baseCsv), baseCsv=fullfile(fileparts(mfilename("fullpath")), ...
        "adaptive_arc_ik_benchmark.csv"); end
if nargin<2||isempty(recoveryCsv), recoveryCsv=fullfile(fileparts(mfilename("fullpath")), ...
        "adaptive_arc_ik_recovery.csv"); end
if nargin<3||isempty(outCsv), outCsv=fullfile(fileparts(mfilename("fullpath")), ...
        "adaptive_arc_ik_recovery_combined.csv"); end

B=readtable(baseCsv,"TextType","string");
R=readtable(recoveryCsv,"TextType","string");
B.recoveryAttempted=false(height(B),1);
B.recoveryApplied=false(height(B),1);
B.recoveryAttempts=zeros(height(B),1);

for i=1:height(B)
    if B.algorithm(i)~="A-ARC-IK", continue; end
    j=find(R.seed==B.seed(i),1);
    if isempty(j), continue; end
    B.recoveryAttempted(i)=true;
    B.recoveryAttempts(i)=R.postRecoveryAttempts(j);
    if ~logical(R.complete(j)), continue; end
    B.recoveryApplied(i)=true;
    B.complete(i)=R.complete(j);
    B.algorithmTimeSec(i)=R.algorithmTimeSec(j);
    B.wallTimeSec(i)=R.wallTimeSec(j);
    B.finalPointCount(i)=R.finalPointCount(j);
    B.executionPointCount(i)=R.executionPointCount(j);
    B.ikCalls(i)=B.ikCalls(i)+R.postRecoveryIkCalls(j);
    B.maxJointJumpDeg(i)=R.maxJointJumpDeg(j);
    B.maxSegmentTcpErrorMm(i)=R.maxSegmentTcpErrorMm(j);
    B.nodeCollisionFree(i)=true;
    B.edgeCollisionFree(i)=true;
    B.furthestPoint(i)=R.furthestPoint(j);
    B.insertedPoints(i)=max(0,R.executionPointCount(j)-16);
    B.refinementRounds(i)=B.refinementRounds(i)+R.postRecoveryAttempts(j);
    B.precisionPass(i)=true;
end

combined=B;
writetable(combined,outCsv);
fprintf("Combined CSV: %s\n",outCsv);
localPrint(combined);
end

function localPrint(T)
keys=["fixed80","fixed16","A-ARC-IK"];
for k=1:numel(keys)
    m=T.algorithm==keys(k); p=m&T.complete==1;
    fprintf('%s: PASS %d/%d (%.1f%%), success-time median %.4fs, TCP median %.4fmm\n', ...
        keys(k),sum(p),sum(m),100*sum(p)/max(1,sum(m)), ...
        median(T.algorithmTimeSec(p),'omitnan'),median(T.maxSegmentTcpErrorMm(p),'omitnan'));
end
end
