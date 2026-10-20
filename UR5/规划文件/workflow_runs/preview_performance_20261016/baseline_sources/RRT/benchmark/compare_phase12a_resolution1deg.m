function comparison = compare_phase12a_resolution1deg(options)
%compare_phase12a_resolution1deg Compare 1-degree rerun with 2-degree data.
%
% Read-only comparison. This function does not call a planner or collision
% checker; it only compares saved report/CSV artifacts.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('newFile',fullfile(root,'results','phase12a_resolution1deg_functional.mat'), ...
    'oldSummaryFile',fullfile(root,'results','phase12a_functional_summary.csv'), ...
    'outputFile',fullfile(root,'results','phase12a_resolution1deg_vs_2deg.csv'));
options=mergeOptions(defaults,options);
new=load(options.newFile,'report');
old=readtable(options.oldSummaryFile,'TextType','string');
rows=repmat(emptyRow(),0,1);
for i=1:height(old)
    idx=find(arrayfun(@(x) string(x.caseID)==old.caseID(i) && ...
        string(x.algorithm)==old.algorithm(i) && x.seed==old.seed(i),new.report.rows),1);
    if isempty(idx), continue; end
    n=new.report.rows(idx); r=emptyRow();
    r.caseID=old.caseID(i); r.algorithm=old.algorithm(i); r.seed=old.seed(i);
    r.oldPlanningTime_ms=old.planningTime_ms(i); r.newPlanningTime_ms=n.planningTime_ms;
    r.planningTimeRatio=n.planningTime_ms/max(old.planningTime_ms(i),eps);
    r.oldFirstSolutionTime_ms=old.firstSolutionTime_ms(i);
    r.newFirstSolutionTime_ms=n.firstSolutionTime_ms;
    r.oldJointPathCostL2=old.jointPathCostL2(i);
    r.newJointPathCostL2=n.jointPathCostL2;
    r.jointPathCostDelta=n.jointPathCostL2-old.jointPathCostL2(i);
    r.oldNodeCount=old.nodeCount(i); r.newNodeCount=n.nodeCount;
    r.oldCollisionConfigurationChecks=old.collisionChecks(i);
    r.newCollisionConfigurationChecks=n.collisionConfigurationChecks;
    r.configurationCheckRatio=n.collisionConfigurationChecks/max(old.collisionChecks(i),eps);
    r.newEdgeCollisionChecks=n.edgeCollisionChecks;
    r.success=n.success; r.pathValid=n.pathValid;
    rows(end+1,1)=r; %#ok<AGROW>
end
comparison=struct('oldResolutionDeg',2,'newResolutionDeg',1,'rows',rows, ...
    'matchedRows',numel(rows),'allSuccessful',all([rows.success]), ...
    'allPathValid',all([rows.pathValid]), ...
    'note','Old collisionChecks is the historical discrete configuration-check count; new explicit field is collisionConfigurationChecks.');
s=load(options.newFile,'report');
s.report.resolutionComparison=comparison;
save(options.newFile,'-struct','s','-v7.3');
writeRows(rows,options.outputFile);
fprintf('RESOLUTION1DEG_COMPARISON rows=%d allSuccessful=%d allPathValid=%d\n', ...
    comparison.matchedRows,comparison.allSuccessful,comparison.allPathValid);
end

function row=emptyRow()
row=struct('caseID',"",'algorithm',"",'seed',NaN, ...
    'oldPlanningTime_ms',NaN,'newPlanningTime_ms',NaN,'planningTimeRatio',NaN, ...
    'oldFirstSolutionTime_ms',NaN,'newFirstSolutionTime_ms',NaN, ...
    'oldJointPathCostL2',NaN,'newJointPathCostL2',NaN,'jointPathCostDelta',NaN, ...
    'oldNodeCount',NaN,'newNodeCount',NaN, ...
    'oldCollisionConfigurationChecks',NaN,'newCollisionConfigurationChecks',NaN, ...
    'configurationCheckRatio',NaN,'newEdgeCollisionChecks',NaN, ...
    'success',false,'pathValid',false);
end

function writeRows(rows,file)
fid=fopen(file,'w'); if fid<0, error('Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',['caseID,algorithm,seed,oldPlanningTime_ms,newPlanningTime_ms,planningTimeRatio,', ...
    'oldFirstSolutionTime_ms,newFirstSolutionTime_ms,oldJointPathCostL2,newJointPathCostL2,', ...
    'jointPathCostDelta,oldNodeCount,newNodeCount,oldCollisionConfigurationChecks,', ...
    'newCollisionConfigurationChecks,configurationCheckRatio,newEdgeCollisionChecks,success,pathValid',char(10)]);
for i=1:numel(rows)
    r=rows(i);
    fprintf(fid,'"%s","%s",%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%.9g,%d,%d,%d%s', ...
        r.caseID,r.algorithm,r.seed,r.oldPlanningTime_ms,r.newPlanningTime_ms, ...
        r.planningTimeRatio,r.oldFirstSolutionTime_ms,r.newFirstSolutionTime_ms, ...
        r.oldJointPathCostL2,r.newJointPathCostL2,r.jointPathCostDelta, ...
        r.oldNodeCount,r.newNodeCount,r.oldCollisionConfigurationChecks, ...
        r.newCollisionConfigurationChecks,r.configurationCheckRatio, ...
        r.newEdgeCollisionChecks,r.success,r.pathValid,char(10));
end
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
