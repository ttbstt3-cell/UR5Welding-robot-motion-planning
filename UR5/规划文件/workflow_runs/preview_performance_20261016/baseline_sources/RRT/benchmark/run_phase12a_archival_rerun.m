function report = run_phase12a_archival_rerun(options)
%run_phase12a_archival_rerun Reproduce Phase 12A-1 only to archive qPaths.
%
% This is not a new performance experiment.  It fixes the earlier artifact
% omission by rerunning the exact 6 cases x 4 planners with the original
% seed/configuration and saving every qPath plus provenance metadata.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('oldSummaryFile',fullfile(root,'results','phase12a_functional_summary.csv'), ...
    'outputFile',fullfile(root,'results','phase12a_functional_archival.mat'), ...
    'summaryFile',fullfile(root,'results','phase12a_functional_archival_summary.csv'), ...
    'comparisonFile',fullfile(root,'results','phase12a_functional_archival_comparison.csv'), ...
    'maxIterations',300,'maxJointStepRad',deg2rad(25), ...
    'jointEdgeResolutionRad',deg2rad(2),'goalBias',0.20,'seed',1, ...
    'saveResult',true);
options=mergeOptions(defaults,options);

runnerOptions=struct('maxIterations',options.maxIterations, ...
    'maxJointStepRad',options.maxJointStepRad, ...
    'jointEdgeResolutionRad',options.jointEdgeResolutionRad, ...
    'goalBias',options.goalBias,'seed',options.seed, ...
    'saveResult',options.saveResult,'phase','Phase 12A-1 archival rerun', ...
    'archivalRerun',true,'outputFile',options.outputFile, ...
    'summaryFile',options.summaryFile);
report=run_phase12a_functional(runnerOptions);
report.comparison=compareRows(report.rows,options.oldSummaryFile);
report.note=['Exact archival rerun for qPath persistence; planner logic and ', ...
    'case/config/seed definitions were not changed.'];
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writeComparison(report.comparison.rows,options.comparisonFile);
end
fprintf('\nARCHIVAL COMPARISON PASS=%d\n',report.comparison.pass);
end

function comparison=compareRows(newRows,oldFile)
if ~isfile(oldFile), error('Original summary not found: %s',oldFile); end
old=readtable(oldFile,'TextType','string');
out=repmat(struct('caseID',"",'algorithm',"",'seed',NaN,'match',false, ...
    'successSame',false,'pathValidSame',false,'pathCostSame',false, ...
    'nodeCountSame',false,'collisionChecksSame',false,'pathStateCountSame',false, ...
    'firstIterationSame',false,'timeRatio',NaN,'pass',false,'note',""),0,1);
for i=1:height(old)
    idx=find(arrayfun(@(x) string(x.caseID)==old.caseID(i) && ...
        string(x.algorithm)==old.algorithm(i) && x.seed==old.seed(i),newRows),1);
    row=emptyComparisonRow();
    row.caseID=old.caseID(i); row.algorithm=old.algorithm(i); row.seed=old.seed(i);
    if isempty(idx)
        row.note="missing archival row"; out(end+1,1)=row; continue;
    end
    n=newRows(idx); row.match=true;
    row.successSame=n.success==logical(old.success(i));
    row.pathValidSame=n.pathValid==logical(old.pathValid(i));
    % The historical CSV stores jointPathCostL2 with finite decimal
    % precision.  Compare against the archived MAT value with an explicit
    % audit tolerance rather than treating CSV round-off as planner drift.
    row.pathCostSame=bothFiniteOrSame(n.jointPathCostL2,old.jointPathCostL2(i),1e-7);
    row.nodeCountSame=n.nodeCount==old.nodeCount(i);
    row.collisionChecksSame=n.collisionChecks==old.collisionChecks(i);
    row.pathStateCountSame=n.pathStateCount==old.pathStateCount(i);
    row.firstIterationSame=bothFiniteOrSame(n.firstSolutionIteration,old.firstSolutionIteration(i),0);
    row.timeRatio=n.planningTime_ms/max(old.planningTime_ms(i),eps);
    row.pass=row.match && row.successSame && row.pathValidSame && row.pathCostSame && ...
        row.nodeCountSame && row.collisionChecksSame && row.pathStateCountSame && row.firstIterationSame;
    if ~row.pass, row.note="deterministic metric drift"; end
    out(end+1,1)=row;
end
comparison=struct('oldSummaryFile',oldFile,'rows',out,'pass',all([out.pass]), ...
    'timeComparisonInformational',true, ...
    'note','Planning wall time is not a deterministic equality criterion.');
end

function row=emptyComparisonRow()
row=struct('caseID',"",'algorithm',"",'seed',NaN,'match',false, ...
    'successSame',false,'pathValidSame',false,'pathCostSame',false, ...
    'nodeCountSame',false,'collisionChecksSame',false,'pathStateCountSame',false, ...
    'firstIterationSame',false,'timeRatio',NaN,'pass',false,'note',"");
end

function same=bothFiniteOrSame(a,b,tolerance)
if isnan(a) && isnan(b), same=true; return; end
same=isfinite(a) && isfinite(b) && abs(a-b)<=tolerance;
end

function writeComparison(rows,file)
fid=fopen(file,'w'); if fid<0, error('Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',['caseID,algorithm,seed,match,successSame,pathValidSame,pathCostSame,', ...
    'nodeCountSame,collisionChecksSame,pathStateCountSame,firstIterationSame,', ...
    'timeRatio,pass,note',char(10)]);
for i=1:numel(rows)
    r=rows(i);
    fprintf(fid,'"%s","%s",%d,%d,%d,%d,%d,%d,%d,%d,%d,%.9g,%d,"%s"%s', ...
        r.caseID,r.algorithm,r.seed,r.match,r.successSame,r.pathValidSame, ...
        r.pathCostSame,r.nodeCountSame,r.collisionChecksSame,r.pathStateCountSame, ...
        r.firstIterationSame,r.timeRatio,r.pass,r.note,char(10));
end
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
