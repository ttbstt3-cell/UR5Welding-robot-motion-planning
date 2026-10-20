function comparison = refresh_phase12a_archival_comparison(options)
%refresh_phase12a_archival_comparison Compare an archival report read-only.
%
% This helper intentionally does not call any planner.  It compares the
% persisted archival rows against the original Phase 12A CSV and refreshes
% only the comparison artifact.  The path-cost tolerance accounts for the
% finite decimal precision of the historical CSV.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('archivalFile',fullfile(root,'results','phase12a_functional_archival.mat'), ...
    'oldSummaryFile',fullfile(root,'results','phase12a_functional_summary.csv'), ...
    'comparisonFile',fullfile(root,'results','phase12a_functional_archival_comparison.csv'));
options=mergeOptions(defaults,options);
s=load(options.archivalFile,'report');
comparison=compareRows(s.report.rows,options.oldSummaryFile);
s.report.comparison=comparison;
save(options.archivalFile,'-struct','s','-v7.3');
writeComparison(comparison.rows,options.comparisonFile);
fprintf('ARCHIVAL COMPARISON REFRESH PASS=%d rows=%d pathCostTolerance=1e-7\n', ...
    comparison.pass,numel(comparison.rows));
end

function comparison=compareRows(newRows,oldFile)
if ~isfile(oldFile), error('Original summary not found: %s',oldFile); end
old=readtable(oldFile,'TextType','string');
out=repmat(emptyComparisonRow(),0,1);
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
    'timeComparisonInformational',true,'pathCostTolerance',1e-7, ...
    'note','Planning wall time is not deterministic; path cost uses 1e-7 tolerance for historical CSV round-off.');
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
