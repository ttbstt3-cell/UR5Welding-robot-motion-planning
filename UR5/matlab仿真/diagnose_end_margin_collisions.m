function diagnostics = diagnose_end_margin_collisions(options)
%DIAGNOSE_END_MARGIN_COLLISIONS
% Summarize where the local dense diagnostic saw collision-rejected edges.
%
% The current rigidBodyTree/checkCollision aggregate API returns a Boolean
% collision result, not the colliding body pair.  This report therefore does
% not invent object names: collisionPair is explicitly marked unavailable.
% The report does locate every rejected transition along the physical seam so
% endpoint effects can be separated from interior failures.

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(mfilename('fullpath'));
defaults = struct( ...
    'denseReportFile',fullfile(root,'local_dense_connectivity_new11.mat'), ...
    'physicalSeamFile',fullfile(root,'end_margin_benchmark_runs','0mm','seed20270001.mat'), ...
    'outputCsv',fullfile(root,'end_collision_diagnostics.csv'), ...
    'outputMat',fullfile(root,'end_collision_diagnostics.mat'));
options = localMerge(defaults,options);

S = load(options.denseReportFile,'report');
cases = S.report.cases;
P = load(options.physicalSeamFile,'result');
physicalSeam = P.result.seam;
p0 = physicalSeam.startWP(:).';
p1 = physicalSeam.endWP(:).';
t = (p1-p0); t = t/norm(t);
L = norm(p1-p0);

rows = repmat(localRow(),0,1);
for ic = 1:numel(cases)
    c = cases(ic);
    pos = c.densePositionsWP;
    tr = c.transitions;
    for k = 1:numel(tr)
        if tr(k).collisionRejectCount <= 0, continue; end
        a = pos(k,:); b = pos(k+1,:); mid = 0.5*(a+b);
        s = dot(mid-p0,t);
        row = localRow();
        row.seed = c.seed;
        row.transitionIndex = k;
        row.sFromMm = 1000*dot(a-p0,t);
        row.sToMm = 1000*dot(b-p0,t);
        row.sMidMm = 1000*s;
        row.distanceFromStartMm = 1000*s;
        row.distanceToEndMm = 1000*(L-s);
        row.nearestPhysicalEndMm = min(row.distanceFromStartMm,row.distanceToEndMm);
        row.collisionRejectedEdges = tr(k).collisionRejectCount;
        row.poseRejectedEdges = tr(k).poseRejectCount;
        row.jointRejectedEdges = tr(k).jointRejectCount;
        row.fromBase = tr(k).fromBase;
        row.toBase = tr(k).toBase;
        row.collisionPair = "UNAVAILABLE_AGGREGATE_CHECKCOLLISION";
        rows(end+1,1) = row; %#ok<AGROW>
    end
end

if isempty(rows)
    rows = localRow(); rows(1).seed = NaN;
end
writetable(struct2table(rows),options.outputCsv);
diagnostics = struct('rows',rows,'physicalStartWP',p0,'physicalEndWP',p1, ...
    'physicalLengthMm',1000*L,'options',options,'created',char(datetime('now')));
save(options.outputMat,'diagnostics','-v7.3');

fprintf('Saved endpoint collision diagnostics: %s\n',options.outputCsv);
if any(~isnan([rows.seed]))
    seeds = unique([rows.seed]);
    fprintf('Seeds with collision-rejected transitions: %s\n',strjoin(string(seeds),', '));
    fprintf('Physical seam length: %.3f mm\n',1000*L);
    fprintf('Nearest physical-end distance range: %.3f .. %.3f mm\n', ...
        min([rows.nearestPhysicalEndMm]),max([rows.nearestPhysicalEndMm]));
end
end

function row = localRow()
row = struct('seed',NaN,'transitionIndex',NaN,'fromBase',NaN,'toBase',NaN, ...
    'sFromMm',NaN,'sToMm',NaN,'sMidMm',NaN,'distanceFromStartMm',NaN, ...
    'distanceToEndMm',NaN,'nearestPhysicalEndMm',NaN, ...
    'collisionRejectedEdges',0,'poseRejectedEdges',0,'jointRejectedEdges',0, ...
    'collisionPair',"");
end

function out = localMerge(defaults,custom)
out = defaults;
f = fieldnames(custom);
for i=1:numel(f), out.(f{i}) = custom.(f{i}); end
end
