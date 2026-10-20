function report = merge_phase12a_pilot_shards(options)
%merge_phase12a_pilot_shards Merge independently executed pilot shards.
%
% Each shard uses the exact Phase 12A-2 configuration and disjoint cases.
% This function only reads saved reports and writes the combined artifact.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('shardFiles',{{}}, ...
    'outputFile',fullfile(root,'results','phase12a_pilot_480.mat'), ...
    'rawCsvFile',fullfile(root,'results','phase12a_pilot_480_runs.csv'), ...
    'summaryCsvFile',fullfile(root,'results','phase12a_pilot_480_summary.csv'), ...
    'figureDir',fullfile(root,'results'));
options=mergeOptions(defaults,options);
if isempty(options.shardFiles), error('No shard files supplied.'); end
reports=cell(numel(options.shardFiles),1);
for i=1:numel(options.shardFiles)
    s=load(options.shardFiles{i},'report');
    if ~isfield(s,'report'), error('Shard has no final report: %s',options.shardFiles{i}); end
    reports{i}=s.report;
end
base=reports{1};
rows=base.rows; paths=base.paths; metadata=base.pathMetadata;
cases=base.cases; summary=base.summary;
for i=2:numel(reports)
    rows=[rows; reports{i}.rows]; %#ok<AGROW>
    paths=[paths; reports{i}.paths]; %#ok<AGROW>
    metadata=[metadata; reports{i}.pathMetadata]; %#ok<AGROW>
    % Shards created by older runner versions may store cases as row
    % vectors, while newer ones store them as column vectors.
    cases=[cases(:); reports{i}.cases(:)]; %#ok<AGROW>
    summary=[summary; reports{i}.summary]; %#ok<AGROW>
end
report=base;
report.rows=rows; report.paths=paths; report.pathMetadata=metadata;
report.cases=cases; report.summary=summary;
report.completedRuns=numel(rows);
report.allSuccessful=all([rows.success]);
report.allFinalValidated=all([rows.finalValidationPass]);
report.shardFiles=options.shardFiles;
report.note='Merged Phase 12A-2 shards; planning speed and joint-space path length only. No trajectory timing, ROS2, or CoppeliaSim.';
save(options.outputFile,'report','-v7.3');
writetable(struct2table(rows),options.rawCsvFile);
writetable(struct2table(summary),options.summaryCsvFile);
makeCoreFigures(rows,report.algorithms,options.figureDir);
fprintf('MERGED_PILOT rows=%d success=%d final025=%d\n',numel(rows), ...
    sum([rows.success]),sum([rows.finalValidationPass]));
end

function makeCoreFigures(rows,algorithms,figureDir)
if ~isfolder(figureDir), mkdir(figureDir); end
valid=[rows.finalValidationPass] & [rows.success];
times=[rows(valid).planningTime_ms]; lengths=[rows(valid).jointPathLengthL2_rad];
groups=cellstr(string({rows(valid).algorithm}));
f=figure('Visible','off'); boxplot(times,groups); ylabel('Planning time (ms)'); grid on;
title('Phase 12A-2 planning time'); saveas(f,fullfile(figureDir,'phase12a_pilot_planning_time_boxplot.png')); close(f);
f=figure('Visible','off'); boxplot(lengths,groups); ylabel('Joint path length L_q (rad)'); grid on;
title('Phase 12A-2 joint path length'); saveas(f,fullfile(figureDir,'phase12a_pilot_path_length_boxplot.png')); close(f);
f=figure('Visible','off'); hold on; colors=lines(numel(algorithms));
for a=1:numel(algorithms)
    name=algorithmLabel(algorithms{a});
    mask=valid(:).' & arrayfun(@(x) string(x.algorithm)==name,rows).';
    scatter([rows(mask).planningTime_ms],[rows(mask).jointPathLengthL2_rad],24,colors(a,:), ...
        'filled','DisplayName',name);
end
xlabel('Planning time (ms)'); ylabel('Joint path length L_q (rad)'); grid on;
legend('Location','best'); title('Phase 12A-2 speed vs path length');
saveas(f,fullfile(figureDir,'phase12a_pilot_planning_vs_path_scatter.png')); close(f);
end

function label=algorithmLabel(name)
switch char(name)
    case 'RRTConnectJoint', label="RRT-Connect";
    case 'BiRRTStarJoint', label="Bi-RRT*";
    case 'ClassGoalLiteJoint', label="ClassGoal-Lite";
    case 'ClassifiedRRTStarJoint', label="Full Classified-RRT*";
    otherwise, label=string(name);
end
end

function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
