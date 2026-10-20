function comparison = compare_phase12a_anytime_with_original_rrt(anytimeFile,legacyFile,options)
%COMPARE_PHASE12A_ANYTIME_WITH_ORIGINAL_RRT Create an auditable comparison.
% The historical RRT-Connect result is kept separate from the 30 s anytime
% method-validation run.  The latter contains Home -> PreWeld only.

if nargin < 1 || isempty(anytimeFile)
    root = fileparts(fileparts(mfilename('fullpath')));
    anytimeFile = fullfile(root,'results','phase12a_anytime_home_preweld_seed1.mat');
end
if nargin < 2 || isempty(legacyFile)
    root = fileparts(fileparts(mfilename('fullpath')));
    legacyFile = fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat');
end
if nargin < 3 || isempty(options), options=struct(); end
root = fileparts(fileparts(mfilename('fullpath')));
defaults = struct('csvFile',fullfile(root,'results','phase12a_anytime_vs_original_rrt.csv'), ...
    'figureFile',fullfile(root,'results','phase12a_anytime_vs_original_rrt.png'));
options = localMergeOptions(defaults,options);
addpath(genpath(root));
s = load(anytimeFile,'report'); anytime = s.report;
s = load(legacyFile,'result'); legacy = s.result;

% The first two rows make the direct Home -> PreWeld comparison.
rows(1) = localRow('Home -> PreWeld','Original RRT-Connect', ...
    legacy.approach.stats.planningTimeSec,legacy.approach.stats.planningTimeSec, ...
    legacy.approach.stats.pathLengthRad,legacy.approach.stats.pathLengthRad, ...
    legacy.approach.stats.iterations,legacy.approach.stats.collisionChecks, ...
    legacy.approach.stats.edgeChecks,'historical_reproduction');
rows(2) = localRow('Home -> PreWeld','RRT-Connect (30 s run)', ...
    anytime.rows(1).firstSolutionTime_ms/1000,anytime.rows(1).planningTime_ms/1000, ...
    anytime.rows(1).firstPathLengthL2_rad,anytime.rows(1).bestPathLengthAt30s, ...
    anytime.rows(1).nodeCount,anytime.rows(1).collisionConfigurationChecks, ...
    anytime.rows(1).edgeCollisionChecks,'anytime_first_solution');
rows(3) = localRow('Home -> PreWeld','Bi-RRT* (30 s anytime)', ...
    anytime.rows(2).firstSolutionTime_ms/1000,anytime.rows(2).planningTime_ms/1000, ...
    anytime.rows(2).firstPathLengthL2_rad,anytime.rows(2).bestPathLengthAt30s, ...
    anytime.rows(2).nodeCount,anytime.rows(2).collisionConfigurationChecks, ...
    anytime.rows(2).edgeCollisionChecks,'anytime_best_so_far');

% Keep the historical exit segment in the same comparison artifact.  There
% is intentionally no new Bi-RRT* exit value in the single-task run.
rows(4) = localRow('PostWeld -> Home','Original RRT-Connect', ...
    legacy.exit.stats.planningTimeSec,legacy.exit.stats.planningTimeSec, ...
    legacy.exit.stats.pathLengthRad,legacy.exit.stats.pathLengthRad, ...
    legacy.exit.stats.iterations,legacy.exit.stats.collisionChecks, ...
    legacy.exit.stats.edgeChecks,'historical_reproduction');

comparison = struct('createdAt',string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'anytimeFile',string(anytimeFile),'legacyFile',string(legacyFile), ...
    'rows',rows,'note',"The latest 30 s file is a Home -> PreWeld method validation; the original RRT baseline contains both approach and exit segments.");
if ~isempty(options.csvFile), writetable(struct2table(rows),options.csvFile); end
if ~isempty(options.figureFile), localMakeFigure(rows,options.figureFile); end
fprintf('COMPARISON_DONE csv=%s figure=%s\n',options.csvFile,options.figureFile);
end

function row = localRow(task,algorithm,firstTime,wallTime,firstLength,bestLength,iterations,configChecks,edgeChecks,status)
row = struct('task',string(task),'algorithm',string(algorithm), ...
    'firstSolutionTime_s',firstTime,'planningWallTime_s',wallTime, ...
    'firstPathLengthL2_rad',firstLength,'bestPathLengthAt30s_rad',bestLength, ...
    'iterationsOrNodes',iterations,'collisionConfigurationChecks',configChecks, ...
    'edgeCollisionChecks',edgeChecks,'status',string(status));
end

function localMakeFigure(rows,fileName)
fig = figure('Visible','off','Color','w','Position',[100 100 1200 620]);
t = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
home = rows([rows.task]=="Home -> PreWeld");
labels = categorical(string({home.algorithm}));
nexttile(t); hold on; grid on; box on;
bar(labels,[home.firstSolutionTime_s]);
ylabel('First solution time (s)'); title('Home -> PreWeld');
xtickangle(25);
nexttile(t); hold on; grid on; box on;
x = 1:numel(home); plot(x,[home.firstPathLengthL2_rad],'o-','LineWidth',1.6,'DisplayName','first path');
plot(x,[home.bestPathLengthAt30s_rad],'s--','LineWidth',1.6,'DisplayName','best at 30 s');
xticks(x); xticklabels({home.algorithm}); xtickangle(25);
ylabel('Joint-space path length (rad)'); legend('Location','best');
title('First path vs 30 s best-so-far');
exportgraphics(fig,fileName,'Resolution',150); close(fig);
end

function out = localMergeOptions(defaults,in)
out = defaults; names = fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
