function out = make_phase12a_three_algorithm_comparison(options)
%MAKE_PHASE12A_THREE_ALGORITHM_COMPARISON Make screenshot-style comparisons.
% Algorithms: original RRT-Connect baseline, current RRT-Connect, Bi-RRT*.
% Two figures are written: first certified solution and 30 s best-so-far.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(fileparts(mfilename('fullpath')));
defaults=struct( ...
    'legacyFile',fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat'), ...
    'firstFile',fullfile(root,'results','phase12a_first_solution_two_successful.mat'), ...
    'bestFile',fullfile(root,'results','phase12a_anytime_seamA_two_successful_with_iterations.mat'), ...
    'csvFile',fullfile(root,'results','phase12a_three_algorithm_comparison.csv'), ...
    'firstFigure',fullfile(root,'results','phase12a_three_algorithm_first_solution_comparison.png'), ...
    'bestFigure',fullfile(root,'results','phase12a_three_algorithm_30s_best_comparison.png'));
options=localMergeOptions(defaults,options);
for f={options.legacyFile,options.firstFile,options.bestFile}
    if ~isfile(f{1}), error('Missing comparison input: %s',f{1}); end
end
s=load(options.legacyFile,'result'); legacy=s.result;
s=load(options.firstFile,'report'); first=s.report;
s=load(options.bestFile,'report'); best=s.report;

algorithms=["RRT（原始基线）","RRT-Connect","Bi-RRT*"];
firstRows=repmat(emptyRow(),6,1); k=0;
k=k+1; firstRows(k)=localRow("Home -> PreWeld",algorithms(1),true, ...
    legacy.approach.stats.planningTimeSec,legacy.approach.stats.iterations, ...
    legacy.approach.stats.collisionChecks,legacy.approach.stats.pathLengthRad);
k=k+1; firstRows(k)=localRow("PostWeld -> Home",algorithms(1),true, ...
    legacy.exit.stats.planningTimeSec,legacy.exit.stats.iterations, ...
    legacy.exit.stats.collisionChecks,legacy.exit.stats.pathLengthRad);
for ti=1:2
    r=first.rows(ti*2-1);
    k=k+1; firstRows(k)=localRow(string(r.description),algorithms(2),true, ...
        r.firstSolutionTime_ms/1000,r.firstSolutionIteration,r.collisionConfigurationChecks,r.firstPathLengthL2_rad);
    r=first.rows(ti*2);
    k=k+1; firstRows(k)=localRow(string(r.description),algorithms(3),true, ...
        r.firstSolutionTime_ms/1000,r.firstSolutionIteration,r.collisionConfigurationChecks,r.firstPathLengthL2_rad);
end
firstRows=localReorder(firstRows,algorithms);

bestRows=repmat(emptyRow(),6,1); k=0;
k=k+1; bestRows(k)=localRow("Home -> PreWeld",algorithms(1),true, ...
    legacy.approach.stats.planningTimeSec,legacy.approach.stats.iterations, ...
    legacy.approach.stats.collisionChecks,legacy.approach.stats.pathLengthRad);
k=k+1; bestRows(k)=localRow("PostWeld -> Home",algorithms(1),true, ...
    legacy.exit.stats.planningTimeSec,legacy.exit.stats.iterations, ...
    legacy.exit.stats.collisionChecks,legacy.exit.stats.pathLengthRad);
for ti=1:2
    r=best.rows(ti*2-1);
    k=k+1; bestRows(k)=localRow(string(r.description),algorithms(2),true, ...
        r.planningTime_ms/1000,r.iterations,r.collisionConfigurationChecks,r.bestPathLengthAt30s);
    r=best.rows(ti*2);
    k=k+1; bestRows(k)=localRow(string(r.description),algorithms(3),true, ...
        r.planningTime_ms/1000,r.iterations,r.collisionConfigurationChecks,r.bestPathLengthAt30s);
end
bestRows=localReorder(bestRows,algorithms);

allRows=[firstRows;bestRows];
allRows(1:6)=localSetMode(allRows(1:6),"First certified solution");
allRows(7:12)=localSetMode(allRows(7:12),"30 s best-so-far");
writetable(struct2table(allRows),options.csvFile);
localMakeFigure(firstRows,algorithms,"首条合法路径对比",options.firstFigure);
localMakeFigure(bestRows,algorithms,"30 s best-so-far 对比",options.bestFigure);
out=struct('csvFile',string(options.csvFile),'firstFigure',string(options.firstFigure), ...
    'bestFigure',string(options.bestFigure),'firstRows',firstRows,'bestRows',bestRows);
fprintf('THREE_ALGORITHM_COMPARISON_DONE\n');
end

function row=localRow(path,algorithm,pass,timeSec,iterations,checks,lengthRad)
row=struct('mode',"",'algorithm',string(algorithm),'path',string(path),'result',string(localPass(pass)), ...
    'planningTime_s',timeSec,'iterations',iterations,'collisionChecks',checks,'pathLength_rad',lengthRad);
end

function rows=localSetMode(rows,mode)
[rows.mode]=deal(string(mode));
end

function rows=localReorder(rows,algorithms)
% Input layout is [original home, original exit, RRT home, Bi home,
% RRT exit, Bi exit].  Output is algorithm blocks, each with two tasks.
out=repmat(rows(1),6,1);
out(1)=rows(1); out(2)=rows(2);
out(3)=rows(3); out(4)=rows(5);
out(5)=rows(4); out(6)=rows(6);
for ai=1:3
    out(2*ai-1).algorithm=algorithms(ai);
    out(2*ai).algorithm=algorithms(ai);
end
rows=out;
end

function textValue=localPass(tf)
if tf, textValue="PASS"; else, textValue="FAIL"; end
end

function localMakeFigure(rows,algorithms,figureTitle,fileName)
fig=figure('Visible','off','Color','w','Position',[80 80 2200 780]);
t=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
for ai=1:3
    ax=nexttile(t); axis(ax,[0 1 0 1]); axis(ax,'off');
    title(ax,algorithms(ai),'FontSize',18,'FontWeight','bold');
    text(ax,0.5,0.96,figureTitle,'HorizontalAlignment','center','FontSize',13);
    headers={"路径","结果","规划时间","迭代次数","碰撞检查","路径长度"};
    x=[0.02 0.38 0.52 0.66 0.79 0.92];
    for j=1:numel(headers)
        text(ax,x(j),0.82,headers{j},'FontSize',11,'FontWeight','bold', ...
            'HorizontalAlignment',localAlign(j));
    end
    subset=rows([rows.algorithm]==algorithms(ai));
    for ri=1:2
        y=0.62-(ri-1)*0.25; r=subset(ri);
        vals={char(r.path),char(r.result),sprintf('%.3f s',r.planningTime_s), ...
            sprintf('%d',round(r.iterations)),sprintf('%d',round(r.collisionChecks)), ...
            sprintf('%.3f rad',r.pathLength_rad)};
        for j=1:numel(vals)
            text(ax,x(j),y,vals{j},'FontSize',12,'HorizontalAlignment',localAlign(j));
        end
    end
    line(ax,[0.01 0.99],[0.72 0.72],'Color',[0.75 0.75 0.75]);
    line(ax,[0.01 0.99],[0.47 0.47],'Color',[0.88 0.88 0.88]);
end
exportgraphics(fig,fileName,'Resolution',180); close(fig);
end

function a=localAlign(j)
if j==1, a='left'; else, a='center'; end
end

function out=localMergeOptions(defaults,in)
out=defaults; f=fieldnames(defaults);
for i=1:numel(f), if isfield(in,f{i}) && ~isempty(in.(f{i})), out.(f{i})=in.(f{i}); end, end
end

function row=emptyRow()
row=struct('mode',"",'algorithm',"",'path',"",'result',"",'planningTime_s',NaN, ...
    'iterations',NaN,'collisionChecks',NaN,'pathLength_rad',NaN);
end
