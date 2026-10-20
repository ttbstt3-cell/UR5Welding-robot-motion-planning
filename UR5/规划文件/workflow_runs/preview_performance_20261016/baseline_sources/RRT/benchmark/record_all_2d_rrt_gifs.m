function report = record_all_2d_rrt_gifs(options)
% record_all_2d_rrt_gifs  Record all project 2-D RRT demos as GIF files.
%
% The ten launchers are run on the shared default 2-D map with the same seed.
% Each GIF contains the planner's search-tree animation and final blue path;
% after planning, a purple marker replays the resulting path.
%
% Example:
%   report = record_all_2d_rrt_gifs();

if nargin < 1 || isempty(options), options = struct(); end
root = fileparts(fileparts(mfilename('fullpath')));
defaults = struct( ...
    'outputDirectory',fullfile(root,'results','2d_rrt_gifs'), ...
    'seed',1, ...
    'maxIterations',5000, ...
    'gifFrameDelay',0.04, ...
    'gifMaxFrames',900, ...
    'gifFrameStride',2, ...
    'motionFrames',35, ...
    'closeFigures',true);
options = mergeOptions(defaults,options);
if ~isfolder(options.outputDirectory), mkdir(options.outputDirectory); end

setenv('RRT_HEADLESS','1');
addpath(root);
addpath(fullfile(root,'core'));
addpath(fullfile(root,'planners'));

names = {'RRT_basic','RRT_connect','RRT_star','Bi_RRT','Bi_RRT_star', ...
    'Informed_RRT_star','Q_RRT_star','GD_RRT_star', ...
    'Classified_RRT_star','APF_Bi_RRT_star'};
labels = {'Basic RRT','RRT-Connect','RRT*','Bi-RRT','Bi-RRT*', ...
    'Informed RRT*','Q-RRT*','GD-RRT*','Classified RRT*','APF-Bi-RRT*'};

report = struct('algorithm',string.empty(0,1), ...
    'label',string.empty(0,1), ...
    'success',false(0,1), ...
    'pathLength',NaN(0,1), ...
    'planningTime_s',NaN(0,1), ...
    'frameCount',zeros(0,1), ...
    'gifFile',string.empty(0,1));

for k = 1:numel(names)
    if options.closeFigures, close all force; end
    algorithm = names{k};
    gifFile = fullfile(options.outputDirectory,[algorithm '.gif']);
    config = defaultRRTConfig();
    config.seed = options.seed;
    config.maxIterations = options.maxIterations;
    config.showAnimation = true;
    config.animationPause = 0;
    config.optimizeIterations = 0;
    config.gifFile = gifFile;
    config.gifFrameDelay = options.gifFrameDelay;
    config.gifMaxFrames = options.gifMaxFrames;
    config.gifFrameStride = options.gifFrameStride;

    % The direct APF demo uses a smaller expansion step on this wall map;
    % retain that established demo setting so it also produces a path.
    if strcmp(algorithm,'APF_Bi_RRT_star')
        config.stepSize = 1;
    end

    fprintf('\n[GIF %d/%d] %s\n',k,numel(names),labels{k});
    try
        timer = tic;
        result = feval(algorithm,config);
        planningTime = toc(timer);
        fig = currentPlannerFigure();
        if isgraphics(fig) && isfield(result,'success') && result.success && ...
                isfield(result,'path') && ~isempty(result.path)
            addPlannerMotion(fig,result.path,options.motionFrames);
        elseif isgraphics(fig)
            title(sprintf('%s - failed',labels{k}));
            drawnow;
            recordPlannerGifFrame(fig,true);
        end
        frameCount = readPlannerGifFrameCount(fig);
        success = isfield(result,'success') && logical(result.success);
        if success && isfield(result,'pathLength')
            pathLengthValue = result.pathLength;
        else
            pathLengthValue = NaN;
        end
        if isfield(result,'planningTime') && isfinite(result.planningTime)
            plannerTimeValue = result.planningTime;
        else
            plannerTimeValue = planningTime;
        end
        fprintf('  success=%d pathLength=%.6g planningTime=%.3fs frames=%d\n', ...
            success,pathLengthValue,plannerTimeValue,frameCount);
    catch err
        warning('2-D GIF failed for %s: %s',algorithm,err.message);
        result = struct('success',false,'pathLength',NaN,'planningTime',NaN);
        frameCount = 0;
        plannerTimeValue = NaN;
        pathLengthValue = NaN;
        if isgraphics(currentPlannerFigure())
            fig = currentPlannerFigure();
            title(sprintf('%s - error',labels{k}));
            drawnow;
            recordPlannerGifFrame(fig,true);
            frameCount = readPlannerGifFrameCount(fig);
        end
    end

    report.algorithm(end+1,1) = string(algorithm);
    report.label(end+1,1) = string(labels{k});
    report.success(end+1,1) = logical(result.success);
    report.pathLength(end+1,1) = pathLengthValue;
    report.planningTime_s(end+1,1) = plannerTimeValue;
    report.frameCount(end+1,1) = frameCount;
    report.gifFile(end+1,1) = string(gifFile);
    if options.closeFigures, close all force; end
end

reportTable = table(report.algorithm,report.label,report.success, ...
    report.pathLength,report.planningTime_s,report.frameCount,report.gifFile, ...
    'VariableNames',{'algorithm','label','success','pathLength', ...
    'planningTime_s','frameCount','gifFile'});
summaryFile = fullfile(options.outputDirectory,'summary.mat');
csvFile = fullfile(options.outputDirectory,'summary.csv');
save(summaryFile,'report','reportTable','options');
writetable(reportTable,csvFile);
fprintf('\nGIF_DONE count=%d directory=%s\n',numel(names),options.outputDirectory);
disp(reportTable);
end

function fig = currentPlannerFigure()
fig = gobjects(0);
figures = findall(0,'Type','figure');
if ~isempty(figures)
    fig = figures(1);
end
end

function count = readPlannerGifFrameCount(fig)
count = 0;
if isgraphics(fig) && isappdata(fig,'RRT_GIF_RECORDER')
    recorder = getappdata(fig,'RRT_GIF_RECORDER');
    if isfield(recorder,'frameCount'), count = recorder.frameCount; end
end
end

function addPlannerMotion(fig,path,motionFrames)
if isempty(path) || ~isgraphics(fig), return; end
ax = findobj(fig,'Type','axes');
if isempty(ax), return; end
ax = ax(1);
hold(ax,'on');
motionLine = plot(ax,nan,nan,'-','Color',[0.55 0.15 0.75], ...
    'LineWidth',2.0,'Tag','RRT_GIF_MOTION');
motionPoint = plot(ax,path(1,1),path(1,2),'o', ...
    'Color',[0.35 0.05 0.55], ...
    'MarkerFaceColor',[0.70 0.25 0.90], ...
    'MarkerSize',7,'Tag','RRT_GIF_MOTION');
motionFrames = max(2,round(motionFrames));
indices = unique(round(linspace(1,size(path,1),motionFrames)));
for i = indices
    motionLine.XData = path(1:i,1);
    motionLine.YData = path(1:i,2);
    motionPoint.XData = path(i,1);
    motionPoint.YData = path(i,2);
    drawnow;
    recordPlannerGifFrame(fig,true);
end
end

function out = mergeOptions(defaults,in)
out = defaults;
names = fieldnames(in);
for i = 1:numel(names)
    out.(names{i}) = in.(names{i});
end
end
