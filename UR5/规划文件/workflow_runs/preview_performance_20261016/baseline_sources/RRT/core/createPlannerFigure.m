function figureHandle = createPlannerFigure(config,name)
% createPlannerFigure  Create the optional 2-D planner visualization.

headless = strcmp(getenv('RRT_HEADLESS'),'1');
if headless
    visibility = 'off';
else
    visibility = 'on';
end

figureHandle = figure( ...
    'Name',name, ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Visible',visibility, ...
    'Position',[100 100 900 700], ...
    'Resize','off');

hold on;
grid on;
box on;
axis equal;
xlim(config.mapBounds(1:2));
ylim(config.mapBounds(3:4));
xlabel('X');
ylabel('Y');
title(name);

for i = 1:size(config.obstacles,1)
    rectangle( ...
        'Position',config.obstacles(i,:), ...
        'FaceColor',[0.55 0.55 0.55], ...
        'EdgeColor','k', ...
        'LineWidth',1.2);
end

plot(config.start(1),config.start(2),'go','MarkerFaceColor','g','MarkerSize',9);
plot(config.goal(1),config.goal(2),'ro','MarkerFaceColor','r','MarkerSize',9);
text(config.start(1)+2,config.start(2),'Start');
text(config.goal(1)+2,config.goal(2),'Goal');
drawnow;

% The GIF recorder is opt-in through config.gifFile.  Keeping the hook here
% means every 2-D planner uses exactly the same scene and colour conventions.
if isfield(config,'gifFile') && ~isempty(config.gifFile)
    delay = localConfigValue(config,'gifFrameDelay',0.04);
    maxFrames = localConfigValue(config,'gifMaxFrames',900);
    frameStride = localConfigValue(config,'gifFrameStride',2);
    startPlannerGifRecording(figureHandle,config.gifFile,delay, ...
        maxFrames,frameStride);
end

end

function value = localConfigValue(config,name,defaultValue)
if isfield(config,name) && ~isempty(config.(name))
    value = config.(name);
else
    value = defaultValue;
end
end
