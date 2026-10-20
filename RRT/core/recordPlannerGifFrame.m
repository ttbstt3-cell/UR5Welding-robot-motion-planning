function recorded = recordPlannerGifFrame(fig,force)
% recordPlannerGifFrame  Append the current 2-D planner figure to a GIF.

if nargin < 2, force = false; end
recorded = false;
if ~isgraphics(fig,'figure') || ~isappdata(fig,'RRT_GIF_RECORDER')
    return;
end

recorder = getappdata(fig,'RRT_GIF_RECORDER');
if ~isstruct(recorder) || ~isfield(recorder,'filename')
    return;
end

recorder.sampleCount = recorder.sampleCount+1;
if ~force && mod(recorder.sampleCount-1,recorder.frameStride) ~= 0
    setappdata(fig,'RRT_GIF_RECORDER',recorder);
    return;
end

% Keep one final forced frame even if the search has reached the normal cap.
if recorder.frameCount >= recorder.maxFrames && ~force
    setappdata(fig,'RRT_GIF_RECORDER',recorder);
    return;
end
if recorder.frameCount > recorder.maxFrames
    setappdata(fig,'RRT_GIF_RECORDER',recorder);
    return;
end

drawnow;
try
    frame = getframe(fig);
    [indexedImage,map] = rgb2ind(frame.cdata,256);
    if recorder.frameCount == 0
        imwrite(indexedImage,map,recorder.filename,'gif', ...
            'LoopCount',Inf,'DelayTime',recorder.delayTime);
    else
        imwrite(indexedImage,map,recorder.filename,'gif', ...
            'WriteMode','append','DelayTime',recorder.delayTime);
    end
    recorder.frameCount = recorder.frameCount+1;
    recorded = true;
catch captureError
    % Do not make a planner fail only because a headless graphics backend
    % cannot capture one frame.  Keep the error available for diagnostics.
    recorder.lastError = captureError.message;
end
setappdata(fig,'RRT_GIF_RECORDER',recorder);
end
