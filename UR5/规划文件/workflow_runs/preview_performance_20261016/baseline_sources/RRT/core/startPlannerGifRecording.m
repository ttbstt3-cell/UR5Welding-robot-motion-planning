function startPlannerGifRecording(fig,filename,delayTime,maxFrames,frameStride)
% startPlannerGifRecording  Attach a lightweight GIF recorder to a 2-D figure.
%
% The recorder is intentionally opt-in.  Planner code only needs to call
% recordPlannerGifFrame after drawnow; this keeps the planning algorithms and
% their existing colours unchanged.

if nargin < 3 || isempty(delayTime), delayTime = 0.04; end
if nargin < 4 || isempty(maxFrames), maxFrames = 900; end
if nargin < 5 || isempty(frameStride), frameStride = 2; end

filename = char(filename);
parent = fileparts(filename);
if ~isempty(parent) && ~isfolder(parent)
    mkdir(parent);
end
if isfile(filename)
    delete(filename);
end

recorder = struct( ...
    'filename',filename, ...
    'delayTime',double(delayTime), ...
    'maxFrames',double(maxFrames), ...
    'frameStride',max(1,round(double(frameStride))), ...
    'sampleCount',0, ...
    'frameCount',0);
setappdata(fig,'RRT_GIF_RECORDER',recorder);
drawnow;
recordPlannerGifFrame(fig,true);
end
