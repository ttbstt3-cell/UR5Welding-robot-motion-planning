function [timeFromStart, details] = parameterize_joint_path(qPath, varargin)
%PARAMETERIZE_JOINT_PATH Create conservative monotonic joint timestamps.
%
% This function only assigns time.  It does not alter, wrap, reorder, or
% smooth the supplied continuous joint path.  The returned vector is safe to
% inspect in the HMI and can be exported explicitly by the caller.

qPath = double(qPath);
if ndims(qPath) ~= 2 || size(qPath,2) ~= 6 || size(qPath,1) < 2
    error("qPath must be an N-by-6 matrix with at least two points.");
end
if any(~isfinite(qPath(:)))
    error("qPath contains NaN or Inf.");
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p,"MaxVelocityDegPerSec",5,@(x)isscalar(x)&&isfinite(x)&&x>0);
addParameter(p,"MaxAccelerationDegPerSec2",15,@(x)isscalar(x)&&isfinite(x)&&x>0);
addParameter(p,"MinimumSegmentDurationSec",0.25,@(x)isscalar(x)&&isfinite(x)&&x>0);
parse(p,varargin{:});
cfg = p.Results;

dq = diff(qPath,1,1);
dqAbs = abs(dq);
vMax = deg2rad(cfg.MaxVelocityDegPerSec);
aMax = deg2rad(cfg.MaxAccelerationDegPerSec2);

% Every joint must satisfy both the velocity and a conservative acceleration
% bound.  Taking the maximum over the six joints keeps the segment duration
% common to all joints and preserves the exact path samples.
dtVelocity = max(dqAbs,[],2) ./ vMax;
dtAcceleration = sqrt(2*max(dqAbs,[],2) ./ aMax);
dt = max([dtVelocity,dtAcceleration,repmat(cfg.MinimumSegmentDurationSec,size(dq,1),1)],[],2);
dt(~isfinite(dt) | dt <= 0) = cfg.MinimumSegmentDurationSec;
timeFromStart = [0; cumsum(dt)];

details = struct();
details.maxVelocityDegPerSec = cfg.MaxVelocityDegPerSec;
details.maxAccelerationDegPerSec2 = cfg.MaxAccelerationDegPerSec2;
details.minimumSegmentDurationSec = cfg.MinimumSegmentDurationSec;
details.segmentDurationSec = dt;
details.maxAdjacentStepRad = max(max(dqAbs,[],2));
details.durationSec = timeFromStart(end);
details.maxEstimatedVelocityRadPerSec = max(max(dqAbs ./ dt,[],2));
details.maxEstimatedAccelerationRadPerSec2 = NaN;
if size(qPath,1) > 2
    v = dq ./ dt;
    a = diff(v,1,1) ./ dt(2:end);
    details.maxEstimatedAccelerationRadPerSec2 = max(abs(a(:)));
end
end
