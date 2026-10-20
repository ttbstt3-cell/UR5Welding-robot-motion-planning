function [P,R,S]=hmi_continuous_reference(ref,t)
% Independent frozen original Cartesian geometry + declared quintic clock.
assert(isfinite(ref.duration) && ref.duration>0,'CONTINUOUS:Reference','Invalid reference duration');
if isfield(ref,'clockTimes')
    assert(all(diff(ref.clockTimes)>0) && all(diff(ref.clockProgress)>0) &&...
        ref.clockTimes(1)==0 && ref.clockTimes(end)==ref.duration &&...
        ref.clockProgress(1)==0 && ref.clockProgress(end)==1,...
        'CONTINUOUS:Reference','Invalid monotone reference clock');
    tau=ppval(pchip(ref.clockTimes,ref.clockProgress),max(0,min(ref.duration,double(t(:)))));
else
    tau=double(t(:))/ref.duration;
end
tau=max(0,min(1,tau));
S=10*tau.^3-15*tau.^4+6*tau.^5;
[P,R]=hmi_continuous_pose_at_s(ref,S);
end
