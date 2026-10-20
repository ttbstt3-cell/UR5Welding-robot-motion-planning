function [P,R,S]=cp_reference(ref,t)
% Independent frozen original Cartesian geometry + declared quintic clock.
if isfield(ref,'clockTimes')
    tau=ppval(pchip(ref.clockTimes,ref.clockProgress),max(0,min(ref.duration,double(t(:)))));
else
    tau=double(t(:))/ref.duration;
end
tau=max(0,min(1,tau));
S=10*tau.^3-15*tau.^4+6*tau.^5;
[P,R]=cp_pose_at_s(ref,S);
end
