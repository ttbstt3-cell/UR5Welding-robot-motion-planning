function [P,R,S]=cp_reference(ref,t)
% Independent frozen original Cartesian geometry + declared quintic clock.
tau=max(0,min(1,double(t(:))/ref.duration));
S=10*tau.^3-15*tau.^4+6*tau.^5;
[P,R]=cp_pose_at_s(ref,S);
end
