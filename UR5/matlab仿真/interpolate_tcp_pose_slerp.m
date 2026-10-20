function T = interpolate_tcp_pose_slerp(T1,T2,s)
%INTERPOLATE_TCP_POSE_SLERP Linear XYZ and shortest quaternion SLERP.
validateattributes(s,{'numeric'},{'scalar','real','finite','>=',0,'<=',1});
if s==0,T=T1;return;end
if s==1,T=T2;return;end
q1=rotm2quat(T1(1:3,1:3));q2=rotm2quat(T2(1:3,1:3));
c=dot(q1,q2);if c<0,q2=-q2;c=-c;end
c=max(-1,min(1,c));theta=acos(c);
if theta<1e-8
    q=(1-s)*q1+s*q2;
else
    q=sin((1-s)*theta)/sin(theta)*q1+sin(s*theta)/sin(theta)*q2;
end
q=q/norm(q);T=eye(4);T(1:3,1:3)=quat2rotm(q);
T(1:3,4)=(1-s)*T1(1:3,4)+s*T2(1:3,4);
end
