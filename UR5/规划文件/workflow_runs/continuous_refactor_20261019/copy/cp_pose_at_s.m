function [P,R,poses]=cp_pose_at_s(ref,S)
S=S(:); n=numel(S); P=zeros(n,3); R=zeros(3,3,n); poses=cell(1,n);
edges=discretize(S,[-Inf;ref.s(2:end-1);Inf]);
for k=1:n
    e=edges(k); u=max(0,min(1,(S(k)-ref.s(e))/(ref.s(e+1)-ref.s(e))));
    T=interpolate_tcp_pose_slerp(ref.poses{e},ref.poses{e+1},u);
    P(k,:)=T(1:3,4).'; R(:,:,k)=T(1:3,1:3); poses{k}=T;
end
end
