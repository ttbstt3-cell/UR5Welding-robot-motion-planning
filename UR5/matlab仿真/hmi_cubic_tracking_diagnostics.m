function [r,s]=hmi_cubic_tracking_diagnostics(Q,P,t,V,scene,m,labels,pathTimeReference)
% FK-only optimizer diagnostics. NEVER substitutes for collision validation.
if nargin<6, m=31; end
if nargin<7, labels=repmat({'weld'},1,size(Q,1)); end
if nargin<8, pathTimeReference=[]; end
t=t(:); n=size(Q,1); M=1+(n-1)*(m-1);
referenceP=[]; referenceR=[];
if ~isempty(pathTimeReference)
    ts=[];
    for edge=1:n-1
        u=linspace(0,1,m).'; if edge>1, u=u(2:end); end
        ts=[ts;t(edge)+u*(t(edge+1)-t(edge))]; %#ok<AGROW>
    end
    [referenceP,referenceR]=hmi_continuous_reference(pathTimeReference,ts);
end
s=struct('q',zeros(M,6),'time',zeros(M,1),'edge',zeros(M,1),'u',zeros(M,1),...
    'actualPose',zeros(4,4,M),'targetPose',zeros(4,4,M),...
    'positionError',zeros(M,1),'orientationError',zeros(M,1),'contourError',zeros(M,1));
anchors=cell2mat(cellfun(@(T)T(1:3,4).',P(:),'UniformOutput',false));
av=diff(anchors); aa=sum(av.^2,2); k=0;
for e=1:n-1
    h=t(e+1)-t(e); D=Q(e+1,:)-Q(e,:);
    for j=1:m
        if e>1 && j==1, continue; end
        k=k+1; u=(j-1)/(m-1);
        q=Q(e,:)+(3*u^2-2*u^3)*D+h*(u^3-2*u^2+u)*V(e,:)+h*(u^3-u^2)*V(e+1,:);
        A=getTransform(scene.robot,q,scene.tcpBodyName);
        if isempty(pathTimeReference)
            G=interpolate_tcp_pose_slerp(P{e},P{e+1},u);
        else
            G=eye(4); G(1:3,4)=referenceP(k,:).'; G(1:3,1:3)=referenceR(:,:,k);
        end
        s.q(k,:)=q; s.time(k)=t(e)+u*h; s.edge(k)=e; s.u(k)=u;
        s.actualPose(:,:,k)=A; s.targetPose(:,:,k)=G;
        s.positionError(k)=1000*norm(A(1:3,4)-G(1:3,4));
        s.orientationError(k)=rad2deg(acos(max(-1,min(1,(trace(A(1:3,1:3)*G(1:3,1:3)')-1)/2))));
        z=max(0,min(1,sum((A(1:3,4).'-anchors(1:end-1,:)).*av,2)./max(aa,eps)));
        s.contourError(k)=1000*min(vecnorm(A(1:3,4).'-anchors(1:end-1,:)-z.*av,2,2));
    end
end
[pe,ip]=max(s.positionError); [oe,io]=max(s.orientationError); [ce,ic]=max(s.contourError);
r=struct('position_max_mm',pe,'position_rms_mm',sqrt(mean(s.positionError.^2)),...
    'orientation_max_deg',oe,'orientation_rms_deg',sqrt(mean(s.orientationError.^2)),...
    'contour_max_mm',ce,'contour_rms_mm',sqrt(mean(s.contourError.^2)),...
    'position_case',sampleCase(ip),'orientation_case',sampleCase(io),'contour_case',sampleCase(ic),...
    'derivatives',accp_cubic_derivative_metrics(Q,t,V),'duration_sec',t(end)-t(1),...
    'collision_status','NOT_CHECKED','executionReady',false,'sample_count',M);
    function c=sampleCase(i)
        c=struct('time_sec',s.time(i),'path_segment_index',s.edge(i),'u',s.u(i),...
            'task_segment',labels{min(s.edge(i)+1,n)},'q_rad',s.q(i,:),'q_deg',rad2deg(s.q(i,:)),...
            'actual_TCP_pose',s.actualPose(:,:,i),'expected_TCP_pose',s.targetPose(:,:,i),...
            'frame_id',scene.robot.BaseName,'position_error_mm',s.positionError(i),...
            'orientation_error_deg',s.orientationError(i));
    end
end
