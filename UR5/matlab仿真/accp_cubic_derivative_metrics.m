function m=accp_cubic_derivative_metrics(Q,t,V,window)
% Exact piecewise cubic metrics. Segment jerk excludes acceleration jumps.
if nargin<4, window=[t(1) t(end)]; end
h=diff(t(:)); n=numel(h); D=diff(Q); bb=h.*V(1:end-1,:);
cc=3*D-2*bb-h.*V(2:end,:); dd=-2*D+bb+h.*V(2:end,:);
a0=2*cc./h.^2; a1=(2*cc+6*dd)./h.^2; jerk=6*dd./h.^3;
jumps=a0(2:end,:)-a1(1:end-1,:);
vmax=max(abs(V),[],1); qmin=min(Q,[],1); qmax=max(Q,[],1);
variation=zeros(1,6); reversals=zeros(1,6);
for e=1:n
    lo=max(0,(window(1)-t(e))/h(e)); hi=min(1,(window(2)-t(e))/h(e));
    for j=1:6
        if abs(dd(e,j))>eps
            u=-cc(e,j)/(3*dd(e,j));
            if u>0 && u<1
                vmax(j)=max(vmax(j),abs(bb(e,j)+2*cc(e,j)*u+3*dd(e,j)*u^2)/h(e));
            end
        end
        rootsU=roots([3*dd(e,j),2*cc(e,j),bb(e,j)]);
        for u=rootsU(:).'
            if isreal(u) && u>0 && u<1
                q=Q(e,j)+bb(e,j)*u+cc(e,j)*u^2+dd(e,j)*u^3;
                qmin(j)=min(qmin(j),q); qmax(j)=max(qmax(j),q);
            end
        end
        if hi<=lo, continue; end
        cuts=[lo hi];
        if abs(a1(e,j)-a0(e,j))>eps
            u=-a0(e,j)/(a1(e,j)-a0(e,j));
            if u>lo && u<hi, cuts=[lo u hi]; reversals(j)=reversals(j)+1; end
        end
        for k=1:numel(cuts)-1
            v=@(u)(bb(e,j)+2*cc(e,j)*u+3*dd(e,j)*u^2)/h(e);
            variation(j)=variation(j)+abs(v(cuts(k+1))-v(cuts(k)));
        end
    end
end
T=sum(h);
m=struct('max_velocity_per_joint',vmax,'max_velocity',max(vmax),...
    'max_acceleration_per_joint',max(abs([a0;a1]),[],1),...
    'max_acceleration',max(abs([a0(:);a1(:)])),...
    'rms_acceleration_per_joint',sqrt(sum(h.*(a0.^2+a0.*a1+a1.^2)/3,1)/T),...
    'rms_acceleration',sqrt(sum(h.*sum(a0.^2+a0.*a1+a1.^2,2)/3)/(6*T)),...
    'max_segment_jerk_per_joint',max(abs(jerk),[],1),...
    'max_segment_jerk',max(abs(jerk(:))),...
    'rms_segment_jerk_per_joint',sqrt(sum(h.*jerk.^2,1)/T),...
    'rms_segment_jerk',sqrt(sum(h.*sum(jerk.^2,2))/(6*T)),...
    'max_acceleration_jump_per_joint',max(abs(jumps),[],1),...
    'max_acceleration_jump',max(abs(jumps(:))),...
    'rms_acceleration_jump',sqrt(mean(jumps(:).^2)),...
    'acceleration_start',a0,'acceleration_end',a1,'segment_jerk',jerk,...
    'node_acceleration_jump',jumps,'position_min',qmin,'position_max',qmax,...
    'focus_window_sec',window,'focus_velocity_total_variation',variation,...
    'focus_velocity_extrema_count',reversals,...
    'jerk_scope','SEGMENT_INTERIOR_ONLY; NODE_ACCELERATION_JUMPS_REPORTED_SEPARATELY');
end
