function locate_minimum_position_bound()
here=fileparts(mfilename('fullpath')); d=load(fullfile(here,'initial_qp.mat'),'problem'); p=d.problem;
input=load(fullfile(here,'initial_input.mat'),'labels','joins');
ss=load(fullfile(here,'scene_snapshot.mat'),'scene'); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真'));
n=numel(p.f); a=p; mask=double(p.rowGroup==4);
a.A=[a.A,-sparse(mask)]; a.b=a.b-.46*mask; a.Eq=[a.Eq,sparse(size(a.Eq,1),1)];
a.f=[zeros(n,1);1]; a.H=blkdiag(a.H,1); a.lb=[a.lb;0]; a.ub=[a.ub;10]; a.x0=[a.x0;.46];
[s,tr]=transform_diagnostic_qp(a,'scaled'); timer=tic;
opts=optimoptions('linprog','Display','none','MaxTime',120,'ConstraintTolerance',1e-8);
[z,~,flag,info,lambda]=linprog(s.f,s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
r=struct('scope','LINEARIZED_TCP_POSITION_BOUND_ATTRIBUTION_ONLY','exitflag',flag,...
    'message',info.message,'elapsed_sec',toc(timer),'orientation_bound_deg',.46,...
    'auxiliary_bound_upper_mm',10,'executionReady',false);
if flag>0
    x=tr.D.*(tr.offset+tr.P*z); r.minimum_position_bound_mm=x(end);
    r.original_residual=max([0;a.A*x-a.b;abs(a.Eq*x-a.eq);a.lb-x;x-a.ub]);
    [~,order]=sort(abs(lambda.ineqlin),'descend'); top={};
    for k=1:min(30,numel(order))
        row=tr.retained_row_indices(order(k)); e=double(p.rowEdge(row));
        top{end+1}=struct('original_row',row,'group',p.rowGroup(row),'edge',e,...
            'u',p.rowU(row),'segment',input.labels{min(e+1,numel(input.labels))},...
            'scaled_dual_magnitude',abs(lambda.ineqlin(order(k)))); %#ok<AGROW>
    end
    r.top_active_rows=top;
end
% Fixed-node and edge geometry at start/end and both task joins.
edges=unique([1;size(p.Q,1)-1;input.joins(:)-1;input.joins(:)]); summaries={};
for e=edges(:).'
    T=p.P{e}; U=p.P{e+1};
    summaries{end+1}=struct('edge',e,'label',input.labels{e+1},'dt',p.t(e+1)-p.t(e),...
        'TCP_chord_mm',1000*norm(U(1:3,4)-T(1:3,4)),...
        'TCP_rotation_deg',rad2deg(acos(max(-1,min(1,(trace(U(1:3,1:3)*T(1:3,1:3)')-1)/2)))),...
        'q_delta_deg',rad2deg(p.Q(e+1,:)-p.Q(e,:)),...
        'v_start',p.seed(e,:),'v_end',p.seed(e+1,:)); %#ok<AGROW>
end
r.boundary_edges=summaries;
[quick,~]=hmi_cubic_tracking_diagnostics(p.Q,p.P,p.t,p.seed,ss.scene,101,input.labels);
r.initial_seed_tracking=quick;
fid=fopen(fullfile(here,'minimum_position_attribution.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(qp_diag_json_safe(r),'PrettyPrint',true),'char');
fprintf('MINIMUM_POSITION_BOUND flag=%d\n',flag);
end
