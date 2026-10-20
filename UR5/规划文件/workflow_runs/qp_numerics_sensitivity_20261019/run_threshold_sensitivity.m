function run_threshold_sensitivity(snapshot)
% ONLY DIAGNOSTIC TCP right-hand-side changes. Formal gates stay 0.5.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
d=load(fullfile(here,[snapshot '_qp.mat']),'problem'); original=d.problem;
trials={};
if strcmp(snapshot,'initial'), values=[.40 .46 .49 .50 .55 .65]; parts=[4 5];
else, values=[.46 .55 .65 .80]; parts=4; end
result=struct('snapshot',snapshot,'scope','ISOLATED_TCP_INTERNAL_RHS_DIAGNOSTIC_ONLY',...
    'formal_position_gate_mm',.5,'formal_orientation_gate_deg',.5,...
    'production_unchanged',true,'executionReady',false,'robot_motion','NO');
out=fullfile(here,[snapshot '_sensitivity.json']);
for group=parts
    for value=values
        p=original; mask=p.rowGroup==group; p.b(mask)=p.b(mask)+value-.46;
        [s,tr]=transform_diagnostic_qp(p,'scaled_eliminated');
        timer=tic; opts=optimoptions('linprog','Display','none','MaxTime',90,'ConstraintTolerance',1e-8);
        [z,~,flag,info]=linprog(zeros(numel(s.f),1),s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
        trial=struct('changed_TCP_group',group,'internal_bound',value,...
            'other_TCP_bound',.46,'exitflag',flag,'message',info.message,'elapsed_sec',toc(timer),...
            'linear_feasible',false,'executionReady',false,'collision_status','NOT_RUN');
        if flag>0 && ~isempty(z)
            x=tr.D.*(tr.offset+tr.P*z);
            trial.original_residual=max([0;p.A*x-p.b;abs(p.Eq*x-p.eq);p.lb-x;x-p.ub]);
            trial.linear_feasible=trial.original_residual<=1e-8;
        end
        trials{end+1}=trial; result.trials=trials; writeJson(out,result); %#ok<AGROW>
        fprintf('SENSITIVITY %s group=%d bound=%.2f flag=%d feasible=%d\n',snapshot,group,value,flag,trial.linear_feasible);
    end
end
% Minimum required TCP bound, not a candidate trajectory or relaxed gate.
minimums={};
for group=parts
    p=original; mask=double(p.rowGroup==group); n=numel(p.f);
    p.A=[p.A,-sparse(mask)]; p.b=p.b-.46*mask;
    p.Eq=[p.Eq,sparse(size(p.Eq,1),1)]; p.f=[zeros(n,1);1];
    p.H=blkdiag(p.H,1); p.lb=[p.lb;0]; p.ub=[p.ub;2]; p.x0=[p.x0;.46];
    [s,tr]=transform_diagnostic_qp(p,'scaled_eliminated');
    timer=tic; opts=optimoptions('linprog','Display','none','MaxTime',90,'ConstraintTolerance',1e-8);
    [z,~,flag,info]=linprog(s.f,s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
    r=struct('changed_TCP_group',group,'other_TCP_bound',.46,'exitflag',flag,...
        'message',info.message,'elapsed_sec',toc(timer),'executionReady',false);
    if flag>0 && ~isempty(z)
        x=tr.D.*(tr.offset+tr.P*z); r.minimum_internal_bound=x(end);
        r.original_residual=max([0;p.A*x-p.b;abs(p.Eq*x-p.eq);p.lb-x;x-p.ub]);
    end
    minimums{end+1}=r; result.minimum_bounds=minimums; writeJson(out,result); %#ok<AGROW>
end
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(qp_diag_json_safe(value),'PrettyPrint',true),'char');
end
