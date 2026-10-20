function run_qp_numeric_trial(snapshot,mode,algorithm,positionBound,orientationBound,trialTag)
if nargin<4, positionBound=.46; end
if nargin<5, orientationBound=.46; end
if nargin<6, trialTag=''; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
d=load(fullfile(here,[snapshot '_qp.mat']),'problem'); p=d.problem;
official=p;
p.b(p.rowGroup==4)=p.b(p.rowGroup==4)+positionBound-.46;
p.b(p.rowGroup==5)=p.b(p.rowGroup==5)+orientationBound-.46;
[s,tr]=transform_diagnostic_qp(p,mode); timer=tic;
result=struct('snapshot',snapshot,'mode',mode,'algorithm',algorithm,...
    'formal_thresholds_unchanged',true,'formal_QP_constraints_unchanged',positionBound==.46 && orientationBound==.46,...
    'TCP_internal_position_mm',positionBound,'TCP_internal_orientation_deg',orientationBound,...
    'vars',numel(p.f),'inequalities',numel(p.b),'equalities',numel(p.eq),...
    'transformed_vars',numel(s.f),'transformed_inequalities',numel(s.b),...
    'executionReady',false,'robot_motion','NO','status','STARTED');
name=[snapshot '_' mode '_' algorithm];
if positionBound~=.46 || orientationBound~=.46, name=[name sprintf('_pos%.3f_ori%.3f',positionBound,orientationBound)]; end
if ~isempty(trialTag), name=[name '_' trialTag]; end
path=fullfile(here,[name '.json']); writeJson(path,result);
result.conditioning=rmfield(tr,{'D','P','offset','free','retained_row_indices'});
result.matrix_stats=struct('H_diagonal_min',min(diag(p.H)),'H_diagonal_max',max(diag(p.H)),...
    'row_norm_min',min(full(max(abs(p.A),[],2))),'row_norm_max',max(full(max(abs(p.A),[],2))),...
    'time_step_min',min(diff(p.t)),'time_step_max',max(diff(p.t)));
z=[]; flag=NaN; output=struct('message','');
if strcmp(algorithm,'linprog')
    opts=optimoptions('linprog','Display','none','MaxTime',120,'ConstraintTolerance',1e-8);
    [z,~,flag,output]=linprog(zeros(numel(s.f),1),s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
else
    x0=[];
    if strcmp(algorithm,'active-set')
        lo=optimoptions('linprog','Display','none','MaxTime',90,'ConstraintTolerance',1e-8);
        [x0,~,lf,li]=linprog(zeros(numel(s.f),1),s.A,s.b,s.E,s.eq,s.lb,s.ub,lo);
        result.seed_LP=struct('exitflag',lf,'message',li.message);
        if lf<=0
            result.status='NO_FEASIBLE_SEED'; result.elapsed_sec=toc(timer); writeJson(path,result); return;
        end
    end
    opts=optimoptions('quadprog','Display','off','Algorithm',algorithm,'MaxIterations',300,...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8);
    [z,~,flag,output]=quadprog(s.H,s.f,s.A,s.b,s.E,s.eq,s.lb,s.ub,x0,opts);
end
result.exitflag=flag; result.solver_output=output; result.elapsed_sec=toc(timer);
result.status='NO_SOLUTION_ACCEPTED';
if flag>0 && ~isempty(z)
    x=tr.D.*(tr.offset+tr.P*z);
    residual=struct('inequality_max',max([0;p.A*x-p.b]),'equality_max',max([0;abs(p.Eq*x-p.eq)]),...
        'bound_max',max([0;p.lb-x;x-p.ub]));
    result.original_residual=residual; result.original_objective=.5*x'*p.H*x+p.f'*x;
    result.official_QP_inequality_violation=max([0;official.A*x-official.b]);
    result.original_constraints_pass=max(structfun(@(v)v,residual))<=1e-8;
    result.status='LINEAR_SOLUTION_ONLY';
    V=reshape(x,size(p.Q)); result.fixed_Q_TCP_time=true;
    ss=load(fullfile(here,'scene_snapshot.mat'),'scene'); scene=ss.scene;
    [tracking,~]=hmi_cubic_tracking_diagnostics(p.Q,p.P,p.t,V,scene,101);
    result.exact_tracking=tracking;
    % This isolated diagnostic never exports a trajectory or execution permit.
    result.exact_formal_precision_pass=tracking.position_max_mm<.5 &&...
        tracking.contour_max_mm<.5 && tracking.orientation_max_deg<=.5;
    result.formal_final_collision='NOT_RUN';
    executionReady=false; save(fullfile(here,[name '_candidate.mat']),...
        'V','result','executionReady','-v7.3');
end
save(fullfile(here,[name '_result.mat']),'result','-v7.3');
writeJson(path,result);
fprintf('QP_TRIAL %s %s %s flag=%g elapsed=%.6f\n',snapshot,mode,algorithm,flag,result.elapsed_sec);
end
function writeJson(path,value)
fid=fopen(path,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(qp_diag_json_safe(value),'PrettyPrint',true),'char');
end
