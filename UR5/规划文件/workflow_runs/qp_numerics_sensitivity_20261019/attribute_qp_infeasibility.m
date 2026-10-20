function attribute_qp_infeasibility(snapshot)
% Constraint SUBSET LPs are attribution only, never planning candidates.
here=fileparts(mfilename('fullpath')); d=load(fullfile(here,[snapshot '_qp.mat']),'problem'); p=d.problem;
names={'physical_only','physical_and_position','physical_and_orientation','all_constraints',...
    'all_with_rest_equalities_only','all_with_join_equalities_only'};
results={};
for k=1:numel(names)
    a=p;
    if k==1, keep=a.rowGroup<=3;
    elseif k==2, keep=a.rowGroup~=5;
    elseif k==3, keep=a.rowGroup~=4;
    else, keep=true(size(a.b)); end
    a.A=a.A(keep,:); a.b=a.b(keep);
    if k==5, a.Eq=a.Eq(1:12,:); a.eq=a.eq(1:12);
    elseif k==6, a.Eq=a.Eq(13:end,:); a.eq=a.eq(13:end); end
    r=solve(a); r.name=names{k}; r.scope='SUBSET_FEASIBILITY_NOT_AN_ACCEPTED_PATH';
    results{end+1}=r; %#ok<AGROW>
    fprintf('ATTRIBUTION %s %s flag=%d\n',snapshot,names{k},r.exitflag);
end
joint={};
for threshold=[.46 .49 .5 .55 .65 .8 1 2 5]
    a=p; mask=a.rowGroup>=4; a.b(mask)=a.b(mask)+threshold-.46;
    r=solve(a); r.position_bound_mm=threshold; r.orientation_bound_deg=threshold;
    r.scope='DIAGNOSTIC_BOTH_TCP_RHS_ONLY'; joint{end+1}=r; %#ok<AGROW>
    fprintf('JOINT_SENSITIVITY %s bound=%.2f flag=%d\n',snapshot,threshold,r.exitflag);
end
% Minimum common numerical bound, different physical units reported as such.
a=p; n=numel(a.f); mask=double(a.rowGroup>=4);
a.A=[a.A,-sparse(mask)]; a.b=a.b-.46*mask;
a.Eq=[a.Eq,sparse(size(a.Eq,1),1)]; a.H=blkdiag(a.H,1);
a.f=[zeros(n,1);1]; a.lb=[a.lb;0]; a.ub=[a.ub;1000]; a.x0=[a.x0;.46];
[s,tr]=transform_diagnostic_qp(a,'scaled_eliminated'); opts=optimoptions('linprog','Display','none','MaxTime',120,'ConstraintTolerance',1e-8);
[z,~,flag,info]=linprog(s.f,s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
minimum=struct('exitflag',flag,'message',info.message);
if flag>0
    x=tr.D.*(tr.offset+tr.P*z); minimum.bound_numeric_mm_and_deg=x(end);
    minimum.original_residual=max([0;a.A*x-a.b;abs(a.Eq*x-a.eq);a.lb-x;x-a.ub]);
end
matrixStats=struct('H_condest_raw',condest(p.H)); [s,~]=transform_diagnostic_qp(p,'scaled_eliminated');
matrixStats.H_condest_scaled_eliminated=condest(s.H);
report=struct('snapshot',snapshot,'subset_results',{results},'both_TCP_sensitivity',{joint},...
    'minimum_common_bound',minimum,'matrix_stats',matrixStats,'executionReady',false,'robot_motion','NO');
fid=fopen(fullfile(here,[snapshot '_attribution.json']),'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(qp_diag_json_safe(report),'PrettyPrint',true),'char');
end
function r=solve(a)
[s,tr]=transform_diagnostic_qp(a,'scaled_eliminated'); clock=tic;
opts=optimoptions('linprog','Display','none','MaxTime',90,'ConstraintTolerance',1e-8);
[z,~,flag,info]=linprog(zeros(numel(s.f),1),s.A,s.b,s.E,s.eq,s.lb,s.ub,opts);
r=struct('exitflag',flag,'message',info.message,'elapsed_sec',toc(clock),'executionReady',false);
if flag>0 && ~isempty(z)
    x=tr.D.*(tr.offset+tr.P*z);
    r.original_residual=max([0;a.A*x-a.b;abs(a.Eq*x-a.eq);a.lb-x;x-a.ub]);
end
end
