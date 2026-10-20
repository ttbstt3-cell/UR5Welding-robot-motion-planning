function test_qp_equivalence()
here=fileparts(mfilename('fullpath')); rng(123,'twister');
n=8; p.H=sparse(diag(logspace(-2,2,n))); p.f=[-.1;-.02;-.03;-.5;-.01;-.03;-.01;-.2];
p.A=[speye(n);-speye(n);sparse(ones(1,n));sparse(1,n)]; p.b=[ones(2*n,1);2;1];
p.Eq=sparse([1 0 0 0 0 0 0 0;0 1 1 0 0 0 0 0]); p.eq=[.05;.2];
p.lb=-ones(n,1); p.ub=ones(n,1); p.x0=zeros(n,1);
objectives=zeros(1,3); xs=cell(1,3); modes={'raw','scaled','scaled_eliminated'};
for k=1:3
    [s,tr]=transform_diagnostic_qp(p,modes{k});
    opts=optimoptions('quadprog','Display','off','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);
    [z,~,flag]=quadprog(s.H,s.f,s.A,s.b,s.E,s.eq,s.lb,s.ub,[],opts); assert(flag>0);
    x=tr.D.*(tr.offset+tr.P*z); xs{k}=x;
    assert(max([0;p.A*x-p.b;abs(p.Eq*x-p.eq);p.lb-x;x-p.ub])<1e-8);
    objectives(k)=.5*x'*p.H*x+p.f'*x;
end
assert(max(abs(objectives-objectives(1)))<1e-8);
assert(max(abs(xs{1}-xs{3}))<1e-5);
d=load(fullfile(here,'initial_qp.mat'),'problem'); q=d.problem;
[s,tr]=transform_diagnostic_qp(q,'scaled_eliminated');
z=randn(numel(s.f),1)*1e-3; x=tr.D.*(tr.offset+tr.P*z);
res=max(abs(q.Eq*x-q.eq)); assert(res<1e-10);
report=struct('pass',true,'synthetic_objectives',objectives,'real_equality_mapping_max_residual',res,...
    'scope','ALGEBRAIC_TRANSFORM_TEST_NOT_A_ROBOT_PLAN','executionReady',false);
fid=fopen(fullfile(here,'transformation_tests.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char'); disp('QP_TRANSFORMATION_EQUIVALENCE_PASS');
end
