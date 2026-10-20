function [s,transform]=transform_diagnostic_qp(p,mode)
% Algebraically equivalent transformations ONLY. Physical residual is always
% checked with the original p; not a change to any acceptance threshold.
n=numel(p.f); d=ones(n,1); R=ones(size(p.b)); RE=ones(size(p.eq));
if ~strcmp(mode,'raw')
    d=1./sqrt(max(abs(diag(p.H)),1e-20));
    AD=p.A*spdiags(d,0,n,n); ED=p.Eq*spdiags(d,0,n,n);
    R=1./max([full(max(abs(AD),[],2)),abs(p.b),1e-12*ones(size(p.b))],[],2);
    if ~isempty(p.eq), RE=1./max([full(max(abs(ED),[],2)),abs(p.eq),1e-12*ones(size(p.eq))],[],2); end
end
D=spdiags(d,0,n,n); H=D*p.H*D; f=d.*p.f;
objectiveScale=1;
if ~strcmp(mode,'raw'), objectiveScale=max(1,norm(H,inf)); end
H=H/objectiveScale; f=f/objectiveScale;
H=(H+H')/2;
A=spdiags(R,0,numel(R),numel(R))*p.A*D; b=R.*p.b;
E=spdiags(RE,0,numel(RE),numel(RE))*p.Eq*D; eq=RE.*p.eq;
lb=p.lb./d; ub=p.ub./d; x0=p.x0./d;
P=speye(n); offset=zeros(n,1); free=1:n;
if strcmp(mode,'scaled_eliminated') && ~isempty(eq)
    [~,RR,permutation]=qr(full(E),'econ','vector');
    assert(rank(RR)==size(E,1),'Dependent equalities require separate diagnosis');
    piv=permutation(1:size(E,1)); free=setdiff(1:n,piv,'stable');
    Ep=E(:,piv); Ef=E(:,free); offset(piv)=Ep\eq;
    P=sparse(n,numel(free)); P(free,:)=speye(numel(free)); P(piv,:)=-Ep\Ef;
    f=P'*(H*offset+f); H=P'*H*P; H=(H+H')/2;
    reducedB=b-A*offset;
    A=[A*P;P(piv,:);-P(piv,:)]; b=[reducedB;ub(piv)-offset(piv);offset(piv)-lb(piv)];
    E=sparse(0,numel(free)); eq=[]; lb=lb(free); ub=ub(free); x0=x0(free);
end
% Exact removal of redundant constants and singleton rows, carrying every
% singleton inequality into bounds. No approximate general row pruning.
counts=sum(spones(A),2); constants=find(counts==0);
constantInfeasible=any(b(constants)<0);
single=find(counts==1);
[ii,jj,vv]=find(A(single,:)); rhs=b(single(ii))./vv;
if ~strcmp(mode,'raw')
    for k=1:numel(ii)
        if vv(k)>0, ub(jj(k))=min(ub(jj(k)),rhs(k));
        else, lb(jj(k))=max(lb(jj(k)),rhs(k)); end
    end
end
if strcmp(mode,'raw'), keep=true(size(b)); constants=[]; single=[];
else, keep=counts>1; end
s=struct('H',H,'f',f,'A',A(keep,:),'b',b(keep),'E',E,'eq',eq,'lb',lb,'ub',ub,'x0',x0);
transform=struct('D',d,'P',P,'offset',offset,'free',free,'row_scale_min',min(R),...
    'row_scale_max',max(R),'removed_constant_rows',numel(constants),...
    'singleton_rows_to_bounds',numel(single),'constant_infeasible',constantInfeasible,...
    'bound_infeasible',any(lb>ub),'objective_scale',objectiveScale,...
    'retained_row_indices',find(keep));
end
