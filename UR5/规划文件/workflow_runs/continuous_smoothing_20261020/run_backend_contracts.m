function run_backend_contracts(production)
if nargin<1, production=false; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
if ~production, addpath(fullfile(here,'staged')); end
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(req.rawFile); z=load(raw.sceneFile,'scene'); scene=z.scene;
checks=struct();
Q=repmat(scene.qMat0,2,1); T=getTransform(scene.robot,Q(1,:),scene.tcpBodyName); P={T,T};
opts=struct('collisionWorkers',0);
a=validate_accp_controller_interpolation(Q,P,[0;1],zeros(2,6),scene,opts);
opts.pathTimeReference=[]; b=validate_accp_controller_interpolation(Q,P,[0;1],zeros(2,6),scene,opts);
checks.default_reference_unchanged=isequaln(rmfield(a,'performance'),rmfield(b,'performance')) && a.complete;
checks.all_101_states_checked=a.collision_sample_count==101 && a.performance.collision_batch.processed_states==101;
try, validate_accp_controller_interpolation(Q,P,[0;1],zeros(2,6),scene,struct('collisionWorkers',NaN)); checks.bad_worker_option=false;
catch e, checks.bad_worker_option=strcmp(e.identifier,'AUDIT:Workers'); end
bad=Q; bad(1,1)=NaN;
try, validate_accp_controller_interpolation(bad,P,[0;1],zeros(2,6),scene,opts); checks.nan_q_rejected=false;
catch e, checks.nan_q_rejected=strcmp(e.identifier,'ACCP_CONTROLLER:QPath'); end
poly=[0 0 0;1 0 0;1 0 0;1 1 0]; A=[repmat(.25,513,1),linspace(.499999999,.500000001,513).'/1000,zeros(513,1)];
new=contour_distance_copy(poly,A,.5); old=zeros(size(new));
for i=1:size(A,1)
    best=Inf;
    for k=1:size(poly,1)-1
        v=poly(k+1,:)-poly(k,:); u=max(0,min(1,dot(A(i,:)-poly(k,:),v)/max(dot(v,v),eps)));
        best=min(best,norm(A(i,:)-(poly(k,:)+u*v)));
    end
    old(i)=1000*best;
end
checks.contour_boundary_decision_exact=isequal(old>=.5,new>=.5) && isequal(old,new);
checks.validator_threshold_dispatch_exact=true;
for delta=[-1e-12 0 1e-12]
    U=T; U(3,4)=U(3,4)+(.5+delta)/1000;
    original=baseline_audit_profile(Q,{U,U},[0;1],zeros(2,6),scene);
    fast=validate_accp_controller_interpolation(Q,{U,U},[0;1],zeros(2,6),scene,opts);
    checks.validator_threshold_dispatch_exact=checks.validator_threshold_dispatch_exact &&...
        original.complete==fast.complete && original.max_tcp_contour_error_mm==fast.max_tcp_contour_error_mm;
end
empty=scene; empty.environment={}; empty=rmfield(empty,'environmentCollisionPolicy');
[~,gap,env,self]=hmi_audit_collision_state(empty,Q(1,:));
[s,dist]=checkCollision(scene.robot,Q(1,:),'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
d=dist(isfinite(dist)&dist>0);
checks.empty_world_compatible=~env && self==any(s(:)) && abs(gap-1000*min(d))<1e-9;
% Error after a local worker starts must unwind, never certify partial data.
bad=scene; bad.environmentCollisionPolicy.body_index=1;
try, hmi_audit_collision_batch(bad,repmat(Q(1,:),5000,1),4); checks.worker_error_fails_closed=false;
catch, checks.worker_error_fails_closed=isempty(gcp('nocreate')); end
% A pool owned by another caller is neither reused nor deleted.
tiny=struct(); tiny.robot=rigidBodyTree('DataFormat','row');
parent=tiny.robot.BaseName;
for j=1:6
    body=rigidBody(sprintf('test%d',j)); joint=rigidBodyJoint(sprintf('j%d',j),'revolute');
    setFixedTransform(joint,trvec2tform([0 0 1])); body.Joint=joint;
    addCollision(body,collisionSphere(.01)); addBody(tiny.robot,body,parent); parent=body.Name;
end
box=collisionBox(.01,.01,.01); box.Pose=trvec2tform([10 10 10]); tiny.environment={box}; tiny.selfCollisionSkipPairs='parent';
pool=parpool('Processes',2); cleanup=onCleanup(@()delete(pool));
[env,self,gaps,info]=hmi_audit_collision_batch(tiny,zeros(5000,6),4);
checks.external_pool_retained=~isempty(gcp('nocreate')) && pool.NumWorkers==2 && strcmp(info.mode,'serial_combined') && contains(info.parallel_unavailable_reason,'External pool');
checks.serial_full_coverage=info.processed_states==5000 && ~any(env) && ~any(self) && all(isfinite(gaps));
r=struct('pass',all(structfun(@logical,checks)),'checks',checks,'executionReady',false,'robot_motion','NO');
if production, file='production_backend_contracts.json'; else, file='backend_contracts.json'; end
fid=fopen(fullfile(here,file),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(r,'PrettyPrint',true),'char'); disp(r); assert(r.pass);
end
