function test_collision_equivalence
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(req.rawFile); ss=load(raw.sceneFile,'scene'); scene=ss.scene;
rng(183,'twister'); lim=scene.jointLimits;
Q=[scene.qMat0;zeros(1,6);pi,-pi/2,0,-pi/2,0,0;lim(:,1).';lim(:,2).';...
    lim(:,1).'+rand(100,6).*(lim(:,2)-lim(:,1)).'];
flags=zeros(size(Q,1),2); maximumDistanceDifference=0; clock=tic;
for k=1:size(Q,1)
 [env,de]=check_scene_environment_collision(scene,Q(k,:));
 [self,ds]=checkCollision(scene.robot,Q(k,:),'IgnoreSelfCollision','off',...
  'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
 [~,gap,eh,sh,ne,ns]=combined_collision_copy(scene,Q(k,:));
 assert(isequal(isnan(de),isnan(ne)) && isequal(isinf(de),isinf(ne)) &&...
  isequal(isnan(ds),isnan(ns)) && isequal(isinf(ds),isinf(ns)),'EQUIV:Pairs','Pair matrices differ');
 assert(eh==any(env(:)) && sh==any(self(:)),'EQUIV:Flags','Collision flags differ');
 a=[de(:);ds(:)]; b=[ne(:);ns(:)]; finite=isfinite(a)&isfinite(b);
 maximumDistanceDifference=max(maximumDistanceDifference,max([0;abs(a(finite)-b(finite))]));
 d=a(isfinite(a)&a>0); if ~isempty(d), assert(abs(gap-1000*min(d))<1e-7); end
 flags(k,:)=[eh sh];
end
bad=scene; bad.environmentCollisionPolicy.body_index=1;
try, combined_collision_copy(bad,Q(1,:)); policyRejected=false; catch, policyRejected=true; end
r=struct('pass',maximumDistanceDifference<1e-10 && policyRejected,'states',size(Q,1),...
 'environment_collision_states',nnz(flags(:,1)),'self_collision_states',nnz(flags(:,2)),...
 'maximum_pair_distance_difference_m',maximumDistanceDifference,'bad_policy_rejected',policyRejected,...
 'elapsed_sec',toc(clock),'executionReady',false,'robot_motion','NO');
fid=fopen(fullfile(here,'collision_equivalence.json'),'w'); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(r,'PrettyPrint',true),'char'); disp(r); assert(r.pass);
end
