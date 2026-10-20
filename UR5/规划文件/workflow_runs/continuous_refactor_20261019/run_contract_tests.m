function run_contract_tests
% Offline data contracts. Never builds a ROS node or execution artifact.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(fullfile(here,'copy'));
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(req.rawFile); x=load(raw.sceneFile,'scene'); scene=x.scene;
z=load(fullfile(here,'current','geometry.mat')); g=z.g;
x=load(fullfile(here,'current','parameterization.mat')); c=x.candidate;
checks=struct();
checks.original_weld_q_exact=isequal(c.qPath(c.maps{2},:),raw.segments{2}.qPath);
checks.original_weld_tcp_exact=isequal(reshape(c.tcpPosePath(c.maps{2}),1,[]),reshape(raw.segments{2}.tcpPosePath,1,[]));
checks.original_all_q_exact=true;
for k=1:3, checks.original_all_q_exact=checks.original_all_q_exact && isequal(c.qPath(c.maps{k},:),raw.segments{k}.qPath); end
checks.rest_end_velocities=all(c.velocities([1 end],:)==0,'all');
dm=accp_cubic_derivative_metrics(c.qPath,c.timeFromStart,c.velocities);
checks.rest_end_acceleration=max(abs([dm.acceleration_start(1,:),dm.acceleration_end(end,:)]))<1e-8;
checks.interface_C2=all(abs(dm.node_acceleration_jump(c.joins-1,:))<1e-6,'all');
checks.time_strictly_increasing=all(diff(c.timeFromStart)>0);
checks.execution_false=isequal(c.executionReady,false);
% Reference independence: a corrupt joint state cannot rewrite the target.
[P1,R1]=cp_reference(c.pathTimeReference,c.timeFromStart);
bad=c; bad.qPath(10,1)=bad.qPath(10,1)+.1;
[P2,R2]=cp_reference(bad.pathTimeReference,bad.timeFromStart);
checks.reference_independent_of_candidate=isequal(P1,P2)&&isequal(R1,R2);
u=1e-4; T=c.timeFromStart(end);
[pa,~]=cp_reference(c.pathTimeReference,[T*(1-u);T]);
checks.home_reference_decelerates=norm(diff(pa))/(u*T)<1e-5;
badraw=raw; badraw.segments{2}.tcpPosePath{5}(1,4)=badraw.segments{2}.tcpPosePath{5}(1,4)+.002;
try, cp_geometry(badraw,scene); checks.mismatched_fixed_target_rejected=false;
catch e, checks.mismatched_fixed_target_rejected=strcmp(e.identifier,'CP:OriginalGeometry'); end
badraw=raw; badraw.segments{2}.qPath(1,6)=badraw.segments{2}.qPath(1,6)+2*pi;
try, cp_geometry(badraw,scene); checks.turn_boundary_rejected=false;
catch e, checks.turn_boundary_rejected=strcmp(e.identifier,'CP:Boundary'); end
checks.original_cartesian_reference_exact=isequal(g.reference.poses,g.originalP);
r=struct('checks',checks,'pass',all(structfun(@(v)logical(v),checks)),...
    'scope','Data/FK/analytic contracts only; not a collision or execution approval','executionReady',false);
fid=fopen(fullfile(here,'contract_tests.json'),'w'); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(r,'PrettyPrint',true),'char');
disp(r); assert(r.pass,'CP:Contract','Independent module contract test failed');
end
