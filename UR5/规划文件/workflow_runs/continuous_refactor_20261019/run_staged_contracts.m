function run_staged_contracts(production)
if nargin<1, production=false; end
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT')));
if ~production, addpath(fullfile(here,'staged')); end
req=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(req.rawFile); z=load(raw.sceneFile,'scene'); scene=z.scene;
z=load(fullfile(here,'current','parameterization.mat')); c=z.candidate; tr=z.tr;
checks=struct(); report=struct('velocity_optimization',struct('pass',true),'stage_timings',struct());
bad=c; bad.qPath(end,1)=bad.qPath(end,1)+.01;
r=hmi_continuous_acceptance(bad,raw,scene,req,report);
checks.invalid_endpoint_keeps_precise_failure=~r.final_acceptance_pass && strcmp(r.acceptance_failure_identifier,'CONTINUOUS:Endpoints') && ~r.executionReady;
badraw=raw; badraw.metadata.realhome_version='REALHOME-000000';
r=hmi_continuous_acceptance(c,badraw,scene,req,report);
checks.stale_home_rejected_before_expensive_audit=~r.final_acceptance_pass && strcmp(r.acceptance_failure_identifier,'CONTINUOUS:HomeStale') && ~isfield(r,'controller_validation');
bad=c.pathTimeReference; bad.clockProgress(2)=-1;
try, hmi_continuous_reference(bad,.5); checks.invalid_clock_rejected=false;
catch e, checks.invalid_clock_rejected=strcmp(e.identifier,'CONTINUOUS:Reference'); end
badreq=req; badreq.raw_sha256='BAD'; badreq.outputDir=fullfile(here,'failure_fixture');
r=prepare_hmi_continuous_request(badreq);
checks.stale_raw_fails_closed=~r.success && ~r.executionReady && strcmp(r.failure_identifier,'CONTINUOUS:Stale') && strcmp(r.failed_stage,'INPUT_VALIDATION');
checks.no_export_on_failure=~isfield(r,'artifacts') && ~isfile(fullfile(badreq.outputDir,'offline_optimized_trajectory.mat'));
Q=repmat(raw.metadata.real_home_q,2,1); T=getTransform(scene.robot,Q(1,:),scene.tcpBodyName); P={T,T};
a=validate_accp_controller_interpolation(Q,P,[0;1],zeros(2,6),scene);
b=validate_accp_controller_interpolation(Q,P,[0;1],zeros(2,6),scene,struct('pathTimeReference',[]));
checks.legacy_reference_default_identical=isequaln(a,b) && a.complete && ~a.executionReady;
[p,~]=hmi_continuous_reference(c.pathTimeReference,c.timeFromStart);
checks.reference_finite=all(isfinite(p),'all');
checks.single_timing_owner_analytic_pass=tr.pass && tr.interface_C2_pass && tr.analytic_limits_pass;
for f=dir(fullfile(here,'staged','*.m')).'
    if production, file=fullfile(root,'matlab仿真',f.name); else, file=fullfile(f.folder,f.name); end
    issues=checkcode(file,'-id'); %#ok<NASGU>
end
result=struct('pass',all(structfun(@logical,checks)),'checks',checks,'executionReady',false,'robot_motion','NO');
if production, output='production_contracts.json'; else, output='staged_contracts.json'; end
fid=fopen(fullfile(here,output),'w'); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); disp(result); assert(result.pass);
end
