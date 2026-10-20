function report=diagnose_hmi_interface_branch_repair(sourceDir,outputDir)
% Offline interface-only recovery. No ACCP/IK/TCP/geometry/control edits.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
addpath(here); addpath(genpath(fullfile(fileparts(root),'RRT'))); setenv('RRT_HEADLESS','1');
if ~isfolder(outputDir), mkdir(outputDir); end
raw=load(fullfile(sourceDir,'full_raw_trajectory.mat'));
ss=load(raw.sceneFile,'scene'); scene=ss.scene;
assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256));
report=struct('executionReady',false,'robot_motion','NO','source_directory',sourceDir,...
    'collision_scene_version',raw.metadata.calibration_binding.collision_scene_version);
current=hmi_planning_dependency_snapshot(root);
report.dependency_validation=hmi_planning_dependency_changes(scene.planningDependencySnapshot,current);
assert(report.dependency_validation.pass,'INTERFACE:STALE','Source scene dependencies changed');
request=jsondecode(fileread(fullfile(sourceDir,'request.json')));
a=raw.segments{1}; w=raw.segments{2}; x=raw.segments{3};
report.before=boundary(scene,a.qPath(end,:),w.qPath(1,:));
report.before.exit_home_numeric_delta_deg=rad2deg(x.qPath(end,:)-request.qHome(:).');
report.before.approach_joint_limits_pass=all(a.qPath>=scene.jointLimits(:,1).' & a.qPath<=scene.jointLimits(:,2).','all');
report.weld_q_nodes_preserved=true; report.weld_node_count=size(w.qPath,1);
report.weld_q_sha256=hmi_workflow_file_hash(fullfile(sourceDir,'preview_accp.mat'));
report.constant_turn_candidates=constantCandidates(a.qPath,w.qPath,request.qStart(:).',scene.jointLimits);
writeJson(fullfile(outputDir,'interface_diagnosis.json'),report);
parts={a,w,x};
for label={'approach','exit'}
    name=label{1}; original=load(fullfile(sourceDir,[name '_planner.mat']),'p');
    if strcmp(name,'approach'), start=request.qStart(:).'; goal=w.qPath(1,:); index=1;
    else, start=w.qPath(end,:); goal=request.qHome(:).'; index=3; end
    tries={}; accepted=false; bestCost=Inf; bestQ=[];
    for attempt=0:3
        timer=tic;
        if attempt==0, p=original.p; seed=request.seed;
        else
            seed=request.seed+attempt;
            cfg=struct('scene',scene,'start',start,'goal',goal,'seed',seed,...
                'maxIterations',1000000,'maxPlanningTimeSec',request.maxPlanningTimeSec,...
                'maxJointStepRad',deg2rad(12),'jointEdgeResolutionRad',deg2rad(2),...
                'goalBias',0.2,'anytime',true,'continueAfterFirstSolution',true,...
                'checkpointTimesSec',[5 10 15 20 30]);
            p=FastConnectAnytimeRRTStarJoint(cfg);
        end
        br=struct('pass',false,'failure_reason','PLANNER_FAILED'); Q=[];
        if p.success && ~isempty(p.path), [Q,br]=hmi_checked_free_path_branch(scene,p.path,start,goal); end
        tries{end+1}=struct('attempt',attempt,'seed',seed,'planner_success',p.success,...
            'branch_audit',br,'elapsed_sec',toc(timer)); %#ok<AGROW>
        save(fullfile(outputDir,sprintf('%s_attempt_%d.mat',name,attempt)),'p','br','-v7');
        report.([name '_trials'])=tries; writeJson(fullfile(outputDir,'interface_diagnosis.json'),report);
        if br.pass
            accepted=true; cost=sum(vecnorm(diff(Q),2,2));
            if cost<bestCost, bestCost=cost; bestQ=Q; end
            % At least one fresh approach search is required for the pinned
            % ACCP branch, even if the old long numeric sweep was safe.
            if attempt>0 && ~br.actual_long_rotation_revalidated, break; end
        end
    end
    if ~accepted, report.failure_reason=[name ': no legal branch']; writeJson(fullfile(outputDir,'interface_diagnosis.json'),report); return; end
    parts{index}=segment(scene,bestQ,name); parts{index}.source_binding=raw.metadata.calibration_binding;
end
report.after=boundary(scene,parts{1}.qPath(end,:),parts{2}.qPath(1,:));
report.after.weld_exit=boundary(scene,parts{2}.qPath(end,:),parts{3}.qPath(1,:));
report.after.exit_home_numeric_delta_deg=rad2deg(parts{3}.qPath(end,:)-request.qHome(:).');
report.after.approach_weld_transition=hmi_audit_joint_interface_path(scene,...
    [parts{1}.qPath(end-1:end,:);parts{2}.qPath(1:2,:)],101);
assert(isequal(parts{2},w),'ACCP weld segment must be unchanged');
file=fullfile(outputDir,'full_raw_trajectory.mat');
fresh=assemble_hmi_preview_trajectory(parts,raw.metadata.calibration_binding,raw.sceneFile,file);
metadata=raw.metadata; metadata.executionReady=false; metadata.preview_only=true;
metadata.scene_snapshot_sha256=hmi_workflow_file_hash(raw.sceneFile);
metadata.interface_branch_repaired=true;
metadata.segment_break_indices=fresh.metadata.segment_break_indices;
save(file,'metadata','-append');
report.preview_audit=audit_hmi_preview_display(file,outputDir);
report.interface_pass=report.after.approach_weld_transition.pass && report.preview_audit.collision_check_pass;
report.dependency_validation_after=hmi_planning_dependency_changes(scene.planningDependencySnapshot,...
    hmi_planning_dependency_snapshot(root));
report.interface_pass=report.interface_pass && report.dependency_validation_after.pass;
report.previewReady=report.interface_pass; report.success=report.interface_pass;
report.execution_point_count=size(fresh.qPath,1); report.duration=fresh.timeFromStart(end);
report.collision_check_pass=report.preview_audit.collision_check_pass;
report.algorithm='CBi-RRT* + ACCP'; report.task='Full_Welding_Task';
report.approach_actual_rotation_deg=sum(abs(diff(parts{1}.qPath(:,6))))*180/pi;
report.exit_actual_rotation_deg=sum(abs(diff(parts{3}.qPath(:,6))))*180/pi;
report.output_preview=file;
writeJson(fullfile(outputDir,'interface_diagnosis.json'),report);
writeJson(fullfile(outputDir,'report.json'),report);
end
function r=boundary(scene,a,b)
A=getTransform(scene.robot,a,scene.tcpBodyName); B=getTransform(scene.robot,b,scene.tcpBodyName);
r=struct('approach_end_deg',rad2deg(a),'weld_start_deg',rad2deg(b),'delta_deg',rad2deg(b-a),...
    'integer_turn_difference',(b-a)/(2*pi),'tcp_position_delta_mm',1000*norm(A(1:3,4)-B(1:3,4)),...
    'tcp_orientation_delta_deg',rad2deg(acos(max(-1,min(1,(trace(A(1:3,1:3)'*B(1:3,1:3))-1)/2)))));
end
function records=constantCandidates(A,W,start,lim)
records={};
for turns=-1:1
    C=A; C(:,6)=C(:,6)+turns*2*pi; D=W; D(:,6)=D(:,6)+turns*2*pi;
    records{end+1}=struct('J6_turns',turns,'approach_all_limits_pass',all(C>=lim(:,1).' & C<=lim(:,2).','all'),...
        'approach_preserves_RealHome',max(abs(C(1,:)-start))<1e-9,...
        'approach_end_deg',rad2deg(C(end,:)),...
        'weld_all_limits_pass',all(D>=lim(:,1).' & D<=lim(:,2).','all'),...
        'weld_J6_minmax_deg',rad2deg([min(D(:,6)),max(D(:,6))])); %#ok<AGROW>
end
end
function s=segment(scene,Q,label)
[t,~]=parameterize_joint_path(Q,'MaxVelocityDegPerSec',5,'MaxAccelerationDegPerSec2',15);
P=cell(1,size(Q,1)); for k=1:numel(P), P{k}=getTransform(scene.robot,Q(k,:),scene.tcpBodyName); end
s=struct('label',label,'qPath',Q,'tcpPosePath',{P},'timeFromStart',t,'velocities',[zeros(1,6);diff(Q)./diff(t)]);
end
function writeJson(file,data)
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
