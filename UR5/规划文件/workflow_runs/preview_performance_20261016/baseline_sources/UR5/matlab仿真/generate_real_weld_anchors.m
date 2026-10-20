function report=generate_real_weld_anchors(outputDir,previewOnly,selectionOptions)
% Cold ACCP run in the explicit REAL scene. Never publishes motion.
if nargin<2, previewOnly=false; end
if nargin<3, selectionOptions=struct(); end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
% MATLAB may run with matlab仿真 as cwd. Normalize caller paths against the
% repository parent so planner round files are written where requested.
if ~localIsAbsolute(outputDir)
    repo=fileparts(root);
    if startsWith(outputDir,['UR5' filesep]) || startsWith(outputDir,'UR5/')
        outputDir=fullfile(repo,outputDir);
    else
        outputDir=fullfile(root,outputDir);
    end
end
if ~isfolder(outputDir), mkdir(outputDir); end
report=struct('success',false,'status','PLAN_FAILED','robot_motion','NO','preview_only',previewOnly);
t=tic;
try
    scene=build_real_planning_scene(previewOnly);
    report.calibration_binding=scene.calibrationBinding;
    if isfield(scene,'calibrationVersions')
        report.calibration_versions=scene.calibrationVersions;
    end
    report.real_tcp_loaded=scene.realTcpLoaded; report.real_workpiece_loaded=scene.realWorkpieceLoaded;
    report.gun_component_count=scene.realGunComponentCount;
    report.wire_axis_status=scene.realWireAxisStatus;
    report.real_workpiece_origin_root_m=scene.T_MatBase_Workpiece(1:3,4).';
    report.saved_real_home=scene.qMat0;
    % Formal real-scene policy: keep the visual wire unchanged and shorten
    % only the MATLAB collision cylinder by 3 mm for ACCP collision checks.
    wireCollisionShortenMm = 3;
    report.wire_collision_shorten_mm = wireCollisionShortenMm;
    wireStatus = upper(string(scene.realWireAxisStatus));
    if ~previewOnly && ~any(wireStatus == ["MODEL_AXIS_TO_TCP_ACTIVE","MODEL_DEFINED_ACTIVE","VERIFIED","CONFIRMED"])
        error('REAL_ANCHORS:ToolAxis','Real wire/tip geometry is unavailable or invalid; expected MODEL_DEFINED_ACTIVE, VERIFIED, or CONFIRMED after manifest checks');
    end
    script=fullfile(here,'sync_ur5_real_scene.m');
    if previewOnly, script=fullfile(here,'sync_ur5_real_scene_diagnostic.m'); end
    opts=struct('sceneScript',script,'initialPoints',16,'maxRefinementRounds',4, ...
       'maxPoints',40,'maxPlanningTimeSec',60,'enablePostRecovery',true, ...
       'branchReferenceQ',scene.qMat0,'randomSeed',20260917, ...
       'outputFile',fullfile(outputDir,'real_seam.mat'), ...
       'roundOutputPrefix',fullfile(outputDir,'real_seam_round'), ...
       'publish',false,'animate',false);
    opts.wire_collision_shorten_mm = wireCollisionShortenMm;
    opts.candidate_scoring_mode=localResultField(selectionOptions,'candidate_scoring_mode',true);
    opts.candidate_score_weights=localResultField(selectionOptions,'candidate_score_weights',[1 1 0.1 0.1]);
    opts.candidate_diagnostics=true;
    result=replan_weld_actual_axis_adaptive_feasibility(opts);
    report.initial_points=localResultField(result,'initialPoints',16);
    report.final_points=localResultField(result,'finalPoints',NaN);
    report.refinement_rounds=localResultField(result,'refinementRounds',NaN);
    report.planning_time_sec=localResultField(result,'totalPlanningTimeSec',NaN);
    report.complete=logical(localResultField(result,'complete',false));
    if isfield(result,'refinementReports') && ~isempty(result.refinementReports)
        rr=result.refinementReports;
        report.final_points=rr(end).pointsAfter;
        report.refinement_rounds=rr;
        report.planning_time_sec=sum([rr.algorithmTimeSec]);
    elseif isfield(result,'roundReports') && ~isempty(result.roundReports)
        rr=result.roundReports;
        report.final_points=rr(end).pointsAfter;
        report.refinement_rounds=rr;
        report.planning_time_sec=sum([rr.algorithmTimeSec]);
    end
    if ~isfield(result.scene,'calibrationBinding') || ~isequal(result.scene.calibrationBinding,scene.calibrationBinding)
        error('REAL_ANCHORS:SceneMismatch','ACCP did not use the requested real scene');
    end
    report.seam_complete=logical(result.complete);
    report.point1_diagnostics_file=localResultField(result,'point1DiagnosticsFile','');
    report.point1_filter_counts=localResultField(result,'point1FilterCounts',struct());
    if isfield(result,'continuationStats')
        stats=result.continuationStats;
        report.ik_calls=localResultField(stats,'ikCalls',NaN);
        report.ik_pose_success=localResultField(stats,'ikPoseSuccess',NaN);
        if isfinite(report.ik_calls) && report.ik_calls>0
            report.ik_pose_success_rate=report.ik_pose_success/report.ik_calls;
        else
            report.ik_pose_success_rate=NaN;
        end
        report.edge_collision_checks=localResultField(stats,'edgeCollisionChecks',NaN);
    end
    report.collision_check_pass=logical(result.complete) && ...
        logical(localResultField(result,'nodeCollisionFree',false)) && ...
        logical(localResultField(result,'edgeCollisionFree',false));
    if ~result.complete, error('REAL_ANCHORS:Seam','ACCP did not complete the real seam'); end
    q=double(result.qPath); % ACCP's embedded precision output, not sparse nodes.
    pre=transition(scene,q(1,:),-1); post=transition(scene,q(end,:),-1);
    anchors=struct('qHome',scene.qMat0,'qPreWeld',pre(end,:),'qPostWeld',post(end,:), ...
      'qWeldStart',q(1,:),'qWeldEnd',q(end,:),'branchReferenceQ',q(1,:), ...
      'calibration_binding',scene.calibrationBinding,'source','ACCP REAL cold start', ...
      'preview_only',previewOnly,'wire_axis_status',scene.realWireAxisStatus);
    if isfield(scene,'calibrationVersions')
        anchors.calibration_versions=scene.calibrationVersions;
    end
    report.seam_points=size(q,1); report.qPreWeld=anchors.qPreWeld;
    report.anchor_file=fullfile(outputDir,'weld_anchor_points_real.json');
    writeJson(report.anchor_file,anchors);
    report.success=true; report.status='OFFLINE_DIAGNOSTIC_ONLY';
    % Only a valid model-defined/verified wire record is promoted to a real
    % welding anchor; malformed or missing geometry remains blocked.
    if ~previewOnly && any(wireStatus == ["MODEL_AXIS_TO_TCP_ACTIVE","MODEL_DEFINED_ACTIVE","VERIFIED","CONFIRMED"])
        copyfile(report.anchor_file,fullfile(root,'规划文件','weld_anchor_points_real.json'));
        report.status='VALIDATED_REAL_ANCHORS';
    end
catch err
    report.failure_reason=err.message; report.identifier=err.identifier;
    report.failure_stack=err.stack;
    fprintf('%s\n',getReport(err,'extended','hyperlinks','off'));
    fprintf('REAL_ANCHORS_FAILED %s\n',err.message);
end
report.elapsed_sec=toc(t); writeJson(fullfile(outputDir,'report.json'),report);
end
function path=transition(scene,q,direction)
% Short Cartesian withdrawal, using the existing IK and collision utilities.
ik=inverseKinematics('RigidBodyTree',scene.robot); path=q;
T=getTransform(scene.robot,q,scene.tcpBodyName); axis=T(1:3,1:3)*scene.wireAxisTcp(:);
for d=.005:.005:.025
    target=T; target(1:3,4)=T(1:3,4)+direction*d*axis;
    sol=ik(scene.tcpBodyName,target,[1 1 1 1 1 1],path(end,:));
    sol=sol+2*pi*round((path(end,:)-sol)/(2*pi));
    actual=getTransform(scene.robot,sol,scene.tcpBodyName);
    if norm(actual-target,'fro')>1e-3 || ~jointWithinLimits(sol,scene.jointLimits)
        error('REAL_ANCHORS:Transition','Approach/exit IK or joint limits failed');
    end
    [free,~]=collisionFreeJointState(scene.robot,sol,scene.environment,scene.selfCollisionSkipPairs);
    opts=struct('selfCollisionSkipPairs',{scene.selfCollisionSkipPairs});
    [edge,~,~,~]=collisionFreeJointEdge(scene.robot,path(end,:),sol,scene.environment,deg2rad(.5),opts);
    if ~free || ~edge, error('REAL_ANCHORS:Transition','Approach/exit node or dense edge collision'); end
    path(end+1,:)=sol; %#ok<AGROW>
end
end
function writeJson(file,data)
fid=fopen(file,'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(data),'char');
end
function tf=localIsAbsolute(p)
tf=startsWith(p,filesep) || (ispc && numel(p)>=2 && p(2)==':');
end
function value=localResultField(s,name,default)
if isfield(s,name), value=s.(name); else, value=default; end
end
