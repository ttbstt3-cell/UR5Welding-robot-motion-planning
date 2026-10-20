function report=prepare_hmi_continuous_request(request)
% Second-stage execution preparation. No ROS connection or robot command.
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=char(request.outputDir); if ~isfolder(out), mkdir(out); end
report=struct('executionReady',false,'success',false,'robot_motion','NO',...
    'task','Full_Welding_Task','algorithm','CBi-RRT* + ACCP-D','preview_preserved',true);
timer=tic;
try
    raw=load(request.rawFile); segments=raw.segments;
    if isfield(raw.metadata,'algorithm'), report.algorithm=raw.metadata.algorithm; end
    assert(raw.metadata.preview_only,'Second stage requires a preview artifact');
    assert(numel(segments)==3 && strcmp(segments{1}.label,'approach') &&...
        strcmp(segments{2}.label,'weld') && strcmp(segments{3}.label,'exit'),...
        'Full task requires approach/weld/exit segments');
    assert(strcmp(hmi_workflow_file_hash(raw.sceneFile),raw.metadata.scene_snapshot_sha256),...
        'Preview scene snapshot was modified');
    ss=load(raw.sceneFile,'scene'); scene=ss.scene;
    % Representation errors are not Cartesian gaps and must never be sent
    % to local IK as a request to rotate through an equivalent full turn.
    report.stage='INTERFACE_GEOMETRY';
    for k=1:numel(segments)
        q=segments{k}.qPath; lim=scene.jointLimits;
        if any(q<lim(:,1).' | q>lim(:,2).','all')
            error('PREPARE:InterfaceLimits','%s numeric path violates joint limits; regenerate its pinned branch',segments{k}.label);
        end
    end
    fprintf('HMI_WELD_STAGE stage=BOUNDARIES\n');
    report.boundaries_before={boundary(segments{1},segments{2}),boundary(segments{2},segments{3})};
    pieces={segments{1}}; transitions={};
    for k=1:2
        a=segments{k}; b=segments{k+1};
        dq=b.qPath(1,:)-a.qPath(end,:);
        if any(abs(dq)>pi) && max(abs(dq/(2*pi)-round(dq/(2*pi))))<1e-9
            error('PREPARE:PeriodicBranch','%s -> %s has an integer-turn branch mismatch; replan the free-space segment with the exact ACCP endpoint',a.label,b.label);
        end
        if max(abs(a.qPath(end,:)-b.qPath(1,:)))>1e-10
            fprintf('HMI_WELD_STAGE stage=LOCAL_TRANSITION\n');
            [bridge,br]=hmi_cartesian_boundary_transition(a,b,scene,request);
            transitions{end+1}=br; %#ok<AGROW>
            if ~br.pass
                report.local_transitions=transitions; error('PREPARE:Boundary','Boundary Cartesian transition failed: %s',br.failure_reason);
            end
            pieces{end+1}=bridge; %#ok<AGROW>
        end
        pieces{end+1}=b; %#ok<AGROW>
    end
    report.local_transitions=transitions;
    fprintf('HMI_WELD_STAGE stage=VELOCITY_OPTIMIZATION\n');
    report.stage='VELOCITY_OPTIMIZATION';
    weld=segments{2};
    % Reuse a verified D profile only for identical immutable q/pose/time data.
    saved=fullfile(root,'规划文件','real_anchor_runs','hybrid_velocity_20261014_final','hybrid_velocity_diagnostic.mat');
    if isfield(weld,'source_file') && isfile(weld.source_file), saved=weld.source_file; end
    reuse=false;
    if isfile(saved)
        d=load(saved,'r'); d=d.r;
        reuse=isequal(weld.qPath,d.qPath) && isequal(weld.tcpPosePath(:),d.tcpPosePath(:)) &&...
            isequal(weld.timeFromStart(:),d.timeFromStart(:));
    end
    if reuse
        weldV=d.velocities{4}; report.hybrid_velocity_source='Existing D, identical q/TCP/time; full audit repeated';
    else
        if size(weld.qPath,1)~=47
            error('PREPARE:HybridContract','Existing Hybrid optimizer requires 47 weld nodes; preview remains available (%d nodes)',size(weld.qPath,1));
        end
        base=generate_accp_adaptive_velocity_profile(weld.qPath,weld.tcpPosePath,...
            weld.timeFromStart,scene,struct());
        base.calibrationBinding=raw.metadata.calibration_binding;
        % Preserve the precise timing input failure instead of allowing the
        % Hybrid module's generic "C is not valid" assertion to hide it.
        report.weld_velocity_input_audits=cellfun(@auditSummary,base.audits,'UniformOutput',false);
        save(fullfile(out,'velocity_profile_input_diagnostic.mat'),'base','-v7.3');
        c=base.audits{4};
        if ~c.complete
            % Only node velocities are changed here. The old Hybrid module
            % still requires a feasible C seed and is deliberately untouched.
            [weldV,weldOptimization]=optimize_hmi_cubic_node_velocities(weld.qPath,...
                weld.tcpPosePath,weld.timeFromStart,base.velocities{4},scene,struct());
            report.weld_controller_velocity_optimization=weldOptimization;
            if ~weldOptimization.pass
                error('PREPARE:TimingInput','Fixed-node controller velocity optimization failed: %s',weldOptimization.failure_reason);
            end
            report.hybrid_velocity_source='Controller-aware fixed-node velocity recovery; dense full-task audit mandatory';
        else
            hybrid=optimize_accp_hybrid_velocity(base,scene,struct());
            if ~hybrid.hybridCandidateFound, error('PREPARE:Hybrid','Hybrid velocity generation failed'); end
            weldV=hybrid.velocities{4}; report.hybrid_velocity_source='Existing Hybrid optimizer';
        end
    end
    [Q,P,t,V,labels,maps,joins]=mergePieces(pieces);
    iw=find(cellfun(@(s)strcmp(s.label,'weld'),pieces),1); weldIndices=maps{iw};
    V(weldIndices,:)=weldV;
    [candidate,vr]=optimize_hmi_controller_continuity(Q,P,t,V,scene,labels,maps,joins);
    Q=candidate.qPath; P=candidate.tcpPosePath; t=candidate.timeFromStart; V=candidate.velocities;
    labels=candidate.segmentLabel; maps=candidate.maps; joins=candidate.joins;
    report.local_velocity_trials=vr.timing_trials;
    report.velocity_optimization=vr;
    if ~vr.pass, error('PREPARE:Velocity','%s',vr.failure_reason); end
    report.boundaries_after=struct('velocity_jump',0,'acceleration_jumps',vr.derivatives.node_acceleration_jump(joins-1,:));
    report.original_path_nodes_preserved=true;
    for k=1:numel(pieces)
        if max(abs(Q(maps{k},:)-pieces{k}.qPath),[],'all')>1e-10
            error('PREPARE:NodeMutation','An original path node was changed');
        end
    end
    assert(max(abs(Q(1,:)-raw.metadata.requested_start_q))<1e-6 &&...
        max(abs(Q(end,:)-raw.metadata.real_home_q))<1e-6,'Task endpoints differ from requested start / saved RealHome');
    fprintf('HMI_WELD_STAGE stage=CONTROLLER_VALIDATION\n');
    report.stage='CONTROLLER_VALIDATION';
    count=max(5000,request.minControllerSamples); m=ceil((count-1)/(size(Q,1)-1))+1;
    audit=validate_accp_controller_interpolation(Q,P,t,V,scene,struct(...
        'samplesPerSegment',m,'maxContourErrorMm',0.5,'maxTimeSyncErrorMm',0.5,...
        'maxOrientationErrorDeg',0.5,'maxVelocityRadPerSec',deg2rad(5),'maxAccelerationRadPerSec2',0.261799));
    lim=scene.jointLimits; dm=vr.derivatives;
    exact=dm.max_velocity<=deg2rad(5) && dm.max_acceleration<=0.261799 &&...
        all(dm.position_min>=lim(:,1).') && all(dm.position_max<=lim(:,2).');
    audit.analytic_extrema_pass=exact; audit.complete=audit.complete && exact;
    audit.max_joint_velocity_rad_per_sec=dm.max_velocity; audit.max_joint_acceleration_rad_per_sec2=dm.max_acceleration;
    report.controller_validation=audit; report.jerk_statistics=struct('max',dm.max_segment_jerk,...
        'rms',dm.rms_segment_jerk,'node_acceleration_jump_max',dm.max_acceleration_jump);
    if ~audit.complete
        if ~audit.collision_pass, reason='Collision failed';
        elseif ~exact, reason='Analytic velocity/acceleration/joint extrema failed';
        else, reason=['Controller interpolation failed: ' audit.failureReason]; end
        error('PREPARE:Controller','%s',reason);
    end
    fprintf('HMI_WELD_STAGE stage=VERSIONS\n');
    if ~strcmp(hmi_workflow_file_hash(request.rawFile),request.raw_sha256)
        error('PREPARE:Version','Version mismatch: raw preview file changed during preparation');
    end
    if ~strcmp(raw.metadata.calibration_binding.planning_scene,'REAL_CALIBRATION')
        error('PREPARE:Scene','Version mismatch: simulation preview cannot be certified for the real scene');
    end
    currentScene=build_real_planning_scene(false);
    current=hmi_workflow_binding(currentScene,request.current_binding,root);
    reasons=request.execution_gate_reasons;
    report.version_validation=validate_hmi_workflow_versions(raw.metadata.calibration_binding,current,segments,reasons);
    mismatches=report.version_validation.mismatches;
    if ~isempty(mismatches), error('PREPARE:Version','Version mismatch: %s',strjoin(mismatches,', ')); end
    if ~isempty(reasons), error('PREPARE:Reality','Real-scene verification failed: %s',strjoin(cellstr(string(reasons)),'; ')); end
    % This revision is an offline-only experiment. Numerical PASS is not
    % independent physical clearance/servo validation or permission to export.
    report.offline_controller_validation_pass=true;
    candidate.executionReady=false; candidate.diagnostic_only=true;
    save(fullfile(out,'controller_continuity_diagnostic.mat'),'candidate','report','-v7.3');
    if vr.requires_independent_physical_safety_validation
        error('PREPARE:OfflineOnly','Offline controller validation PASS; physical safety/servo validation pending. executionReady=false; no executable trajectory exported.');
    end
    % Only positions and velocities are controller command fields. Supplying
    % accelerations would select quintic interpolation and invalidate this audit.
    qPath=Q; tcpPosePath=P; timeFromStart=t; velocities=V;
    tcpPath=cell2mat(cellfun(@(T)T(1:3,4).',P,'UniformOutput',false).');
    segmentLabel=labels; joint_names=raw.joint_names;
    metadata=struct('source','MATLAB_HMI_EXECUTION_PREPARATION','trajectory_name','trajectory_execute_ready',...
        'preview_only',false,'workflow_stage','EXECUTION_PREPARED','executionReady',true,...
        'calibration_binding',current,'collision_verified',true,'collision_check_pass',true,...
        'controller_interpolation','positions_plus_velocities_cubic_hermite','tcp_frame','base_link',...
        'controller_sample_count',audit.sample_count,'physical_safety_margin_validated',false);
    report.executionReady=true; report.success=true; report.collision_check_pass=true;
    report.continuity_check_pass=true; report.execution_point_count=size(Q,1); report.duration=t(end);
    report.note='Offline preparation only. Existing live Final Check and manual Execute remain mandatory.';
    save(fullfile(out,'trajectory_execute_ready.mat'),'qPath','tcpPosePath','tcpPath','timeFromStart',...
        'velocities','segmentLabel','joint_names','metadata','-v7');
    record=struct('qPath',Q,'tcpPath',tcpPath,'timeFromStart',t,'velocities',V,...
        'tcpPosePath',{P},'segmentLabel',{labels},'joint_names',{joint_names},'metadata',metadata,'report',report);
    writeJson(fullfile(out,'trajectory_execute_ready.json'),record);
catch err
    report.executionReady=false; report.success=false; report.failure_reason=err.message;
    report.failure_identifier=err.identifier;
    report.failure_stack=err.stack;
    fprintf('PREPARATION_FAILED %s\n',err.message);
end
report.planning_time=toc(timer); writeJson(fullfile(out,'report.json'),report);
end
function result=auditSummary(audit)
result=struct();
for field=fieldnames(audit).'
    value=audit.(field{1});
    if ((isnumeric(value)||islogical(value)) && isscalar(value)) || ischar(value) || isstring(value)
        result.(field{1})=value;
    end
end
end
function r=boundary(a,b)
T=a.tcpPosePath{end}; U=b.tcpPosePath{1}; dq=b.qPath(1,:)-a.qPath(end,:);
ma=accp_cubic_derivative_metrics(a.qPath,a.timeFromStart,a.velocities);
mb=accp_cubic_derivative_metrics(b.qPath,b.timeFromStart,b.velocities);
r=struct('joint_delta_rad',dq,'tcp_position_delta_mm',1000*norm(U(1:3,4)-T(1:3,4)),...
    'tcp_orientation_delta_deg',rad2deg(acos(max(-1,min(1,(trace(T(1:3,1:3)'*U(1:3,1:3))-1)/2)))),...
    'velocity_jump',b.velocities(1,:)-a.velocities(end,:),...
    'acceleration_jump',mb.acceleration_start(1,:)-ma.acceleration_end(end,:));
end
function [Q,P,t,V,labels,maps,joins]=mergePieces(parts)
Q=[]; P={}; t=[]; V=[]; labels={}; maps=cell(size(parts)); joins=[];
for k=1:numel(parts)
    s=parts{k}; n=size(s.qPath,1); start=size(Q,1);
    if k==1, idx=1:n; offset=-s.timeFromStart(1); maps{k}=1:n;
    else
        assert(max(abs(Q(end,:)-s.qPath(1,:)))<1e-10,'Unrepaired position discontinuity');
        idx=2:n; offset=t(end)-s.timeFromStart(1); maps{k}=[start,start+(1:n-1)]; joins(end+1)=start; %#ok<AGROW>
    end
    Q=[Q;s.qPath(idx,:)]; P=[P,reshape(s.tcpPosePath(idx),1,[])]; t=[t;s.timeFromStart(idx)+offset]; %#ok<AGROW>
    ref=centralVelocity(s.qPath,s.timeFromStart); V=[V;ref(idx,:)]; labels=[labels,repmat({s.label},1,numel(idx))]; %#ok<AGROW>
end
assert(all(diff(t)>0),'Merged timestamps not strictly increasing');
end
function V=centralVelocity(Q,t)
h=diff(t); slope=diff(Q)./h; V=[slope(1,:);zeros(size(Q,1)-2,6);slope(end,:)];
for k=2:size(Q,1)-1, V(k,:)=(h(k)*slope(k-1,:)+h(k-1)*slope(k,:))/(h(k-1)+h(k)); end
end
function writeJson(file,data)
fid=fopen(file,'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
