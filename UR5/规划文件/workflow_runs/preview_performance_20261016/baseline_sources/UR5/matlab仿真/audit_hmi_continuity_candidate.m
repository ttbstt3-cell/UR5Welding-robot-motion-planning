function result=audit_hmi_continuity_candidate(mode)
% Offline full-geometry audit. Deliberately no ready-file export mechanism.
here=fileparts(mfilename('fullpath')); root=fileparts(here); addpath(here);
addpath(genpath(fullfile(fileparts(root),'RRT')));
out=fullfile(root,'规划文件','workflow_runs','continuity_optimization_20261015');
if strcmp(mode,'weld')
    d=load(fullfile(out,'weld_candidate.mat')); t=d.trialT;
elseif strcmp(mode,'full_c2')
    d=load(fullfile(out,'full_candidate_c2.mat')); t=d.t;
else
    d=load(fullfile(out,'full_candidate_c1.mat')); t=d.t;
end
Q=d.Q; P=d.P; V=d.V; scene=d.scene;
timer=tic;
before=hmi_planning_dependency_snapshot(root);
binding=hmi_planning_dependency_changes(scene.planningDependencySnapshot,before);
assert(binding.pass,'Candidate scene source binding is STALE');
options=struct('samplesPerSegment',max(101,ceil(5000/(size(Q,1)-1))+1),...
    'maxVelocityRadPerSec',deg2rad(5),'maxAccelerationRadPerSec2',0.261799);
fprintf('DENSE_AUDIT mode=%s nodes=%d samples=%d\n',mode,size(Q,1),1+(size(Q,1)-1)*(options.samplesPerSegment-1));
result=validate_accp_controller_interpolation(Q,P,t,V,scene,options);
result.analytic_derivatives=accp_cubic_derivative_metrics(Q,t,V);
lim=scene.jointLimits; lim0=deriveUR5JointLimits(scene.robot);
lim=[max(lim(:,1),lim0(:,1)),min(lim(:,2),lim0(:,2))];
result.analytic_joint_limits_pass=all(result.analytic_derivatives.position_min>=lim(:,1).'-1e-10) &&...
    all(result.analytic_derivatives.position_max<=lim(:,2).'+1e-10);
result.complete=result.complete && result.analytic_joint_limits_pass &&...
    result.analytic_derivatives.max_velocity<=deg2rad(5) && result.analytic_derivatives.max_acceleration<=0.261799;
result.elapsed_sec=toc(timer); result.executionReady=false;
result.source_version_audit=binding;
result.sources_unchanged_during_audit=hmi_planning_dependency_changes(before,hmi_planning_dependency_snapshot(root));
result.complete=result.complete && result.sources_unchanged_during_audit.pass;
result.not_ready_reasons={'OFFLINE_ONLY','Physical safety margin and 3mm wire approximation not certified','Controller inactive; servo response not physically validated'};
if ~strcmp(mode,'weld')
    result.boundary_acceleration_jumps_rad_s2=result.analytic_derivatives.node_acceleration_jump(d.joins-1,:);
    result.boundary_velocity_jumps_rad_s=zeros(2,6);
    result.strict_c2_interfaces_pass=all(abs(result.boundary_acceleration_jumps_rad_s2)<1e-6,'all');
    result.original_weld_nodes_preserved=isequal(Q(d.maps{2},:),d.original.Q(d.original.maps{2},:));
    pp=P(d.maps{2}); orig=d.original.P(d.original.maps{2}); result.original_weld_targets_preserved=isequal(pp(:),orig(:));
    result.total_time_sec=t(end); result.per_segment_time_sec=[t(d.joins(1)),diff(t(d.joins)),t(end)-t(d.joins(2))];
end
save(fullfile(out,[mode '_dense_audit.mat']),'result','-v7.3');
fid=fopen(fullfile(out,[mode '_dense_audit.json']),'w'); c=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,PrettyPrint=true),'char');
fprintf('DENSE_RESULT mode=%s pass=%d sync=%.9f contour=%.9f ori=%.9f collisions=%d time=%.3f\n',...
    mode,result.complete,result.max_tcp_time_sync_error_mm,result.max_tcp_contour_error_mm,...
    result.max_tcp_orientation_error_deg,~result.collision_pass,result.elapsed_sec);
end
