function [segments,report]=hmi_prepare_deferred_weld_timing(segments,scene)
% Run execution-only ACCP timing when preparation consumes a geometry preview.
% The animation clock is not accepted as an execution timing profile.
clock=tic;
report=struct('requested',false,'algorithm_time_sec',0,'stage','Adaptive Time Scaling',...
    'executionReady',false,'robot_motion','NO');
for k=1:numel(segments)
    weld=segments{k};
    if ~strcmp(weld.label,'weld') || ~isfield(weld,'time_parameterization_deferred') || ...
            ~weld.time_parameterization_deferred, continue; end
    assert(isfield(weld,'geometry_validation') && weld.geometry_validation.complete && ...
        weld.geometry_validation.node_collision_free && weld.geometry_validation.edge_collision_free,...
        'PREPARE:Geometry','Deferred timing requires validated weld geometry');
    beforeQ=weld.qPath; beforePose=weld.tcpPosePath;
    opts=weld.accp_time_scaling_options;
    opts.collisionPass=true;
    opts.collisionSampleCount=weld.geometry_validation.collision_sample_count;
    timing=adaptive_time_scale_accp_output(weld.qPath,weld.tcpPosePath,scene,opts);
    weld.timeFromStart=timing.timeFromStart; weld.velocities=timing.velocities;
    weld.accelerations=timing.accelerations;
    weld.time_parameterization_deferred=false;
    weld.time_parameterization_complete=timing.complete;
    weld.timing_scope='PREPARATION_CANDIDATE_REQUIRES_FULL_CONTROLLER_VALIDATION';
    assert(isequal(beforeQ,weld.qPath) && isequal(beforePose,weld.tcpPosePath),...
        'PREPARE:NodeMutation','Deferred timing changed weld nodes');
    segments{k}=weld; report.requested=true; report.initial_timing=timing;
end
report.algorithm_time_sec=toc(clock);
end
