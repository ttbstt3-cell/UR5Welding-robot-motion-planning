function report=test_hmi_deferred_preview_timing(rawFile,oldAccpFile,outputFile)
% Verify the preparation boundary performs the original timing, exactly once.
r=load(rawFile); s=load(r.sceneFile,'scene'); before=r.segments;
assert(before{2}.time_parameterization_deferred);
[after,timing]=hmi_prepare_deferred_weld_timing(before,s.scene);
old=load(oldAccpFile,'result'); old=old.result;
assert(timing.requested && ~after{2}.time_parameterization_deferred);
assert(isequaln(after{2}.qPath,before{2}.qPath) && isequaln(after{2}.tcpPosePath,before{2}.tcpPosePath));
assert(isequaln(after{2}.timeFromStart,old.timeFromStart(:)));
assert(isequaln(after{2}.velocities,old.velocities));
[again,second]=hmi_prepare_deferred_weld_timing(after,s.scene);
assert(~second.requested && isequaln(again,after));
report=struct('pass',true,'timing_run_once',true,'q_and_tcp_unchanged',true,...
    'timestamps_identical_to_original',true,'velocities_identical_to_original',true,...
    'initial_timing_acceptance',after{2}.time_parameterization_complete,...
    'initial_timing_failure_preserved',timing.initial_timing.failureReason,...
    'controller_sample_count',timing.initial_timing.controllerInterpolationValidation.sample_count,...
    'elapsed_sec',timing.algorithm_time_sec,'executionReady',false,'robot_motion','NO');
fid=fopen(outputFile,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
