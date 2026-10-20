function result=test_hmi_offline_speed_scaling(sourceFile,outputFile)
% Small offline contracts: no collision/geometry shortcuts or robot calls.
source=load(sourceFile); Q=source.qPath; V=source.velocities; t=source.timeFromStart(:);
base=struct('qPath',Q,'tcpPosePath',{source.tcpPosePath},'timeFromStart',t,...
    'velocities',V,'pathTimeReference',source.metadata.path_time_reference);
checks={};
for percent=[1 25 50 75 100]
    f=percent/100; [c,s]=hmi_scale_continuous_candidate(base,percent);
    assert(isequal(c.qPath,Q) && isequal(c.tcpPosePath,source.tcpPosePath));
    assert(isequal(c.timeFromStart,t/f) && isequal(c.velocities,V*f));
    assert(isequal(c.pathTimeReference.s,base.pathTimeReference.s) && ...
        isequal(c.pathTimeReference.poses,base.pathTimeReference.poses) && ...
        isequal(c.pathTimeReference.clockProgress,base.pathTimeReference.clockProgress));
    assert(abs(s.derivatives.max_velocity-s.baseline_derivatives.max_velocity*f)<1e-11);
    assert(abs(s.derivatives.max_acceleration-s.baseline_derivatives.max_acceleration*f^2)<1e-9);
    assert(abs(s.derivatives.max_segment_jerk-s.baseline_derivatives.max_segment_jerk*f^3)<1e-7);
    samples=linspace(0,t(end),151).';
    [P,R]=hmi_continuous_reference(base.pathTimeReference,samples);
    [Ps,Rs]=hmi_continuous_reference(c.pathTimeReference,samples/f);
    assert(max(abs(P-Ps),[],'all')<1e-12 && max(abs(R-Rs),[],'all')<1e-11);
    assert(~c.executionReady && ~s.executionReady);
    checks{end+1}=sprintf('%g percent geometry/clock/derivatives PASS',percent); %#ok<AGROW>
end
for invalid={0,-1,101,NaN,Inf,[50 50],'50'}
    rejected=false; try, hmi_scale_continuous_candidate(base,invalid{1}); catch, rejected=true; end
    assert(rejected,'Bad percent accepted');
end
checks{end+1}='Invalid/nonfinite/non-scalar/overspeed percentages rejected';
broken=base; broken.pathTimeReference.clockTimes(2)=broken.pathTimeReference.clockTimes(2)+1;
rejected=false; try, hmi_scale_continuous_candidate(broken,50); catch, rejected=true; end
assert(rejected); checks{end+1}='Mismatching independent Cartesian clock rejected';
assert(isequal(base.qPath,Q) && isequal(base.timeFromStart,t) && isequal(base.velocities,V));
result=struct('pass',true,'checks',{checks},'source_unchanged',true,'robot_motion','NO',...
    'scope','Unit contracts, not full collision acceptance or physical authorization');
fid=fopen(outputFile,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char');
fprintf('OFFLINE_SPEED_SCALING_CONTRACTS_PASS\n');
end
