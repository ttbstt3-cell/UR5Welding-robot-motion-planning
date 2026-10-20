function report=test_accp_cartesian_refinement(inputFile,outputDir)
% Saved REAL 16-node chain, offline only. No anchor promotion or ROS publish.
S=load(inputFile,'result');core=S.result;
original=core.qPath;configurationComplete=core.complete;
if isfield(core,'qPathInitial'),original=core.qPathInitial;end
if isfield(core,'configurationComplete'),configurationComplete=core.configurationComplete;end
assert(configurationComplete && size(original,1)==16);
assert(core.options.sideToleranceDeg==5 && ~core.options.disableAxisConstraint);
assert(core.scene.wireCollisionEnabled && core.scene.realTcpLoaded && core.scene.realWorkpieceLoaded);
fresh=build_real_planning_scene(false);
assert(isequal(core.scene.calibrationBinding,fresh.calibrationBinding),'Saved scene is stale');
if ~isfolder(outputDir),mkdir(outputDir);end
% Unit tests: true SLERP, not NLERP; endpoints; shortest quaternion arc.
A=eye(4);B=eye(4);B(1:3,1:3)=axang2rotm([0 0 1 deg2rad(120)]);
B(1:3,4)=[1;2;3];T=interpolate_tcp_pose_slerp(A,B,0.25);
assert(norm(T(1:3,1:3)-axang2rotm([0 0 1 deg2rad(30)]),'fro')<1e-12);
assert(norm(T(1:3,4)-[.25;.5;.75])<1e-12);
assert(isequal(interpolate_tcp_pose_slerp(A,B,0),A));
assert(isequal(interpolate_tcp_pose_slerp(A,B,1),B));
B(1:3,1:3)=axang2rotm([0 0 1 deg2rad(270)]);
T=interpolate_tcp_pose_slerp(A,B,.5);
assert(norm(T(1:3,1:3)-axang2rotm([0 0 1 deg2rad(-45)]),'fro')<1e-12);
t=tic;precision=refine_accp_cartesian_path(core,core.options);wall=toc(t);
original_qPath=precision.qPathOriginal;refined_qPath=precision.qPathExecution;
qPath=original_qPath;tcpTarget=precision.tcpTargetOriginal; %#ok<NASGU>
save(fullfile(outputDir,'original_qPath.mat'),'qPath','tcpTarget');
qPath=refined_qPath;tcpTarget=precision.tcpTargetExecution; %#ok<NASGU>
save(fullfile(outputDir,'refined_qPath.mat'),'qPath','tcpTarget','precision','-v7.3');
% Independent 101-sample FK audit against original global Cartesian segments.
nodeIndices=precision.originalNodeIndices;maxError=0;maxOri=0;maxFixed=0;
for i=1:size(original_qPath,1)-1
    for e=nodeIndices(i):nodeIndices(i+1)-1
        for s=linspace(0,1,101)
            q=refined_qPath(e,:)+s*(refined_qPath(e+1,:)-refined_qPath(e,:));
            globalS=((e-nodeIndices(i))+s)/(nodeIndices(i+1)-nodeIndices(i));
            target=interpolate_tcp_pose_slerp(precision.tcpTargetOriginal{i},precision.tcpTargetOriginal{i+1},globalS);
            actual=getTransform(core.scene.robot,q,core.scene.tcpBodyName);
            maxError=max(maxError,norm(actual(1:3,4)-target(1:3,4))*1000);
            c=(trace(target(1:3,1:3).'*actual(1:3,1:3))-1)/2;
            maxOri=max(maxOri,rad2deg(acos(max(-1,min(1,c)))));
            fixed=getTransform(core.scene.robot,q,core.scene.tcpBodyName,'flange');
            fixed0=getTransform(core.scene.robot,original_qPath(1,:),core.scene.tcpBodyName,'flange');
            maxFixed=max(maxFixed,norm(fixed-fixed0,'fro'));
        end
    end
end
report=struct('input_file',inputFile,'interpolation_before','joint-space linear', ...
    'interpolation_after',char(precision.interpolation),'complete',precision.complete, ...
    'original_point_count',size(original_qPath,1),'refined_point_count',size(refined_qPath,1), ...
    'inserted_point_count',size(refined_qPath,1)-size(original_qPath,1), ...
    'original_max_tcp_error_mm',max(precision.initialTcpMetrics.segmentPositionErrorMm), ...
    'refined_max_tcp_error_mm',max(precision.finalTcpMetrics.segmentPositionErrorMm), ...
    'independent_101_samples_max_tcp_error_mm',maxError, ...
    'independent_max_orientation_error_deg',maxOri, ...
    'max_node_error_mm',max(precision.finalTcpMetrics.nodePositionErrorMm), ...
    'max_raw_joint_step_deg',max(abs(rad2deg(diff(refined_qPath))),[],'all'), ...
    'max_flange_tcp_transform_change',maxFixed, ...
    'original_nodes_exactly_preserved',isequal(original_qPath,refined_qPath(nodeIndices,:)), ...
    'node_collision_pass',precision.finalTcpMetrics.nodeCollisionFree, ...
    'dense_edge_collision_pass',precision.finalTcpMetrics.edgeCollisionFree, ...
    'hard_constraints_pass',all(precision.finalTcpMetrics.nodeHardConstraintsPass), ...
    'calibration_binding',core.scene.calibrationBinding,'wall_time_sec',wall, ...
    'ik_calls',precision.ikCalls,'robot_motion','NO','time_parameterized',false, ...
    'segments',precision.segmentReport);
fid=fopen(fullfile(outputDir,'report.json'),'w');assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));disp(report);
assert(report.complete && maxError<0.5,'Cartesian precision test failed');
assert(maxFixed<1e-12 && report.original_nodes_exactly_preserved);
end
