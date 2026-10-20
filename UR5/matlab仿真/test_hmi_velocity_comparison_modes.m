function result=test_hmi_velocity_comparison_modes(outputFile)
% Synthetic hold-path unit test, NOT current-scene or physical certification.
% Exercises both C-PASS Hybrid branches without loading/mutating a UR5 model.
robot=rigidBodyTree('DataFormat','row'); parent=robot.BaseName;
for k=1:6
    body=rigidBody(sprintf('test_link_%d',k));
    joint=rigidBodyJoint(sprintf('test_joint_%d',k),'revolute');
    joint.PositionLimits=[-pi pi]; T=eye(4); T(3,4)=0.1;
    setFixedTransform(joint,T); body.Joint=joint;
    addCollision(body,collisionSphere(0.01)); addBody(robot,body,parent);
    parent=body.Name;
end
tip=rigidBody('TestTCP'); T=eye(4); T(3,4)=0.1;
setFixedTransform(tip.Joint,T); addBody(robot,tip,parent);
scene=struct('robot',robot,'environment',{{}},'tcpBodyName','TestTCP',...
    'jointLimits',repmat([-pi pi],6,1),...
    'selfCollisionSkipPairs','parent',...
    'calibrationBinding',struct('scope','SYNTHETIC_UNIT_TEST_ONLY'));
Q=zeros(47,6); P=repmat({getTransform(robot,Q(1,:),'TestTCP')},1,47); t=(0:46).';
comparison=generate_accp_adaptive_velocity_profile(Q,P,t,scene,struct());
daily=generate_accp_adaptive_velocity_profile(Q,P,t,scene,struct('compareStrategies',false));
assert(comparison.comparison_enabled && isequal(comparison.audited_strategy_indices,1:4));
assert(all(cellfun(@(a)isfield(a,'collision_pass') && a.complete,comparison.audits)));
assert(isequal(comparison.velocities,daily.velocities) && isequal(comparison.audits{4},daily.audits{4}));
assert(~daily.comparison_enabled && isequal(daily.audited_strategy_indices,4));
for k=1:3
    assert(strcmp(daily.audits{k}.status,'NOT_RUN_COMPARISON_ONLY') && ~daily.audits{k}.complete);
end
fullHybrid=optimize_accp_hybrid_velocity(comparison,scene,struct());
dailyHybrid=optimize_accp_hybrid_velocity(daily,scene,struct('compareReferences',false));
assert(fullHybrid.hybridCandidateFound && dailyHybrid.hybridCandidateFound);
assert(isequal(fullHybrid.velocities,dailyHybrid.velocities));
assert(isequal(fullHybrid.audits{3},dailyHybrid.audits{3}) &&...
    isequal(fullHybrid.audits{4},dailyHybrid.audits{4}));
for k=1:2
    assert(strcmp(dailyHybrid.audits{k}.status,'NOT_RUN_COMPARISON_ONLY'));
end
assert(~dailyHybrid.executionReady && ~fullHybrid.executionReady);
result=struct('pass',true,'scope','SYNTHETIC_HOLD_PATH_UNIT_TEST_NOT_UR5_CERTIFICATION',...
    'default_four_comparisons_preserved',true,'daily_only_C_audited',true,...
    'hybrid_C_and_D_audits_preserved',true,'same_generated_velocities',true,...
    'same_Hybrid_D_velocities_and_audit',true,'executionReady',false,'robot_motion','NO');
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(result,'PrettyPrint',true),'char'); disp(result);
end
