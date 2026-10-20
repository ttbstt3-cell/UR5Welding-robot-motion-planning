function report=test_hmi_exact_scene_queries(sceneFile,outputFile)
% Compare cached native matrices/FK including a colliding posture and misses.
s=load(sceneFile,'scene'); scene=s.scene;
binding=struct('scene_snapshot_sha256',hmi_workflow_file_hash(sceneFile),...
    'realhome_q',scene.qMat0,'collision_rule','UNCHANGED_NATIVE_EXHAUSTIVE');
if isfield(scene,'calibrationBinding'), binding.calibration_binding=scene.calibrationBinding; end
cache=HmiExactSceneQueries(scene,binding,true);
Q=[scene.qMat0; zeros(1,6); scene.qMat0+[1e-12 zeros(1,5)]];
maxFk=0;
for repetition=1:2
    for k=1:size(Q,1)
        q=Q(k,:);
        [a,b]=cache.environment(q); [aa,bb]=check_scene_environment_collision(scene,q);
        assert(isequaln(a,aa) && isequaln(b,bb),'Environment native matrix changed');
        [a,b]=cache.self(q); [aa,bb]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
        assert(isequaln(a,aa) && isequaln(b,bb),'Self-collision native matrix changed');
        A=cache.forward(q,scene.tcpBodyName); B=getTransform(scene.robot,q,scene.tcpBodyName);
        maxFk=max(maxFk,max(abs(A-B),[],'all')); assert(isequaln(A,B));
    end
end
stats=cache.summary();
assert(stats.environment_native_queries==3 && stats.environment_hits==3,...
    'Exact q differences were rounded or repeated samples were not cached');
% A newly bound request has no native results, even at the same q. Changing
% RealHome/tool/environment/rule versions cannot access an older instance.
changed=binding; changed.realhome_q=scene.qMat0+[1e-6 zeros(1,5)];
changed.calibration_binding=struct('tcp_version','different','gun_version','different',...
    'workpiece_version','different','environment_version','different','collision_rules','different');
fresh=HmiExactSceneQueries(scene,changed,true); fresh.environment(Q(1,:)); fs=fresh.summary();
assert(fs.environment_hits==0 && fs.environment_native_queries==1);
report=struct('pass',true,'max_fk_matrix_difference',maxFk,'native_matrices_equal',true,...
    'nearby_values_not_rounded',true,'new_request_cold',true,'statistics',stats,...
    'executionReady',false,'robot_motion','NO');
fid=fopen(outputFile,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
end
