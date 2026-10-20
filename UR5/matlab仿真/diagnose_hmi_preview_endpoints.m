function report=diagnose_hmi_preview_endpoints(runDir,outDir)
% Offline endpoint diagnosis. Reads the failed run snapshot without changing it.
root=fileparts(fileparts(mfilename('fullpath')));
project=fileparts(root);
addpath(genpath(fullfile(project,'RRT')));
if ~isfolder(outDir), mkdir(outDir); end
request=jsondecode(fileread(fullfile(runDir,'request.json')));
data=load(fullfile(runDir,'raw_scene.mat'),'scene'); scene=data.scene;
stored=load(request.seamCandidateFile,'r'); goal=stored.r.qPath(1,:);
report=struct('robot_motion','NO','executionReady',false,...
    'source_run',runDir,'scene_sha256',hmi_workflow_file_hash(fullfile(runDir,'raw_scene.mat')),...
    'calibration_binding',scene.calibrationBinding,...
    'seam_candidate_file',request.seamCandidateFile,...
    'seam_candidate_sha256',hmi_workflow_file_hash(request.seamCandidateFile));
states={deg2rad([180 -90 0 -90 0 0]),zeros(1,6),reshape(request.qStart,1,6),goal};
labels={'requested_q0','requested_zero','actual_request_start','cached_qPreWeld'};
report.states=cell(1,numel(states));
for i=1:numel(states)
    report.states{i}=auditState(scene,states{i},labels{i});
    fprintf('%s q_deg=%s self=%d environment=%d forearm_wrist2=%d\n',...
        labels{i},mat2str(rad2deg(states{i}),5),report.states{i}.self_collision,...
        report.states{i}.environment_collision,report.states{i}.forearm_wrist2_collision);
    disp(report.states{i}.collision_pairs);
end
report.qOffset=struct('offset_deg',rad2deg(scene.qOffset),...
    'frozen_adapter_equation','qMat=qCop+qOffset (Coppelia joint convention only)',...
    'frozen_qCop0_deg',rad2deg(scene.qCop0),...
    'real_scene_qMat0_deg',rad2deg(scene.qMat0),...
    'actual_request_qStart_deg',rad2deg(request.qStart),...
    'real_qMat0_matches_request_home',max(abs(scene.qMat0(:)-request.qHome(:)))<1e-12,...
    'real_input_offset_applied',false,...
    'conclusion','REAL builder replaces qMat0 with saved RealHome. HMI sends ROS joint values directly; preview planner and FK do not add qOffset.',...
    'display_conversion','T_base_pose = inv(T_URDFroot_base)*T_URDFroot_pose; frame rotation does not change joint values',...
    'T_URDFroot_base',getTransform(scene.robot,zeros(1,6),'base'));
cfg=struct('scene',scene,'start',request.qStart,'goal',goal,'seed',request.seed,...
    'maxIterations',1000000,'maxPlanningTimeSec',request.maxPlanningTimeSec,...
    'maxJointStepRad',deg2rad(12),'jointEdgeResolutionRad',deg2rad(2),...
    'goalBias',0.2,'anytime',true,'continueAfterFirstSolution',true,...
    'checkpointTimesSec',[5 10 15 20 30]);
copies={fullfile(project,'RRT'),fullfile(root,'机械臂控制算法','源码副本','RRT')};
report.failure_return_regression=cell(1,6);
testStarts={request.qStart,request.qStart,states{1}};
testGoals={goal,states{1},goal};
reasons={'invalid_start_and_goal_collision','invalid_start_collision','invalid_goal_collision'};
for i=1:2
    addpath(genpath(copies{i})); clear FastConnectAnytimeRRTStarJoint;
    for j=1:3
    cfg.start=testStarts{j}; cfg.goal=testGoals{j};
    p=FastConnectAnytimeRRTStarJoint(cfg);
    assert(isstruct(p) && ~p.success && isempty(p.path));
    assert(strcmp(p.terminationReason,reasons{j}));
    assert(p.stage1Iterations==0 && p.stage2Iterations==0 && p.treeANodes==0 && p.treeBNodes==0);
    assert(p.startStateValid==(j==3) && p.goalStateValid==(j==2));
    report.failure_return_regression{(i-1)*3+j}=struct('source',which('FastConnectAnytimeRRTStarJoint'),...
        'pass',true,'success',p.success,'reason',p.terminationReason,...
        'start_check',p.startStateDiagnostic,'goal_check',p.goalStateDiagnostic,...
        'iterations',p.stage1Iterations+p.stage2Iterations,'tree_nodes',p.nodeCount);
    save(fullfile(outDir,sprintf('planner_failure_return_%d_%d.mat',i,j)),'p','-v7');
    end
end
fid=fopen(fullfile(outDir,'endpoint_diagnostic.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char'); clear c;
save(fullfile(outDir,'endpoint_diagnostic.mat'),'report','-v7');
fprintf('Endpoint diagnostic complete: %s\n',outDir);
end

function out=auditState(scene,q,label)
robot=scene.robot; names=robot.BodyNames; envNames=cellstr(string({scene.workpieceProxyInfo.name}));
[ec,de]=checkCollision(robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
[sc,ds]=checkCollision(robot,q,'IgnoreSelfCollision','off',...
    'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
Tb=getTransform(robot,q,'base'); Ttcp=getTransform(robot,q,scene.tcpBodyName);
out=struct('label',label,'q_rad',q,'q_deg',rad2deg(q),...
    'within_joint_limits',all(q(:)>=scene.jointLimits(:,1)&q(:)<=scene.jointLimits(:,2)),...
    'self_collision',any(sc(:)),'environment_collision',any(ec(:)),...
    'TCP_pose_URDFroot',Ttcp,'TCP_pose_base',Tb\Ttcp,...
    'flange_pose_base',Tb\getTransform(robot,q,'flange'),...
    'collision_pairs',{{}},'collision_details',{{}},...
    'forearm_wrist2_collision',false,'minimum_positive_separation_mm',[]);
allPairs=cell(0,2);
for a=1:numel(names)-1
    for b=a+1:numel(names), allPairs(end+1,:)={names{a},names{b}}; end %#ok<AGROW>
end
% Retain exactly one pair per diagnostic query; the original scene and the
% production skip table remain untouched. This avoids inferring collision
% from NaN entries for skipped or empty bodies in the native distance matrix.
for a=1:numel(names)-1
    if isempty(getBody(robot,names{a}).Collisions), continue; end
    for b=a+1:numel(names)
        if isempty(getBody(robot,names{b}).Collisions), continue; end
        pair={names{a},names{b}};
        if isSkipped(pair,scene.selfCollisionSkipPairs), continue; end
        keep=strcmp(allPairs(:,1),pair{1})&strcmp(allPairs(:,2),pair{2});
        [hit,dist]=checkCollision(robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',allPairs(~keep,:),'Exhaustive','on');
        if strcmp(pair{1},'forearm_link')&&strcmp(pair{2},'wrist_2_link')
            out.forearm_wrist2_collision=any(hit(:));
            d=dist(a,b); if isfinite(d), out.forearm_wrist2_separation_mm=1000*d;
            else, out.forearm_wrist2_separation_mm=[]; end
        end
        if any(hit(:))
            out.collision_pairs{end+1}=[pair{1} ' <-> ' pair{2}];
            out.collision_details{end+1}=struct('type','self','a',pair{1},'b',pair{2},...
                'penetration_depth_mm',[],'distance_status','Collision; native signed penetration unavailable');
        end
    end
end
for a=1:numel(names)
    if isempty(getBody(robot,names{a}).Collisions), continue; end
    for b=1:numel(scene.environment)
        if isnan(de(a,b))
            out.collision_pairs{end+1}=[names{a} ' <-> ' envNames{b}];
            out.collision_details{end+1}=struct('type','environment','a',names{a},'b',envNames{b},...
                'penetration_depth_mm',[],'distance_status','Collision; native signed penetration unavailable');
        end
    end
end
positive=[de(isfinite(de)&de>0);ds(isfinite(ds)&ds>0)];
if ~isempty(positive), out.minimum_positive_separation_mm=min(positive)*1000; end
out.minimum_positive_separation_scope='Non-colliding pairs only; not clearance of colliding configuration';
end

function yes=isSkipped(pair,skip)
yes=any((strcmp(skip(:,1),pair{1})&strcmp(skip(:,2),pair{2}))|...
    (strcmp(skip(:,1),pair{2})&strcmp(skip(:,2),pair{1})));
end
