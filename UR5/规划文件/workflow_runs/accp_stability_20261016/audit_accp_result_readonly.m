function audit=audit_accp_result_readonly(file,out)
% Read saved result and native collision backends; no planner/model writes.
root='/home/ttbstt/jixiebi'; addpath(fullfile(root,'UR5','matlab仿真'));
addpath(genpath(fullfile(root,'RRT'))); s=load(file,'result'); r=s.result;
r.scene.jointLimits=deriveUR5JointLimits(r.scene.robot); % Rehydrate removed derived field, same immutable URDF limits.
if ~isfolder(out), mkdir(out); end
audit=struct('source',file,'result_fields',{fieldnames(r)},'executionReady',false,'robot_motion','NO');
flags={'configurationComplete','complete','adaptiveCorrectionComplete','nodeCollisionFree','edgeCollisionFree',...
    'configurationNodeCollisionFree','configurationEdgeCollisionFree','ikSuccessAll','highPrecisionPathAvailable',...
    'precisionStageInvocationCount','timeParameterizationComplete','timeScalingAlgorithmTimeSec',...
    'originalSeamPointCount','maxJointJumpDeg','configurationMaxJointJumpDeg'};
for k=1:numel(flags), if isfield(r,flags{k}), audit.flags.(flags{k})=r.(flags{k}); end, end
audit.options=r.options; audit.layers=cell(1,numel(r.stateBank));
for i=1:numel(r.stateBank)
    a=struct('layer',i,'valid_states',numel(r.stateBank{i}),...
        'reachable_states',sum(r.graph.reachable{i}),'stats',r.layerStats(i));
    d=r.candidateSelectionDiagnostics{i};
    for f={'candidate_count','connected_candidate_count','retained_count','point_status'}
        if isfield(d,f{1}), a.(f{1})=d.(f{1}); end
    end
    audit.layers{i}=a;
end
audit.graph=r.graph; audit.continuation_stats=rmfield(r.continuationStats,'diagnosticCandidatePools');
audit.first_point=struct();
for f=fieldnames(r.firstPointDiagnostics).'
    v=r.firstPointDiagnostics.(f{1});
    if (isnumeric(v)||islogical(v)) && isscalar(v), audit.first_point.(f{1})=v; end
end
% These are the actually retained numeric/pose-valid diagnostic rows, not
% invented total IK-call counts. Each target position maps to the final seam.
audit.retained_pool_filters=cell(1,numel(r.stateBank));
for i=1:numel(r.stateBank)
    audit.retained_pool_filters{i}=struct('layer',i,'pose_valid_records',0,'accepted_records',0,...
        'direction_rejected',0,'side_rejected',0,'environment_rejected',0,'self_rejected',0,...
        'native_collision_pairs',{{}});
end
for ip=1:numel(r.continuationStats.diagnosticCandidatePools)
    p=r.continuationStats.diagnosticCandidatePools{ip};
    [distance,i]=min(vecnorm(r.seam.positionsMat-p.Ttarget(1:3,4).',2,2));
    assert(distance<1e-5,'Cannot identify diagnostic target layer');
    row=audit.retained_pool_filters{i}; row.pose_valid_records=row.pose_valid_records+numel(p.rows);
    for kd=1:numel(p.rows)
        dc=p.rows(kd); reason=char(dc.rejection_reason);
        row.accepted_records=row.accepted_records+strcmp(reason,'accepted');
        row.direction_rejected=row.direction_rejected+contains(reason,'orientation_direction');
        row.side_rejected=row.side_rejected+contains(reason,'side_position');
        row.environment_rejected=row.environment_rejected+strcmp(reason,'workpiece_collision');
        row.self_rejected=row.self_rejected+strcmp(reason,'self_collision');
        if any(strcmp(reason,{'workpiece_collision','self_collision'}))
            q=reshape(dc.q_rad,1,6); [env,ed]=check_scene_environment_collision(r.scene,q);
            [own,sd]=checkCollision(r.scene.robot,q,'IgnoreSelfCollision','off',...
                'SkippedSelfCollisions',r.scene.selfCollisionSkipPairs,'Exhaustive','on');
            row.native_collision_pairs=[row.native_collision_pairs pairNames(r.scene,env,ed,own,sd)];
        end
    end
    row.native_collision_pairs=unique(row.native_collision_pairs); audit.retained_pool_filters{i}=row;
end
audit.bridge=r.cartesianBridge; audit.sparse_q=r.qPathInitial;
audit.refinement_flags=struct('complete',r.cartesianRefinement.complete,...
    'failed_sparse_edges',find(~[r.cartesianRefinement.segmentReport.correctionAccepted]),...
    'failed_dense_edges',find(~r.cartesianRefinement.finalTcpMetrics.segmentCollisionFree),...
    'sparse_nodes',size(r.qPathInitial,1),'diagnostic_dense_nodes',size(r.cartesianRefinement.qPathExecution,1),...
    'public_qPath_rows',size(r.qPath,1),'public_qPathExecution_rows',size(r.qPathExecution,1));
audit.segment_reports=r.cartesianRefinement.segmentReport;
audit.final_metrics=r.cartesianRefinement.finalTcpMetrics;
audit.classification='PRECISION_FAILURE'; if r.adaptiveCorrectionComplete, audit.classification='PASS'; end
audit.preweld=struct('candidate_exists',~isempty(r.qPathInitial),'official_generated',false,...
    'reason','Complete precision-valid seam required before exporting a PreWeld');
audit.preweld.diagnostic_endpoint=validate_hmi_endpoint(r.scene,r.qPathInitial(1,:),'DIAGNOSTIC_PREWELD',struct());
Q=r.cartesianRefinement.qPathExecution; bad=find(~r.cartesianRefinement.finalTcpMetrics.segmentCollisionFree);
envNames=hmi_environment_names(r.scene); bodyNames=[r.scene.robot.BodyNames {r.scene.robot.BaseName}];
audit.bad_edge_audit={};
for e=reshape(bad,1,[])
    dq=Q(e+1,:)-Q(e,:); count=max(101,2*ceil(max(abs(rad2deg(dq)))/.25)+1);
    edge=struct('edge',e,'samples',count,'colliding_samples',0,'axis_rejections',0,...
        'side_rejections',0,'limit_rejections',0,'pairs',{{}},'rejected_samples',{{}});
    for j=0:count-1
        u=j/(count-1); q=Q(e,:)+u*dq;
        [env,de]=check_scene_environment_collision(r.scene,q);
        [own,ds]=checkCollision(r.scene.robot,q,'IgnoreSelfCollision','off',...
            'SkippedSelfCollisions',r.scene.selfCollisionSkipPairs,'Exhaustive','on');
        T=getTransform(r.scene.robot,q,r.scene.tcpBodyName);
        R=r.scene.T_MatBase_Workpiece(1:3,1:3).'; g=T(1:3,1:3)*r.scene.wireAxisTcp(:);
        gp=R*g; axis=r.scene.weldGunAxisWP(:); angle=acosd(max(-1,min(1,dot(gp/norm(gp),axis/norm(axis)))));
        head=R*(T(1:3,4)-r.scene.hullHeadDistanceM*g-r.scene.T_MatBase_Workpiece(1:3,4));
        axisFail=angle>r.options.sideToleranceDeg; sideFail=~(head(2)<.230-r.options.sideMarginM && head(3)>.005+r.options.sideMarginM);
        limitFail=any(q<r.scene.jointLimits(:,1).' | q>r.scene.jointLimits(:,2).');
        hit=any(env(:))||any(own(:)); pairs=pairNames(r.scene,env,de,own,ds);
        edge.colliding_samples=edge.colliding_samples+hit;
        edge.axis_rejections=edge.axis_rejections+axisFail; edge.side_rejections=edge.side_rejections+sideFail;
        edge.limit_rejections=edge.limit_rejections+limitFail; edge.pairs=unique([edge.pairs pairs]);
        if hit||axisFail||sideFail||limitFail
            d=[de(:);ds(:)]; d=d(isfinite(d)&d>0); gap=[]; if ~isempty(d), gap=1000*min(d); end
            edge.rejected_samples{end+1}=struct('u',u,'q_rad',q,'TCP',T,'pairs',{pairs},...
                'axis_angle_deg',angle,'axis_rejected',axisFail,'side_rejected',sideFail,...
                'limits_rejected',limitFail,'minimum_positive_gap_mm',gap); %#ok<AGROW>
        end
    end
    audit.bad_edge_audit{end+1}=edge;
end
audit.missing_statistics_note=['Saved layerStats does not contain all generator rejection counters or rejected collision pairs. '...
    'No per-layer total collision rejection counts are invented; above native audit covers selected failed edges.'];
writeJson(fullfile(out,'saved_result_audit.json'),audit);
fprintf('READONLY_AUDIT configuration=%d precision=%d bad_edges=%s\n',r.configurationComplete,r.adaptiveCorrectionComplete,mat2str(bad));
end
function writeJson(path,data)
f=fopen(path,'w'); c=onCleanup(@()fclose(f)); fwrite(f,jsonencode(data,'PrettyPrint',true),'char');
end
function pairs=pairNames(scene,env,de,own,ds)
% Native collision flags are scalar/vector aggregates, NOT body-pair indices.
names=scene.robot.BodyNames; en=hmi_environment_names(scene); pairs={};
if any(env(:))
    [a,b]=find(isnan(de));
    for k=1:numel(a)
        if a(k)<=numel(names) && ~isempty(getBody(scene.robot,names{a(k)}).Collisions)
            pairs{end+1}=[names{a(k)} ' <-> ' en{b(k)}]; %#ok<AGROW>
        end
    end
end
if any(own(:))
    [a,b]=find(triu(isnan(ds),1));
    for k=1:numel(a)
        if max(a(k),b(k))>numel(names), continue; end
        x=names{a(k)}; y=names{b(k)}; skips=scene.selfCollisionSkipPairs;
        ignored=any((strcmp(skips(:,1),x)&strcmp(skips(:,2),y))|(strcmp(skips(:,1),y)&strcmp(skips(:,2),x)));
        if ~ignored && ~isempty(getBody(scene.robot,x).Collisions) && ~isempty(getBody(scene.robot,y).Collisions)
            pairs{end+1}=[x ' <-> ' y]; %#ok<AGROW>
        end
    end
end
end
