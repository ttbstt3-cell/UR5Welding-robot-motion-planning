function report=hmi_audit_joint_interface_path(scene,Q,samplesPerEdge)
% Audit actual numeric interpolation, NEVER periodic/short-angle edges.
% Same robot, tool, wire, environment and certified contact/skip policy.
if nargin<3, samplesPerEdge=101; end
Q=double(Q); assert(size(Q,2)==6 && size(Q,1)>=2 && all(isfinite(Q(:))));
lim=scene.jointLimits; model=deriveUR5JointLimits(scene.robot);
lim=[max(lim(:,1),model(:,1)),min(lim(:,2),model(:,2))];
queries=[];
if isfield(scene,'previewExactQueryCache') && scene.previewExactQueryCache
    queries=HmiExactSceneQueries(scene,scene.previewQueryBinding,true);
end
report=struct('pass',false,'executionReady',false,'joint_limits_rad',lim,...
    'joint_limits_pass',all(Q>=lim(:,1).' & Q<=lim(:,2).','all'),...
    'sample_count',0,'collision_sample_count',0,'limit_sample_count',0,...
    'collision_pairs',{{}},'first_failure',[],'min_positive_distance_mm',NaN,...
    'maximum_numeric_joint_edge_deg',rad2deg(max(abs(diff(Q)),[],'all')),...
    'interpolation','ACTUAL_NUMERIC_LINEAR_NO_WRAP','sampled_not_continuous_safety_proof',true);
for e=1:size(Q,1)-1
    count=max(samplesPerEdge,1+ceil(max(abs(Q(e+1,:)-Q(e,:)))/deg2rad(0.5)));
    for u=linspace(0,1,count)
        q=(1-u)*Q(e,:)+u*Q(e+1,:); report.sample_count=report.sample_count+1;
        if any(q<lim(:,1).' | q>lim(:,2).')
            report.limit_sample_count=report.limit_sample_count+1;
            if isempty(report.first_failure)
                report.first_failure=struct('edge',e,'u',u,'q_rad',q,'reason','JOINT_LIMIT','collision_pairs',{{}});
            end
            continue;
        end
        if isempty(queries)
            [env,de]=check_scene_environment_collision(scene,q);
            [self,ds]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off',...
                'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
        else
            [env,de]=queries.environment(q); [self,ds]=queries.self(q);
        end
        d=[de(:);ds(:)]; d=d(isfinite(d)&d>0);
        if ~isempty(d)
            gap=1000*min(d);
            if ~isfinite(report.min_positive_distance_mm) || gap<report.min_positive_distance_mm
                report.min_positive_distance_mm=gap;
            end
        end
        if any(env(:)) || any(self(:))
            report.collision_sample_count=report.collision_sample_count+1;
            if isempty(report.first_failure)
                detail=validate_hmi_endpoint(scene,q,'INTERFACE COLLISION');
                report.first_failure=struct('edge',e,'u',u,'q_rad',q,'reason','COLLISION',...
                    'TCP_pose',getTransform(scene.robot,q,scene.tcpBodyName),...
                    'collision_pairs',{detail.collision_pairs},'categories',detail.collision_categories);
                report.collision_pairs=detail.collision_pairs;
            end
        end
    end
end
report.pass=report.joint_limits_pass && report.limit_sample_count==0 && report.collision_sample_count==0;
report.distance_scope='Minimum finite positive pair distance; not signed penetration or certified physical margin';
if ~isempty(queries), report.performance=queries.summary(); end
end
