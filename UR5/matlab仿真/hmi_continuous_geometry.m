function [g,r]=hmi_continuous_geometry(raw,scene)
% Sole geometry owner. Never changes original q or original TCP targets.
clock=tic; Q=[]; P={}; labels={}; maps=cell(1,3); joins=[];
for k=1:3
    a=raw.segments{k}; start=size(Q,1); n=size(a.qPath,1);
    if k==1, idx=1:n; maps{k}=1:n;
    else
        assert(max(abs(Q(end,:)-a.qPath(1,:)))<1e-10,'CP:Boundary','Joint boundary requires explicit geometry repair');
        idx=2:n; maps{k}=[start,start+(1:n-1)]; joins(end+1)=start; %#ok<AGROW>
    end
    Q=[Q;a.qPath(idx,:)]; P=[P,reshape(a.tcpPosePath(idx),1,[])]; labels=[labels,repmat({a.label},1,numel(idx))]; %#ok<AGROW>
end
assert(numel(maps{2})==47,'CP:WeldContract','Exactly 47 original weld nodes required');
% Shared joint endpoints can carry sub-micron independently computed poses.
% The immutable ACCP targets own both weld boundary targets; never alter them.
P(maps{2})=reshape(raw.segments{2}.tcpPosePath,1,[]);
ds=vecnorm(diff(Q),2,2); assert(all(ds>1e-10),'CP:Duplicate','Degenerate original edge');
s=[0;cumsum(ds)]; s=s/s(end);
ref=struct('mode','FROZEN_CARTESIAN_PATH_QUINTIC_PROGRESS_v1','s',s,'poses',{P},'duration',NaN);
g=struct('originalQ',Q,'originalP',{P},'originalS',s,'originalLabels',{labels},...
    'originalMaps',{maps},'originalJoins',joins,'reference',ref);
r=struct('pass',false,'ik_calls',0,'iterations',{{}},'executionReady',false,'failure_reason','');
% Reject inconsistent immutable nodes before any timing or optimization.
[pos,ang]=geometryErrors(scene,Q,P);
assert(max(pos)<.5 && max(ang)<=.5,'CP:OriginalGeometry','Original fixed node TCP mismatch');
r.original_max_position_mm=max(pos); r.original_max_orientation_deg=max(ang);
knots=s; nodes=Q; ik=inverseKinematics('RigidBodyTree',scene.robot);
for iteration=1:6
    pp=spline(knots,nodes.'); samples=[];
    for e=1:numel(knots)-1, samples=[samples;linspace(knots(e),knots(e+1),17).']; end %#ok<AGROW>
    samples=unique(samples); qq=ppval(pp,samples).';
    [~,~,target]=hmi_continuous_pose_at_s(ref,samples); [pos,ang]=geometryErrors(scene,qq,target);
    lim=scene.jointLimits; bound=any(qq<lim(:,1).' | qq>lim(:,2).',2);
    bad=find(pos>.30 | ang>.30 | bound);
    r.iterations{end+1}=struct('number',iteration,'geometry_nodes',numel(knots),...
        'max_position_mm',max(pos),'max_orientation_deg',max(ang),'joint_limits_pass',~any(bound));
    fprintf('CP_GEOMETRY iter=%d knots=%d pos=%.6f ori=%.6f bad=%d\n',iteration,numel(knots),max(pos),max(ang),numel(bad));
    if isempty(bad), r.pass=true; break; end
    if iteration==6 || numel(knots)>1800 || toc(clock)>150
        r.failure_reason='GEOMETRY_REFINEMENT_BUDGET'; break;
    end
    % At most one worst offending sample per current span per iteration.
    edges=discretize(samples(bad),[-Inf;knots(2:end-1);Inf]); selected=[];
    for e=unique(edges).'
        indices=bad(edges==e); [~,worst]=max(max(pos(indices),ang(indices)));
        selected(end+1)=indices(worst); %#ok<AGROW>
    end
    newS=[]; newQ=[];
    for index=selected
        u=samples(index);
        if any(abs(knots-u)<1e-12), r.failure_reason='FIXED_NODE_INVALID'; break; end
        seed=qq(index,:); targetPose=target{index}; found=false;
        near=find(knots<u,1,'last'); seeds=[seed;nodes(near,:);nodes(near+1,:)];
        for trial=1:size(seeds,1)
            r.ik_calls=r.ik_calls+1;
            [q,~]=ik(scene.tcpBodyName,targetPose,[1 1 1 1 1 1],seeds(trial,:));
            q=q+2*pi*round((seed-q)/(2*pi));
            [pe,ae]=geometryErrors(scene,q,{targetPose});
            if pe>.03 || ae>.03 || any(q<lim(:,1).' | q>lim(:,2).') || max(abs(q-seed))>deg2rad(15), continue; end
            [env,~]=check_scene_environment_collision(scene,q);
            [self,~]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off',...
                'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
            free=~any(env(:)) && ~any(self(:));
            if free, found=true; break; end
        end
        if ~found, r.failure_reason='LOCAL_CARTESIAN_IK_OR_NODE_COLLISION'; break; end
        newS(end+1,1)=u; newQ(end+1,:)=q; %#ok<AGROW>
    end
    if ~isempty(r.failure_reason), break; end
    [knots,order]=sort([knots;newS]); nodes=[nodes;newQ]; nodes=nodes(order,:);
end
g.knots=knots; g.nodes=nodes; g.pp=pp;
r.elapsed_sec=toc(clock); r.inserted_geometry_nodes=size(nodes,1)-size(Q,1);
r.collision_status='NEW_IK_NODES_CHECKED_FULL_CURVE_DEFERRED_TO_FINAL_AUDIT';
assert(isequal(nodes(ismember(knots,s),:),Q),'CP:NodeMutation','Original q nodes changed');
end
function [p,a]=geometryErrors(scene,Q,P)
p=zeros(size(Q,1),1); a=p;
for k=1:size(Q,1)
    T=getTransform(scene.robot,Q(k,:),scene.tcpBodyName); U=P{k};
    p(k)=1000*norm(T(1:3,4)-U(1:3,4));
    a(k)=rad2deg(acos(max(-1,min(1,(trace(T(1:3,1:3)*U(1:3,1:3)')-1)/2))));
end
end
