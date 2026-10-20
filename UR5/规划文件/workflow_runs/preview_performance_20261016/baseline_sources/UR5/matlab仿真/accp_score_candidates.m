function [states,audit] = accp_score_candidates(scene,seam,point,states,prev,maxCount,options)
% Audit node-valid candidates before beam pruning. Ranking changes no gate.
% Costs: joint distance rad, axis error rad, inverse normalized limit margin,
% inverse clearance (10 mm reference). Lookahead is a bounded prediction,
% not a proof of reachability; it is deliberately excluded from the score.
savedRng=rng; restoreRng=onCleanup(@() rng(savedRng)); %#ok<NASGU>
homeFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'ur5_hmi','config','hmi_config.yaml');
txt=fileread(homeFile);
block=regexp(txt,'(?m)^real_home:\s*\n((?:\s*-\s*[^\n]+\n){6})','tokens','once');
if isempty(block), error('ACCP:RealHome','Saved RealHome unavailable for candidate audit'); end
values=regexp(block{1},'(?m)^\s*-\s*([^\n]+)','tokens');
qHome=cellfun(@(v)str2double(v{1}),values).';
validateattributes(qHome,{'double'},{'numel',6,'finite'});
limits=localLimits(scene.robot);
bank=states;
if isKey(scene.candidateAuditStore,point), bank=scene.candidateAuditStore(point); end
% Preserve distinct joints AND redundant pose parameters. Record duplicated
% seed hits once, with multiplicity for a complete search audit.
uniqueIdx=[]; multiplicity=[];
for k=1:numel(bank)
    match=0;
    for j=1:numel(uniqueIdx)
        b=bank(uniqueIdx(j));
        if max(abs(bank(k).q-b.q))<1e-7 && ...
                abs(bank(k).travelDeg-b.travelDeg)<1e-7 && ...
                abs(bank(k).rollDeg-b.rollDeg)<1e-7
            match=j; break;
        end
    end
    if match==0, uniqueIdx(end+1)=k; multiplicity(end+1)=1; %#ok<AGROW>
    else, multiplicity(match)=multiplicity(match)+1; end
end
records=repmat(localRecord(),numel(uniqueIdx),1);
stateScores=inf(numel(states),1);
% Independent IK solver prevents diagnostics changing production solver state.
predictionScene=scene;
predictionScene.ikSolver=inverseKinematics('RigidBodyTree',scene.robot);
for k=1:numel(uniqueIdx)
    st=bank(uniqueIdx(k)); r=localRecord(); r.q_deg=rad2deg(st.q);
    r.pose_travel_deg=st.travelDeg; r.pose_roll_deg=st.rollDeg;
    r.seed_hit_count=multiplicity(k);
    r.realhome_joint_distance_rad=norm(st.q-qHome);
    r.realhome_joint_delta_deg=rad2deg(st.q-qHome);
    r.previous_reference='previous retained beam';
    if isempty(prev)
        reference=qHome; r.previous_reference='Saved RealHome (Point1 reference only)';
    else
        Q=vertcat(prev.q); [~,best]=min(vecnorm(Q-st.q,2,2)); reference=Q(best,:);
    end
    r.previous_joint_delta_deg=rad2deg(st.q-reference);
    r.previous_joint_distance_rad=norm(st.q-reference);
    r.gun_axis_angle_error_deg=acosd(max(-1,min(1,st.sideDot)));
    margin=min(st.q-limits(:,1).',limits(:,2).'-st.q);
    r.joint_limit_margin_deg=rad2deg(margin);
    normalized=margin./(limits(:,2)-limits(:,1)).';
    r.joint_limit_cost=mean(1./max(normalized,1e-6));
    [r.collision_clearance_mm,r.clearance_status]=localClearance(scene,st.q);
    if isfinite(r.collision_clearance_mm)
        r.collision_cost=10/max(r.collision_clearance_mm,0.001);
    else
        r.collision_cost=1e6;
    end
    r.cost_terms=[r.previous_joint_distance_rad deg2rad(r.gun_axis_angle_error_deg) ...
        r.joint_limit_cost r.collision_cost];
    r.score=dot(double(options.candidate_score_weights(:)),r.cost_terms(:));
    % states contains only candidates passing the original parent/edge gates.
    for s=1:numel(states)
        if max(abs(states(s).q-st.q))<1e-7 && ...
                abs(states(s).travelDeg-st.travelDeg)<1e-7 && ...
                abs(states(s).rollDeg-st.rollDeg)<1e-7
            r.parent_edge_pass=true;
            if ~isempty(prev)
                r.previous_joint_distance_rad=norm(st.q-states(s).parentQ);
                r.previous_joint_delta_deg=rad2deg(st.q-states(s).parentQ);
                r.cost_terms(1)=r.previous_joint_distance_rad;
                r.score=dot(double(options.candidate_score_weights(:)),r.cost_terms(:));
            end
            stateScores(s)=min(stateScores(s),r.score);
        end
    end
    if point==seam.numPoints
        r.next_reachability='END_OF_SEAM';
    else
        [r.next_reachability,r.next_reachable_count,r.next_min_joint_distance_rad]= ...
            localPredict(predictionScene,seam,point,st,options);
    end
    records(k)=r;
end
if options.candidate_scoring_mode
    [~,order]=sort(stateScores,'ascend');
else
    if isempty(states), order=[];
    else, [~,order]=sort(vertcat(states.continuationCost),'ascend'); end
end
selected=order(1:min(maxCount,numel(order)));
for k=1:numel(records)
    for s=selected(:).'
        if max(abs(rad2deg(states(s).q)-records(k).q_deg))<rad2deg(1e-7) && ...
                abs(states(s).travelDeg-records(k).pose_travel_deg)<1e-7 && ...
                abs(states(s).rollDeg-records(k).pose_roll_deg)<1e-7
            records(k).retained_in_beam=true;
        end
    end
end
audit=struct('point_index',point,'candidate_count',numel(records), ...
    'connected_candidate_count',numel(states),'retained_count',numel(selected), ...
    'realhome_source',homeFile,'q_realhome_deg',rad2deg(qHome), ...
    'cost_units','joint rad; axis rad; inverse normalized limit margin; 10mm/clearance', ...
    'prediction_scope','bounded next-point IK plus original pose/joint/edge collision constraints', ...
    'candidates',{num2cell(records)});
audit.point_status='SEARCHED';
if isempty(states), audit.point_status='NO_CONNECTED_CANDIDATE'; end
if point>1 && isempty(prev), audit.point_status='NO_PREVIOUS_CANDIDATE'; end
audit.parent_edge_pass_note='Exact membership in connected deduplicated pool; absence alone does not prove an edge collision.';
states=states(order);
fprintf('Candidate audit Point %d: node=%d connected=%d kept=%d scoring=%d\n', ...
    point,numel(records),numel(states),numel(selected),options.candidate_scoring_mode);
end

function r=localRecord()
r=struct('q_deg',zeros(1,6),'pose_travel_deg',0,'pose_roll_deg',0, ...
 'seed_hit_count',0,'previous_reference','','previous_joint_delta_deg',zeros(1,6), ...
 'previous_joint_distance_rad',NaN,'realhome_joint_delta_deg',zeros(1,6), ...
 'realhome_joint_distance_rad',NaN,'gun_axis_angle_error_deg',NaN, ...
 'joint_limit_margin_deg',zeros(1,6),'joint_limit_cost',NaN, ...
 'collision_clearance_mm',NaN,'clearance_status','UNKNOWN','collision_cost',NaN, ...
 'cost_terms',zeros(1,4),'score',NaN,'parent_edge_pass',false, ...
 'retained_in_beam',false,'next_reachability','NOT_TESTED', ...
 'next_reachable_count',0,'next_min_joint_distance_rad',NaN);
end

function limits=localLimits(robot)
limits=zeros(0,2);
for i=1:numel(robot.Bodies)
 j=robot.Bodies{i}.Joint;
 if strcmp(j.Type,'revolute'), limits(end+1,:)=j.PositionLimits; end %#ok<AGROW>
end
if size(limits,1)~=6, error('ACCP:Limits','Expected six joint limits'); end
end

function [mm,status]=localClearance(scene,q)
mm=NaN; status='UNKNOWN';
try
 [~,envDist]=checkCollision(scene.robot,q,scene.environment, ...
     'IgnoreSelfCollision','on','Exhaustive','on');
 [~,selfDist]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
     'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
 distances=[envDist(:);selfDist(:)];
 distances=distances(isfinite(distances) & distances>=0);
 if ~isempty(distances), mm=min(distances)*1000; status='AVAILABLE'; end
catch ME
 status=['UNKNOWN: ' ME.identifier];
end
end

function [status,count,minDist]=localPredict(scene,seam,i,st,options)
count=0; minDist=NaN;
co=struct('positionToleranceM',options.positionToleranceM, ...
 'orientationToleranceDeg',options.orientationToleranceDeg, ...
 'sideToleranceDeg',options.sideToleranceDeg,'sideMarginM',options.sideMarginM, ...
 'checkSelfCollision',options.checkSelfCollision,'reuseCandidateCache',false, ...
 'disableAxisConstraint',false);
for travel=st.travelDeg+[-options.maxPoseStepTravelDeg 0 options.maxPoseStepTravelDeg]
 for roll=st.rollDeg+[-options.maxPoseStepRollDeg 0 options.maxPoseStepRollDeg]
  R=make_actual_gun_axis_pose(scene,seam.tangentWP,scene.weldGunAxisWP,travel,mod(roll,360));
  T=eye(4); T(1:3,1:3)=R; T(1:3,4)=seam.positionsWP(i+1,:).';
  r=generate_actual_axis_ik_candidates(scene,scene.T_MatBase_Workpiece*T,travel,mod(roll,360),st.q,co);
  for k=1:numel(r.states)
   next=r.states(k); dq=atan2(sin(next.q-st.q),cos(next.q-st.q));
   rollDelta=abs(rad2deg(atan2(sind(next.rollDeg-st.rollDeg),cosd(next.rollDeg-st.rollDeg))));
   if rollDelta>options.maxRollStepDeg+1e-9 || abs(next.travelDeg-st.travelDeg)>options.maxTravelStepDeg+1e-9 || ...
           max(abs(rad2deg(dq)))>options.maxJointStepDeg, continue; end
   n=max(1,ceil(max(abs(rad2deg(dq)))/options.edgeResolutionDeg));
   free=true;
   for t=0:n
    q=st.q+(t/n)*dq;
    env=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
    self=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
        'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
    if any(env(:)) || any(self(:)), free=false; break; end
   end
   if free
    count=count+1; d=norm(next.q-st.q);
    if isnan(minDist), minDist=d; else, minDist=min(minDist,d); end
   end
  end
 end
end
if count>0, status='REACHABLE_IN_BOUNDED_TEST'; else, status='NO_SOLUTION_IN_BOUNDED_TEST'; end
end
