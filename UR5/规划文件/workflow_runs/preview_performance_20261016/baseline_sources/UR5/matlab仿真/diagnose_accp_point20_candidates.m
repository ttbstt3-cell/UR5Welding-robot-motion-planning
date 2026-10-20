function report=diagnose_accp_point20_candidates(inputFile,outputDir)
% Diagnostic-only replay of the failed real-scene ACCP Point20.
% It reuses the existing IK/collision functions and changes no planner gate.
if nargin<1||isempty(inputFile)
 inputFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'规划文件','real_anchor_runs','full_accp_wire3_20261013','real_seam_round_recovery_beam3.mat');
end
if nargin<2||isempty(outputDir), outputDir=fullfile(fileparts(inputFile),'point20_audit_20261013'); end
if ~isfolder(outputDir), mkdir(outputDir); end
S=load(inputFile,'result'); result=S.result; assert(~isempty(result.stateBank{19}));
scene=result.scene; shortenMm=double(result.options.wire_collision_shorten_mm);
assert(isfield(scene,'wireCollisionShortenMm')&&scene.wireCollisionShortenMm==shortenMm&&shortenMm==3, ...
 'Saved scene is not the fixed 3 mm run');
scene.selfCollisionSkipPairs=structuralSkips(scene.robot);
scene.selfCollisionSkipPairs=ignore_fixed_gun_wire_self_collision(scene.robot,scene.selfCollisionSkipPairs);
seam=result.seam; prev=result.stateBank{19}; qPrev=prev(1).q; o=result.options;
o.auditNormalRollDeg=o.maxPoseStepRollDeg; o.auditNormalTravelDeg=o.maxPoseStepTravelDeg; o.auditNormalSeedPerturbDeg=o.seedPerturbDeg;
% Values below are the already-used no-retry recovery branch, reconstructed
% from the saved run; they are not new production parameters.
o.maxPoseSeeds=max(o.maxPoseSeeds,12); o.maxPoseNeighbors=25;
o.maxPoseStepRollDeg=max(o.maxPoseStepRollDeg,45); o.maxPoseStepTravelDeg=max(o.maxPoseStepTravelDeg,15);
o.seedPerturbDeg=max(o.seedPerturbDeg,15); o.reuseCandidateCache=false;
o.diagnosticCollectIKCandidates=true; o.diagnosticReferenceQ=qPrev; o.diagnosticMaxJointDeltaDeg=inf;
qPred=qPrev; if isfield(prev,'velocityQ'), qPred=qPred+prev.velocityQ; end
jobs=makeJobs(prev,qPred,o);
names=scene.robot.BodyNames; envNames={scene.workpieceProxyInfo.name}; c=counts0(); records=repmat(record0(),0,1); stateRecords=repmat(stateRecord0(),0,1);
fprintf('Point20 audit: %d pose/seed jobs, axis %.3f deg, wire shorten %.3f mm\n',numel(jobs),o.sideToleranceDeg,shortenMm);
for ip=1:numel(jobs)
 travel=jobs(ip).travel; roll=jobs(ip).roll; seeds=jobs(ip).seeds;
 [R,~]=make_actual_gun_axis_pose(scene,seam.tangentWP,scene.weldGunAxisWP,travel,roll);
 T=eye(4); T(1:3,1:3)=R; T(1:3,4)=seam.positionsWP(20,:).'; T=scene.T_MatBase_Workpiece*T;
 co=optionsFor(o); r=generate_actual_axis_ik_candidates(scene,T,travel,roll,seeds,co);
 c.ik_calls=c.ik_calls+r.numIKCalls; c.ik_numeric_success=c.ik_numeric_success+r.numIKNumericSuccess; c.ik_pose_success=c.ik_pose_success+r.numIKPoseOK;
 c.direction_rejected=c.direction_rejected+r.numDirectionRejected; c.side_position_rejected=c.side_position_rejected+r.numWeldingSidePositionRejected; c.joint_limit_rejected=c.joint_limit_rejected+r.numJointRejected;
 c.position_rejected=c.position_rejected+r.numPositionRejected; c.orientation_rejected=c.orientation_rejected+r.numOrientationRejected; c.workpiece_collision_rejected=c.workpiece_collision_rejected+r.numEnvironmentRejected;
 c.self_collision_rejected=c.self_collision_rejected+r.numSelfRejected; c.robot_self_collision_rejected=c.robot_self_collision_rejected+r.numRobotSelfRejected; c.gun_collision_rejected=c.gun_collision_rejected+r.numGunRejected; c.node_accepted=c.node_accepted+r.numAccepted;
 for k=1:numel(r.diagnosticCandidates)
  dc=r.diagnosticCandidates(k); a=audit(scene,dc.q_rad,names,envNames); z=record0(); z.pose_index=ip; z.travel_deg=travel; z.roll_deg=roll; z.seed_index=dc.seed_index; z.q_rad=dc.q_rad; z.q_deg=dc.q_deg; z.tcp_pose=dc.tcp_pose; z.position_error_mm=dc.position_error_mm; z.orientation_error_deg=dc.orientation_error_deg; z.axis_angle_deg=dc.axis_angle_deg; z.initial_rejection=char(string(dc.rejection_reason)); z.joint_delta_from_point19_deg=rad2deg(wrapr(dc.q_rad-qPrev)); z.joint_distance_from_point19_rad=norm(wrapr(dc.q_rad-qPrev)); z.collision_pairs=a.colliding_labels; z.all_pairs=a.all_labels; z.environment_collision=a.environment_collision; z.self_collision=a.self_collision; z.wire_collision=a.wire_collision; z.gun_collision=a.gun_collision; z.robot_self_collision=a.robot_self_collision; z.nearest_distance_mm=a.nearest_distance_mm; z.min_positive_clearance_mm=a.min_positive_clearance_mm; z.node_filter_pass=strcmp(z.initial_rejection,'accepted'); records(end+1,1)=z; %#ok<AGROW>
  if a.wire_collision, c.wire_collision_rejected=c.wire_collision_rejected+1; end
  if a.gun_collision && a.self_collision, c.gun_pair_collision_candidates=c.gun_pair_collision_candidates+1; end
 end
 for k=1:numel(r.states)
  st=r.states(k); [pi,pd]=bestParent(scene,prev,st,o); sr=stateRecord0(); sr.pose_index=ip; sr.travel_deg=travel; sr.roll_deg=roll; sr.q_rad=st.q; sr.q_deg=rad2deg(st.q); sr.tcp_pose=st.Tactual; sr.joint_delta_from_point19_deg=rad2deg(wrapr(st.q-qPrev)); sr.joint_distance_from_point19_rad=norm(wrapr(st.q-qPrev)); sr.node_filter_pass=true; sr.parent_edge_pass=pi>0; sr.continuity_rejection=pi==0; sr.nearest_distance_mm=nearest(scene,st.q); if pi>0, sr.parent_joint_delta_deg=rad2deg(pd); else, [sr.parent_rejection_reason,sr.edge_collision_pairs,sr.edge_nearest_distance_mm]=parentFailureAudit(scene,prev,st,o); end; stateRecords(end+1,1)=sr; %#ok<AGROW>
  if pi>0
   c.connected_candidates=c.connected_candidates+1;
  else
   c.continuity_rejected=c.continuity_rejected+1;
   if strcmp(sr.parent_rejection_reason,'joint_step'), c.joint_step_continuity_rejected=c.joint_step_continuity_rejected+1; end
   if strcmp(sr.parent_rejection_reason,'pose_step'), c.pose_step_continuity_rejected=c.pose_step_continuity_rejected+1; end
   if strcmp(sr.parent_rejection_reason,'edge_collision')
    c.edge_collision_rejected=c.edge_collision_rejected+1;
    if any(contains(sr.edge_collision_pairs,'WeldingGunWire')), c.wire_collision_edge_rejected=c.wire_collision_edge_rejected+1; end
    if any(contains(sr.edge_collision_pairs,'WeldingGunMount'))||any(contains(sr.edge_collision_pairs,'WeldTCP')), c.gun_collision_edge_rejected=c.gun_collision_edge_rejected+1; end
    if any(contains(sr.edge_collision_pairs,'Workpiece')), c.workpiece_collision_edge_rejected=c.workpiece_collision_edge_rejected+1; end
   end
  end
 end
end
% Propagate exact parent/edge results to the records and rank diagnostic-only
% candidates. Node-valid candidates remain diagnostic even if their edge fails.
for k=1:numel(stateRecords), h=findRec(records,stateRecords(k)); if h>0, records(h).node_filter_pass=true; records(h).parent_edge_pass=stateRecords(k).parent_edge_pass; records(h).continuity_rejection=stateRecords(k).continuity_rejection; records(h).parent_rejection_reason=stateRecords(k).parent_rejection_reason; records(h).edge_collision_pairs=stateRecords(k).edge_collision_pairs; records(h).edge_nearest_distance_mm=stateRecords(k).edge_nearest_distance_mm; end, end
for k=1:numel(records)
 if records(k).node_filter_pass && ~records(k).parent_edge_pass
  st=struct('q',records(k).q_rad,'travelDeg',records(k).travel_deg,'rollDeg',records(k).roll_deg);
  [rr,pp,nn]=parentFailureAudit(scene,prev,st,o);
  records(k).parent_rejection_reason=rr; records(k).edge_collision_pairs=pp; records(k).edge_nearest_distance_mm=nn;
 end
end
if isempty(records), top=records; else
 valid=find([records.node_filter_pass]); if isempty(valid), valid=1:numel(records); end
 stage=zeros(numel(valid),1); pc=zeros(numel(valid),1); clr=zeros(numel(valid),1); jd=zeros(numel(valid),1);
 for k=1:numel(valid), ii=valid(k); stage(k)=~records(ii).node_filter_pass; pc(k)=numel(records(ii).collision_pairs); clr(k)=records(ii).min_positive_clearance_mm; jd(k)=records(ii).joint_distance_from_point19_rad; end
 clr(~isfinite(clr))=0; [~,so]=sortrows([stage pc -clr jd],[1 2 3 4]); ord=valid(so);
 top=repmat(record0(),0,1);
 for ii=ord(:).'
  duplicate=false;
  for jj=1:numel(top)
   if max(abs(top(jj).q_deg-records(ii).q_deg))<0.01 && ...
       abs(top(jj).travel_deg-records(ii).travel_deg)<1e-7 && ...
       abs(top(jj).roll_deg-records(ii).roll_deg)<1e-7
    duplicate=true; break;
   end
  end
  if ~duplicate, top(end+1)=records(ii); if numel(top)>=10, break; end, end %#ok<AGROW>
 end
 for k=1:numel(top), top(k).rank=k; end
end
c.final_status='NO_CONNECTED_CANDIDATE'; if c.connected_candidates>0, c.final_status='CONNECTED_CANDIDATE_AVAILABLE'; end
c.point_index=20; c.previous_point_index=19; c.axis_angle_tolerance_deg=o.sideToleranceDeg; c.wire_collision_shorten_mm=shortenMm;
c.note='Counts are per pose/seed attempt and categories can overlap. checkCollision does not expose signed penetration depth.';
report=struct('schema_version','ACCP_POINT20_FILTER_AUDIT_v1','input_file',inputFile,'output_dir',outputDir,'point_index',20,'previous_point_index',19,'robot_motion','NO','planner_logic_changed',false,'collision_rules_changed',false,'tcp_changed',false,'accp_parameters_changed',false,'wire_collision_shorten_mm',shortenMm,'replay_branch','normal neighborhood + angular fallback 10/20 + no-retry recovery beam-3','job_count',numel(jobs),'pose_list',vertcat(jobs.pose),'filter_counts',c,'candidate_count_pose_valid',numel(records),'connected_candidate_count',c.connected_candidates,'top10_ranking','Node-valid first; then collision-pair count, larger clearance, smaller Point19 joint distance. Diagnostic only.','top10_nearest_candidates',top,'calibration_binding',scene.calibrationBinding,'axis_source',scene.wireAxisTcp,'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API');
writeJson(fullfile(outputDir,'point20_filter_summary.json'),report); writeJson(fullfile(outputDir,'point20_all_candidates.json'),struct('schema_version','ACCP_POINT20_ALL_CANDIDATES_v1','candidate_count',numel(records),'candidates',{records})); writeJson(fullfile(outputDir,'point20_top10_nearest.json'),struct('schema_version','ACCP_POINT20_TOP10_v1','candidates',{top},'ranking',report.top10_ranking)); writeTopCsv(fullfile(outputDir,'point20_top10_nearest.csv'),top); writeCounts(fullfile(outputDir,'point20_filter_counts.csv'),c);
fprintf('\nPoint20: IK calls=%d numeric=%d pose-valid=%d\n',c.ik_calls,c.ik_numeric_success,c.ik_pose_success); fprintf('direction=%d joint-limit=%d workpiece=%d gun=%d wire=%d self=%d continuity=%d\n',c.direction_rejected,c.joint_limit_rejected,c.workpiece_collision_rejected,c.gun_collision_rejected,c.wire_collision_rejected,c.self_collision_rejected,c.continuity_rejected); fprintf('node accepted=%d connected=%d status=%s\n',c.node_accepted,c.connected_candidates,c.final_status);
for k=1:numel(top), fprintf('TOP%02d q=[%s] pairs=%d nearest=%.6f mm dJ19=%.6f rad parent=%d\n',k,sprintf(' %.3f',top(k).q_deg),numel(top(k).collision_pairs),top(k).nearest_distance_mm,top(k).joint_distance_from_point19_rad,top(k).parent_edge_pass); end
end

function jobs=makeJobs(prev,qPred,o)
% Reconstruct the Point20 attempts made by the normal layer, both angular
% redundancy retries, and the no-retry bounded recovery layer.
specs={struct('name','normal','roll',o.auditNormalRollDeg,'travel',o.auditNormalTravelDeg,'neighbors',o.maxPoseNeighbors,'perturb',o.auditNormalSeedPerturbDeg,'extra',false), ...
 struct('name','angular10','roll',10,'travel',0,'neighbors',9,'perturb',8,'extra',false), ...
 struct('name','angular20','roll',20,'travel',5,'neighbors',9,'perturb',8,'extra',false), ...
 struct('name','recovery','roll',45,'travel',15,'neighbors',25,'perturb',15,'extra',true)};
% The caller has already raised the local recovery copy to 15; the normal
% value is retained above before that local diagnostic override.
jobs=repmat(struct('travel',0,'roll',0,'seeds',zeros(0,6),'pose',[0 0],'pass',''),0,1);
centre=[prev(1).travelDeg prev(1).rollDeg];
for s=1:numel(specs)
 sp=specs{s}; dr=[-sp.roll 0 sp.roll]; dt=[-sp.travel 0 sp.travel]; p=zeros(0,2);
 for it=1:numel(dt), for ir=1:numel(dr), p(end+1,:)=[centre(1)+dt(it),mod(centre(2)+dr(ir),360)]; end, end
 p=uniquePoseList(p,sp.neighbors,centre); ss=seedBank(qPred,prev(1).q,sp.perturb,o.maxPoseSeeds,sp.extra);
 for i=1:size(p,1), jobs(end+1)=struct('travel',p(i,1),'roll',p(i,2),'seeds',ss,'pose',p(i,:),'pass',sp.name); end %#ok<AGROW>
end
end
function ss=seedBank(qPred,qPrev,perturbDeg,maxSeeds,extra)
dp=deg2rad(perturbDeg); ss=[qPred;qPrev];
if maxSeeds>=3, q=qPred; q(4)=q(4)+dp; ss(end+1,:)=q; end
if maxSeeds>=4, q=qPred; q(4)=q(4)-dp; ss(end+1,:)=q; end
if maxSeeds>=5, q=qPred; q(5)=q(5)+dp; ss(end+1,:)=q; end
if maxSeeds>=6, q=qPred; q(5)=q(5)-dp; ss(end+1,:)=q; end
if maxSeeds>=7, q=qPred; q(6)=q(6)+dp; ss(end+1,:)=q; end
if maxSeeds>=8, q=qPred; q(6)=q(6)-dp; ss(end+1,:)=q; end
if extra && maxSeeds>=10, q=qPred; q(5)=q(5)+2*dp; ss(end+1,:)=q; q=qPred; q(5)=q(5)-2*dp; ss(end+1,:)=q; end
ss=ss(1:min(size(ss,1),maxSeeds),:);
end

function o=optionsFor(x), o=struct('positionToleranceM',x.positionToleranceM,'orientationToleranceDeg',x.orientationToleranceDeg,'sideToleranceDeg',x.sideToleranceDeg,'sideMarginM',x.sideMarginM,'checkSelfCollision',x.checkSelfCollision,'lazyCollision',x.lazyCollision,'reuseCandidateCache',false,'diagnosticCollectIKCandidates',true,'diagnosticReferenceQ',x.diagnosticReferenceQ,'diagnosticMaxJointDeltaDeg',inf,'disableAxisConstraint',x.disableAxisConstraint,'disable_axis_constraint',x.disable_axis_constraint); end
function [idx,dq]=bestParent(scene,prev,st,o), idx=0; dq=zeros(1,6); best=inf; for k=1:numel(prev), if angleDist(prev(k).rollDeg,st.rollDeg)>o.maxRollStepDeg+1e-9||abs(prev(k).travelDeg-st.travelDeg)>o.maxTravelStepDeg+1e-9, continue; end; d=wrapr(st.q-prev(k).q); if max(abs(rad2deg(d)))>o.maxJointStepDeg||sum(d.^2)>=best, continue; end; if ~edgeFree(scene,prev(k).q,st.q,o), continue; end; best=sum(d.^2); idx=k; dq=d; end, end
function [reason,pairs,nearestMm]=parentFailureAudit(scene,prev,st,o)
reason='unknown_edge_rejection'; pairs={}; nearestMm=NaN;
for k=1:numel(prev)
 if angleDist(prev(k).rollDeg,st.rollDeg)>o.maxRollStepDeg+1e-9 || abs(prev(k).travelDeg-st.travelDeg)>o.maxTravelStepDeg+1e-9
  reason='pose_step'; continue;
 end
 d=wrapr(st.q-prev(k).q);
 if max(abs(rad2deg(d)))>o.maxJointStepDeg, reason='joint_step'; continue; end
 [ok,pairs,nearestMm]=edgeAudit(scene,prev(k).q,st.q,o);
 if ok, reason='edge_pass_but_not_selected'; else, reason='edge_collision'; end
 return;
end
end
function [ok,pairs,nearestMm]=edgeAudit(scene,a,b,o)
d=wrapr(b-a); n=max(1,ceil(max(abs(rad2deg(d)))/o.edgeResolutionDeg)); ok=true; pairs={}; nearestMm=NaN;
for k=0:n
 q=a+(k/n)*d; aa=audit(scene,q,scene.robot.BodyNames,{scene.workpieceProxyInfo.name});
 if ~isnan(aa.nearest_distance_mm), nearestMm=aa.nearest_distance_mm; end
 if aa.environment_collision||aa.self_collision, ok=false; pairs=aa.colliding_labels; return; end
end
end
function ok=edgeFree(scene,a,b,o), d=wrapr(b-a); n=max(1,ceil(max(abs(rad2deg(d)))/o.edgeResolutionDeg)); ok=true; for k=0:n, if ~free(scene,a+(k/n)*d), ok=false; return; end, end, end
function ok=free(scene,q), try, e=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on'); catch, e=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end; if any(e(:)), ok=false; return; end; try, s=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on'); catch, s=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end; ok=~any(s(:)); end
function a=audit(scene,q,names,envNames), try, [e,ed]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on'); catch, [e,ed]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end; try, [s,sd]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on'); catch, [s,sd]=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end; labels={}; coll={}; vals=[]; wire=false; gun=false; robot=false; for i=1:min(numel(names),size(ed,1)), for j=1:min(numel(envNames),size(ed,2)), d=ed(i,j); if ~isinf(d), lab=[char(names{i}) ' <-> ' char(envNames{j})]; labels{end+1,1}=lab; if isfinite(d), vals(end+1)=d; end; if isnan(d), coll{end+1,1}=lab; if wireName(names{i}), wire=true; end; if gunName(names{i}), gun=true; end; end; end; end; end; for i=1:min(numel(names),size(sd,1)), for j=i+1:min(numel(names),size(sd,2)), if skippedPair(scene.selfCollisionSkipPairs,names{i},names{j}), continue; end; d=sd(i,j); if ~isinf(d), lab=[char(names{i}) ' <-> ' char(names{j})]; labels{end+1,1}=lab; if isfinite(d), vals(end+1)=d; end; if isnan(d), coll{end+1,1}=lab; hw=wireName(names{i})||wireName(names{j}); hg=gunName(names{i})||gunName(names{j}); wire=wire||hw; gun=gun||hg; robot=robot||(~hw && ~hg); end; end; end; end; near=NaN; pos=NaN; if ~isempty(vals), near=min(vals)*1000; qv=vals(vals>0); if ~isempty(qv), pos=min(qv)*1000; end; end; a=struct('environment_collision',any(e(:)),'self_collision',any(s(:)),'wire_collision',wire,'gun_collision',gun,'robot_self_collision',robot,'all_labels',{labels},'colliding_labels',{coll},'nearest_distance_mm',near,'min_positive_clearance_mm',pos); end
function tf=wireName(n), tf=contains(lower(char(n)),'weldinggunwire')||contains(lower(char(n)),'wirecollision'); end
function tf=gunName(n), x=lower(char(n)); tf=contains(x,'weldinggunmount')||contains(x,'guncollision')||contains(x,'weldtcp'); end
function tf=skippedPair(pairs,a,b), tf=false; for k=1:size(pairs,1), if isequal(sort(string(pairs(k,:))),sort(string({char(a),char(b)}))), tf=true; return; end, end, end
function d=nearest(scene,q), a=audit(scene,q,scene.robot.BodyNames,{scene.workpieceProxyInfo.name}); d=a.nearest_distance_mm; end
function h=findRec(a,s), h=0; for k=1:numel(a), if max(abs(a(k).q_rad-s.q_rad))<1e-4&&abs(a(k).travel_deg-s.travel_deg)<1e-7&&abs(a(k).roll_deg-s.roll_deg)<1e-7, h=k; return; end, end, end
function p=uniquePoseList(p,n,c), p(:,2)=mod(p(:,2),360); keep=true(size(p,1),1); for i=1:size(p,1), for j=i+1:size(p,1), if keep(j)&&abs(p(i,1)-p(j,1))<1e-9&&angleDist(p(i,2),p(j,2))<1e-9, keep(j)=false; end, end, end; p=p(keep,:); sc=abs(p(:,1)-c(1))+arrayfun(@(x)angleDist(x,c(2)),p(:,2)); [~,ix]=sort(sc); p=p(ix(1:min(n,numel(ix))),:); end
function d=angleDist(a,b), d=abs(atan2d(sind(a-b),cosd(a-b))); end
function x=wrapr(x), x=atan2(sin(x),cos(x)); end
function p=structuralSkips(robot), n=robot.BodyNames; p=cell(0,2); for i=1:min(10,numel(n)-1), p(end+1,:)={n{i},n{i+1}}; end; p(end+1,:)={'WeldingGunMount','wrist_3_link'}; p(end+1,:)={'WeldingGunMount','ee_link'}; p(end+1,:)={'WeldingGunMount','WeldTCP'}; p=p(all(ismember(p,n),2),:); end
function c=counts0(), c=struct('point_index',20,'previous_point_index',19,'ik_calls',0,'ik_numeric_success',0,'ik_pose_success',0,'direction_rejected',0,'side_position_rejected',0,'joint_limit_rejected',0,'position_rejected',0,'orientation_rejected',0,'workpiece_collision_rejected',0,'gun_collision_rejected',0,'wire_collision_rejected',0,'gun_pair_collision_candidates',0,'self_collision_rejected',0,'robot_self_collision_rejected',0,'continuity_rejected',0,'joint_step_continuity_rejected',0,'pose_step_continuity_rejected',0,'edge_collision_rejected',0,'wire_collision_edge_rejected',0,'gun_collision_edge_rejected',0,'workpiece_collision_edge_rejected',0,'node_accepted',0,'connected_candidates',0,'final_status',''); end
function r=record0(), r=struct('rank',NaN,'pose_index',NaN,'travel_deg',NaN,'roll_deg',NaN,'seed_index',NaN,'q_rad',zeros(1,6),'q_deg',zeros(1,6),'tcp_pose',eye(4),'position_error_mm',NaN,'orientation_error_deg',NaN,'axis_angle_deg',NaN,'initial_rejection','','node_filter_pass',false,'parent_edge_pass',false,'continuity_rejection',false,'parent_rejection_reason','','edge_collision_pairs',{{}},'edge_nearest_distance_mm',NaN,'joint_delta_from_point19_deg',zeros(1,6),'joint_distance_from_point19_rad',NaN,'parent_joint_delta_deg',zeros(1,6),'environment_collision',false,'self_collision',false,'gun_collision',false,'wire_collision',false,'robot_self_collision',false,'collision_pairs',{{}},'all_pairs',{{}},'nearest_distance_mm',NaN,'min_positive_clearance_mm',NaN,'penetration_depth_mm',NaN,'penetration_depth_status','UNAVAILABLE_CHECKCOLLISION_API'); end
function r=stateRecord0(), r=struct('pose_index',NaN,'travel_deg',NaN,'roll_deg',NaN,'q_rad',zeros(1,6),'q_deg',zeros(1,6),'tcp_pose',eye(4),'joint_delta_from_point19_deg',zeros(1,6),'joint_distance_from_point19_rad',NaN,'parent_joint_delta_deg',zeros(1,6),'node_filter_pass',false,'parent_edge_pass',false,'continuity_rejection',false,'parent_rejection_reason','','edge_collision_pairs',{{}},'edge_nearest_distance_mm',NaN,'nearest_distance_mm',NaN); end
function writeJson(p,x), f=fopen(p,'w'); if f<0,error('Cannot write %s',p);end; cu=onCleanup(@()fclose(f)); fwrite(f,jsonencode(x),'char'); end
function writeCounts(p,c), f=fopen(p,'w'); cu=onCleanup(@()fclose(f)); fprintf(f,'metric,value\n'); n=fieldnames(c); for i=1:numel(n), v=c.(n{i}); if isnumeric(v)&&isscalar(v), fprintf(f,'%s,%.12g\n',n{i},v); end, end; end
function writeTopCsv(p,a), f=fopen(p,'w'); cu=onCleanup(@()fclose(f)); fprintf(f,'rank,pose_index,travel_deg,roll_deg,J1_deg,J2_deg,J3_deg,J4_deg,J5_deg,J6_deg,pair_count,nearest_distance_mm,joint_distance_from_point19_rad,parent_edge_pass,collision_pairs\n'); for i=1:numel(a), labs=''; if ~isempty(a(i).collision_pairs), labs=strjoin(a(i).collision_pairs,'; '); end; fprintf(f,'%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.9f,%.9f,%d,"%s"\n',i,a(i).pose_index,a(i).travel_deg,a(i).roll_deg,a(i).q_deg,numel(a(i).collision_pairs),a(i).nearest_distance_mm,a(i).joint_distance_from_point19_rad,a(i).parent_edge_pass,labs); end; end
