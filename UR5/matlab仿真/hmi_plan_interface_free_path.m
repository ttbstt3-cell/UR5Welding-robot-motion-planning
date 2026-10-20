function [Q,p]=hmi_plan_interface_free_path(scene,start,goal,r,out,label)
% Bounded endpoint-branch recovery outside the unchanged free-space planner.
% Replanning uses the exact ACCP endpoint and the same limits/constraints.
Q=[]; best=[]; trials={}; bestCost=Inf;
fastPreview=isfield(scene,'previewFirstSolution') && scene.previewFirstSolution;
% A first periodic planner path still needs the unchanged physical numeric
% edge audit. If all first-solution trials fail it, retain the historical
% anytime sweep as bounded recovery. A failed branch is never accepted.
maxAttempts=3; if fastPreview, maxAttempts=6; end
for attempt=1:maxAttempts
    seedAttempt=attempt;
    if attempt>3, seedAttempt=attempt-3; end
    cfg=struct('scene',scene,'start',start,'goal',goal,'seed',r.seed+seedAttempt-1,...
        'maxIterations',1000000,'maxPlanningTimeSec',r.maxPlanningTimeSec,...
        'maxJointStepRad',deg2rad(12),'jointEdgeResolutionRad',deg2rad(2),...
        'goalBias',0.2,'anytime',true,'continueAfterFirstSolution',true,...
        'checkpointTimesSec',[5 10 15 20 30]);
    cfg.stopAfterFirstSolution=fastPreview && attempt<=3;
    plannerClock=tic;
    switch char(r.algorithm)
        case 'RRT', candidate=RRTJoint(cfg);
        case 'RRT-Connect', candidate=RRTConnectJoint(cfg);
        case 'Bi-RRT*', candidate=BiRRTStarJoint(cfg);
        otherwise, candidate=FastConnectAnytimeRRTStarJoint(cfg);
    end
    plannerWall=toc(plannerClock); physicalClock=tic;
    branch=struct('pass',false,'failure_reason','PLANNER_FAILED'); C=[];
    if candidate.success && ~isempty(candidate.path)
        [C,branch]=hmi_checked_free_path_branch(scene,candidate.path,start,goal);
    end
    physicalWall=toc(physicalClock);
    trials{end+1}=struct('attempt',attempt,'seed',cfg.seed,'branch',branch,...
        'first_solution_only',cfg.stopAfterFirstSolution,'planner_sec',plannerWall,...
        'physical_branch_audit_sec',physicalWall,'planner_success',candidate.success,...
        'termination_reason',candidate.terminationReason); %#ok<AGROW>
    save(fullfile(out,sprintf('%s_planner_attempt_%d.mat',label,attempt)),...
        'candidate','branch','-v7');
    if branch.pass
        cost=sum(vecnorm(diff(C),2,2));
        if cost<bestCost, best=candidate; Q=C; bestCost=cost; best.interfaceBranch=branch; end
        % Prefer a freshly checked branch without an extra full sweep. A long
        % physical sweep is not made short by modulo: it stays a separately
        % collision-checked fallback if bounded replanning finds nothing better.
        if fastPreview || ~branch.actual_long_rotation_revalidated, break; end
    end
end
if isempty(best)
    p=candidate; p.interfaceTrials=trials;
    save(fullfile(out,[label '_planner.mat']),'p','-v7');
    error('PREVIEW:InterfaceBranch','%s has no endpoint-pinned, limit-valid physical path; last planner: %s; inspect planner attempt audits',label,candidate.terminationReason);
end
p=best; p.interfaceTrials=trials; p.interfaceActualJointLengthRad=bestCost;
save(fullfile(out,[label '_planner.mat']),'p','-v7');
end
