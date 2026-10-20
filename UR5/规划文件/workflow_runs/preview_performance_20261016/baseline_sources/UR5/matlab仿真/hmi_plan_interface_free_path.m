function [Q,p]=hmi_plan_interface_free_path(scene,start,goal,r,out,label)
% Bounded endpoint-branch recovery outside the unchanged free-space planner.
% Replanning uses the exact ACCP endpoint and the same limits/constraints.
Q=[]; best=[]; trials={}; bestCost=Inf;
for attempt=1:3
    cfg=struct('scene',scene,'start',start,'goal',goal,'seed',r.seed+attempt-1,...
        'maxIterations',1000000,'maxPlanningTimeSec',r.maxPlanningTimeSec,...
        'maxJointStepRad',deg2rad(12),'jointEdgeResolutionRad',deg2rad(2),...
        'goalBias',0.2,'anytime',true,'continueAfterFirstSolution',true,...
        'checkpointTimesSec',[5 10 15 20 30]);
    switch char(r.algorithm)
        case 'RRT', candidate=RRTJoint(cfg);
        case 'RRT-Connect', candidate=RRTConnectJoint(cfg);
        case 'Bi-RRT*', candidate=BiRRTStarJoint(cfg);
        otherwise, candidate=FastConnectAnytimeRRTStarJoint(cfg);
    end
    branch=struct('pass',false,'failure_reason','PLANNER_FAILED'); C=[];
    if candidate.success && ~isempty(candidate.path)
        [C,branch]=hmi_checked_free_path_branch(scene,candidate.path,start,goal);
    end
    trials{end+1}=struct('attempt',attempt,'seed',cfg.seed,'branch',branch); %#ok<AGROW>
    save(fullfile(out,sprintf('%s_planner_attempt_%d.mat',label,attempt)),...
        'candidate','branch','-v7');
    if branch.pass
        cost=sum(vecnorm(diff(C),2,2));
        if cost<bestCost, best=candidate; Q=C; bestCost=cost; best.interfaceBranch=branch; end
        % Prefer a freshly checked branch without an extra full sweep. A long
        % physical sweep is not made short by modulo: it stays a separately
        % collision-checked fallback if bounded replanning finds nothing better.
        if ~branch.actual_long_rotation_revalidated, break; end
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
