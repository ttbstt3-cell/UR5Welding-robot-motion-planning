function [Q,report]=hmi_checked_free_path_branch(scene,raw,start,goal)
% Interface adapter, not an RRT change. Numeric endpoints are pinned.
% A periodic lift is only a proposal; neither modulo nor shortest-angle
% equivalence is a permission to cross limits or skip physical edge checks.
start=reshape(double(start),1,6); goal=reshape(double(goal),1,6);
raw=double(raw); Q=[];
report=struct('pass',false,'executionReady',false,'attempts',{{}},...
    'requested_start_rad',start,'requested_goal_rad',goal,'selected_representation','',...
    'failure_reason','No limit-valid, collision-checked numeric branch connects the requested endpoints');
lifted=lift_continuous_joint_path(raw);
candidates={lifted,raw}; names={'PLANNER_PERIODIC_EDGE_LIFT','RAW_NUMERIC_PHYSICAL_EDGES'};
for k=1:2
    C=candidates{k}; lim=scene.jointLimits;
    item=struct('representation',names{k},'q_first_rad',C(1,:),'q_last_rad',C(end,:),...
        'pinned_start_pass',max(abs(C(1,:)-start))<1e-9,...
        'pinned_goal_pass',max(abs(C(end,:)-goal))<1e-9,...
        'joint_limits_pass',all(C>=lim(:,1).' & C<=lim(:,2).','all'),'pass',false);
    if item.pinned_start_pass && item.pinned_goal_pass && item.joint_limits_pass
        item.physical_audit=hmi_audit_joint_interface_path(scene,C,101);
        item.pass=item.physical_audit.pass;
    end
    report.attempts{end+1}=item;
    if ~item.pass, continue; end
    % Densification represents the ACTUAL checked edge, including any long
    % rotation. It never replaces that edge with a wrapped/shorter one.
    Q=C(1,:);
    for e=1:size(C,1)-1
        n=max(1,ceil(max(abs(C(e+1,:)-C(e,:)))/deg2rad(3)));
        for j=1:n, Q(end+1,:)=C(e,:)+(j/n)*(C(e+1,:)-C(e,:)); end %#ok<AGROW>
    end
    assert(max(abs(Q(1,:)-start))<1e-9 && max(abs(Q(end,:)-goal))<1e-9);
    report.pass=true; report.selected_representation=names{k}; report.failure_reason='';
    report.original_path_node_poses_preserved=true;
    report.original_numeric_nodes_preserved=isequal(C,raw); report.output_node_count=size(Q,1);
    report.actual_long_rotation_revalidated=item.physical_audit.maximum_numeric_joint_edge_deg>180;
    report.physical_audit=item.physical_audit; return;
end
end
