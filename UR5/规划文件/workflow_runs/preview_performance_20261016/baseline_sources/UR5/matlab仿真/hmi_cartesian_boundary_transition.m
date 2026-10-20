function [segment,report]=hmi_cartesian_boundary_transition(a,b,scene,budget)
% Bounded local IK at Cartesian/SLERP targets; original endpoint Q is fixed.
report=struct('pass',false,'ik_calls',0,'added_nodes',0,'failure_reason','',...
    'edge_samples',0,'min_positive_clearance_mm',NaN); segment=[]; timer=tic;
ik=inverseKinematics('RigidBodyTree',scene.robot);
for depth=0:budget.maxBoundaryRefinementDepth
    fractions=(1:2^(depth+2)-1)/2^(depth+2); Q=a.qPath(end,:); P={a.tcpPosePath{end}}; okay=true;
    for u=fractions
        target=interpolate_tcp_pose_slerp(a.tcpPosePath{end},b.tcpPosePath{1},u); found=false;
        seeds=[Q(end,:);a.qPath(end,:);b.qPath(1,:)];
        for k=1:size(seeds,1)
            if report.ik_calls>=budget.maxBoundaryIKCalls || toc(timer)>budget.maxBoundaryTimeSec
                report.failure_reason='Cartesian transition budget exhausted'; report.elapsed_sec=toc(timer); return;
            end
            report.ik_calls=report.ik_calls+1;
            [q,~]=ik(scene.tcpBodyName,target,[1 1 1 1 1 1],seeds(k,:));
            q=q+2*pi*round((Q(end,:)-q)/(2*pi));
            T=getTransform(scene.robot,q,scene.tcpBodyName);
            angle=rad2deg(acos(max(-1,min(1,(trace(T(1:3,1:3)'*target(1:3,1:3))-1)/2))));
            if norm(T(1:3,4)-target(1:3,4))*1000>=0.5 || angle>=0.5, continue; end
            if max(abs(q-Q(end,:)))>deg2rad(15), continue; end
            [free,checks,gap]=edgeFree(scene,Q(end,:),q); report.edge_samples=report.edge_samples+checks;
            if free
                found=true; Q(end+1,:)=q; P{end+1}=target; %#ok<AGROW>
                report.min_positive_clearance_mm=minFinite(report.min_positive_clearance_mm,gap); break;
            end
        end
        if ~found, okay=false; report.failure_reason='No constrained collision-free local IK branch'; break; end
    end
    if okay
        q=b.qPath(1,:);
        [free,checks,gap]=edgeFree(scene,Q(end,:),q); report.edge_samples=report.edge_samples+checks;
        if free && max(abs(q-Q(end,:)))<=deg2rad(15)
            Q(end+1,:)=q; P{end+1}=b.tcpPosePath{1}; %#ok<AGROW>
            [t,~]=parameterize_joint_path(Q,'MaxVelocityDegPerSec',5,'MaxAccelerationDegPerSec2',15);
            segment=struct('label',a.label,'subtype','Cartesian transition','qPath',Q,...
                'tcpPosePath',{P},'timeFromStart',t,'velocities',zeros(size(Q)));
            report.pass=true; report.added_nodes=size(Q,1)-2; report.failure_reason='';
            report.min_positive_clearance_mm=minFinite(report.min_positive_clearance_mm,gap); break;
        end
        report.failure_reason='Local IK exists but final endpoint joint step/edge collision failed';
    end
end
report.elapsed_sec=toc(timer);
end
function [free,checks,gap]=edgeFree(scene,qa,qb)
free=true; checks=0; gap=NaN;
for u=linspace(0,1,101)
    [ok,g]=hmi_workflow_state_free(scene,(1-u)*qa+u*qb); checks=checks+1;
    gap=minFinite(gap,g); if ~ok, free=false; return; end
end
end
function x=minFinite(a,b)
if ~isfinite(a), x=b; elseif ~isfinite(b), x=a; else, x=min(a,b); end
end
