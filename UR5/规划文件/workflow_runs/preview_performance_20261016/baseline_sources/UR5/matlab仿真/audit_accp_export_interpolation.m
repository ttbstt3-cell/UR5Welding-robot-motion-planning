function report=audit_accp_export_interpolation(inputFile,coreFile,outputFile)
% Offline FK audit of the existing exporter velocity rule (no ROS calls).
S=load(inputFile,'precision');P=S.precision;C=load(coreFile,'result');scene=C.result.scene;
Q=P.qPathExecution;[times,~]=parameterize_joint_path(Q);
V=[zeros(1,6);diff(Q)./diff(times)];
maxTimeError=0;maxGeometricError=0;worstEdge=0;collisionPass=true;collisionChecks=0;
limits=zeros(6,2);j=0;
for i=1:numel(scene.robot.Bodies)
    jt=scene.robot.Bodies{i}.Joint;
    if strcmp(jt.Type,'revolute'),j=j+1;limits(j,:)=jt.PositionLimits;end
end
limitsPass=true;
for e=1:size(Q,1)-1
    h=times(e+1)-times(e);
    for u=linspace(0,1,101)
        % Cubic Hermite, positions + velocities, no accelerations exported.
        q=(2*u^3-3*u^2+1)*Q(e,:)+(u^3-2*u^2+u)*h*V(e,:)+ ...
            (-2*u^3+3*u^2)*Q(e+1,:)+(u^3-u^2)*h*V(e+1,:);
        T=getTransform(scene.robot,q,scene.tcpBodyName);
        goal=interpolate_tcp_pose_slerp(P.tcpTargetExecution{e},P.tcpTargetExecution{e+1},u);
        err=1000*norm(T(1:3,4)-goal(1:3,4));
        if err>maxTimeError,maxTimeError=err;worstEdge=e;end
        a=P.tcpTargetExecution{e}(1:3,4);b=P.tcpTargetExecution{e+1}(1:3,4);
        v=b-a;s=max(0,min(1,dot(T(1:3,4)-a,v)/dot(v,v)));
        maxGeometricError=max(maxGeometricError,1000*norm(T(1:3,4)-(a+s*v)));
        limitsPass=limitsPass && all(q>=limits(:,1).'-1e-8) && all(q<=limits(:,2).'+1e-8);
        if abs(u*20-round(u*20))<1e-8
            env=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
            self=checkCollision(scene.robot,q,'IgnoreSelfCollision','off', ...
                'SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
            collisionPass=collisionPass && ~any(env(:)) && ~any(self(:));
            collisionChecks=collisionChecks+1;
        end
    end
end
report=struct('exporter_velocity_rule','[zeros(1,6); diff(qPath)./diff(timeFromStart)]', ...
    'interpolation','joint-space cubic Hermite (positions+velocities)', ...
    'sample_count_per_edge',101,'max_same_time_tcp_error_mm',maxTimeError, ...
    'max_geometric_chord_deviation_mm',maxGeometricError,'worst_edge',worstEdge, ...
    'sampled_cubic_collision_pass',collisionPass,'collision_sample_count',collisionChecks, ...
    'sampled_cubic_joint_limits_pass',limitsPass, ...
    'note','Time-phase error differs from geometric path deviation; no robot execution tested', ...
    'robot_motion','NO');
fid=fopen(outputFile,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));disp(report);
end
