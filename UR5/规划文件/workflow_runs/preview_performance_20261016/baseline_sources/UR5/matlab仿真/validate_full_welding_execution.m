function report = validate_full_welding_execution(approach,weld,exit,scene,expectedBinding,options)
%VALIDATE_FULL_WELDING_EXECUTION Offline validation of approach+weld+exit.
% This function is diagnostic only. It does not plan, repair, export, or send
% a robot trajectory. Segment structs require qPath (N-by-6), tcpPosePath
% (cell N of 4-by-4 transforms or 4-by-4-by-N), timeFromStart, velocities,
% and binding. Required binding keys are listed in localRequiredBindings.
if nargin<6 || isempty(options), options=struct(); end
if nargin<5 || isempty(expectedBinding), expectedBinding=struct(); end
cfg=struct('minControllerSamples',5000,'maxTcpContourErrorMm',0.5,...
    'maxTcpTimeSyncErrorMm',0.5,'maxTcpOrientationErrorDeg',0.5,...
    'maxVelocityRadPerSec',deg2rad(5),'maxAccelerationRadPerSec2',0.261799,...
    'jointBoundaryToleranceRad',1e-6,'tcpBoundaryToleranceMm',0.01,...
    'orientationBoundaryToleranceDeg',0.01,'velocityBoundaryToleranceRadPerSec',1e-8,...
    'accelerationBoundaryToleranceRadPerSec2',1e-6,'tcpBodyName','WeldTCP');
for k=fieldnames(cfg).'
    if isfield(options,k{1}) && ~isempty(options.(k{1})), cfg.(k{1})=options.(k{1}); end
end

segments={localNormalize(approach,'CBi-RRT* approach'),...
    localNormalize(weld,'ACCP-D weld'),localNormalize(exit,'CBi-RRT* exit')};
report=struct('schema_version','FULL_WELDING_EXECUTION_VALIDATION_v1',...
    'robot_motion','NO','offline_only',true,'trajectory_exported',false,...
    'executionReady',false,'required_controller_samples',cfg.minControllerSamples,...
    'controller_interpolation','positions+velocities cubic Hermite (ROS 2 joint_trajectory_controller)',...
    'hardware_inner_servo_verified',false,'continuous_collision_proof',false,...
    'wire_collision_shorten_mm',3,'wire_collision_shorten_physical_safety_verified',false,...
    'segments',{{segments{1}.summary,segments{2}.summary,segments{3}.summary}},...
    'version_consistency',localVersionCheck(segments,expectedBinding),...
    'splice_checks',{{}},'controller_audit',[],'classification',{{}},...
    'recommendations',{{}},'metrics',struct(),'validation_status','BLOCKED');

if ~all(cellfun(@(s)s.available,segments))
    report.classification=localUnique([report.classification,{'MISSING_CURRENT_SEGMENT_PATH',...
        'FULL_CONTROLLER_INTERPOLATION_NOT_RUN','FULL_TASK_COLLISION_NOT_RUN'}]);
    report.recommendations{end+1}='重新生成并保存当前REAL_CALIBRATION版本的CBi-RRT* RealHome→PreWeld接近段及PostWeld→RealHome退出段；保存qPath、tcpPosePath、timeFromStart、velocities和完整版本绑定。';
end
if ~report.version_consistency.pass
    report.classification=localUnique([report.classification,{'STALE_OR_INCOMPLETE_VERSION_BINDING'}]);
    report.recommendations{end+1}='为TCP、URDF、焊枪、焊丝、工件位姿和碰撞场景写入可核验版本标识；三段任一不一致时必须重规划，不能拼接。';
end

if all(cellfun(@(s)s.available,segments))
    report.splice_checks={localBoundary(segments{1},segments{2},'approach_to_weld',cfg),...
        localBoundary(segments{2},segments{3},'weld_to_exit',cfg)};
    for i=1:numel(report.splice_checks)
        b=report.splice_checks{i};
        if ~b.position_continuity_pass, report.classification=localUnique([report.classification,{'POSITION_DISCONTINUITY'}]); end
        if ~b.orientation_continuity_pass, report.classification=localUnique([report.classification,{'ORIENTATION_DISCONTINUITY'}]); end
        if ~b.velocity_continuity_pass, report.classification=localUnique([report.classification,{'VELOCITY_DISCONTINUITY'}]); end
        if ~b.acceleration_continuity_pass, report.classification=localUnique([report.classification,{'ACCELERATION_DISCONTINUITY'}]); end
    end
else
    report.splice_checks={localUnverifiedBoundary('approach_to_weld'),localUnverifiedBoundary('weld_to_exit')};
    report.classification=localUnique([report.classification,{'POSITION_ORIENTATION_VELOCITY_ACCELERATION_NOT_VERIFIABLE'}]);
end

allBindingsValid=report.version_consistency.pass;
allSplicesValid=all(cellfun(@(b)b.pass,report.splice_checks));
if all(cellfun(@(s)s.available,segments)) && allBindingsValid && allSplicesValid
    [fullPath,mergeError]=localMerge(segments,cfg);
    if isempty(mergeError)
        n=size(fullPath.qPath,1);
        samplesPerSegment=ceil((cfg.minControllerSamples-1)/(n-1))+1;
        opts=struct('samplesPerSegment',samplesPerSegment,...
            'maxContourErrorMm',cfg.maxTcpContourErrorMm,...
            'maxTimeSyncErrorMm',cfg.maxTcpTimeSyncErrorMm,...
            'maxOrientationErrorDeg',cfg.maxTcpOrientationErrorDeg,...
            'maxVelocityRadPerSec',cfg.maxVelocityRadPerSec,...
            'maxAccelerationRadPerSec2',cfg.maxAccelerationRadPerSec2);
        opts.tcpBodyName=cfg.tcpBodyName;
        audit=validate_accp_controller_interpolation(fullPath.qPath,fullPath.tcpPosePath,...
            fullPath.timeFromStart,fullPath.velocities,scene,opts);
        derivative=localHermiteDerivatives(fullPath.qPath,fullPath.timeFromStart,fullPath.velocities);
        audit.max_jerk_rad_per_sec3=derivative.max_jerk_rad_per_sec3;
        audit.rms_jerk_rad_per_sec3=derivative.rms_jerk_rad_per_sec3;
        audit.max_node_acceleration_jump_rad_per_sec2=derivative.max_node_acceleration_jump_rad_per_sec2;
        audit.rms_node_acceleration_jump_rad_per_sec2=derivative.rms_node_acceleration_jump_rad_per_sec2;
        audit.node_acceleration_jumps_rad_per_sec2=derivative.node_acceleration_jumps_rad_per_sec2;
        audit.sample_count_at_least_5000=audit.sample_count>=cfg.minControllerSamples;
        report.controller_audit=audit;
        report.metrics=struct('duration_sec',fullPath.timeFromStart(end),...
            'waypoint_count',n,'controller_sample_count',audit.sample_count,...
            'max_tcp_contour_error_mm',audit.max_tcp_contour_error_mm,...
            'rms_tcp_contour_error_mm',audit.rms_tcp_contour_error_mm,...
            'max_tcp_time_sync_error_mm',audit.max_tcp_time_sync_error_mm,...
            'rms_tcp_time_sync_error_mm',audit.rms_tcp_time_sync_error_mm,...
            'max_tcp_orientation_error_deg',audit.max_tcp_orientation_error_deg,...
            'max_velocity_rad_per_sec',audit.max_joint_velocity_rad_per_sec,...
            'max_acceleration_rad_per_sec2',audit.max_joint_acceleration_rad_per_sec2,...
            'max_jerk_rad_per_sec3',derivative.max_jerk_rad_per_sec3,...
            'rms_jerk_rad_per_sec3',derivative.rms_jerk_rad_per_sec3,...
            'max_node_acceleration_jump_rad_per_sec2',derivative.max_node_acceleration_jump_rad_per_sec2,...
            'rms_node_acceleration_jump_rad_per_sec2',derivative.rms_node_acceleration_jump_rad_per_sec2,...
            'collision_pass',audit.collision_pass,'joint_limits_pass',audit.joint_limits_pass,...
            'minimum_sampled_clearance_mm',audit.min_positive_clearance_mm);
        if ~audit.collision_pass, report.classification=localUnique([report.classification,{'COLLISION'}]); end
        if ~audit.complete, report.classification=localUnique([report.classification,{'CONTROLLER_INTERPOLATION_ERROR_OR_LIMIT'}]); end
        report.executionReady=logical(audit.complete && audit.sample_count_at_least_5000 &&...
            audit.collision_pass && audit.joint_limits_pass && allBindingsValid && allSplicesValid);
        if report.executionReady, report.validation_status='PASS_OFFLINE_ONLY';
        else, report.validation_status='FAILED_OFFLINE'; end
    else
        report.classification=localUnique([report.classification,{mergeError}]);
        report.recommendations{end+1}='不自动修复拼接；先检查边界速度/加速度，再单独设计并验证过渡段或边界速度重优化。';
    end
else
    if all(cellfun(@(s)s.available,segments)) && allBindingsValid && ~allSplicesValid
        report.recommendations{end+1}='按失败边界分别处理：位置/姿态不连续时评估PreWeld或PostWeld笛卡尔过渡点；速度/加速度不连续时评估边界速度过渡或重优化。当前模块不会自动修改轨迹。';
    end
    report.recommendations{end+1}='完整任务未拼接，因此没有声称满足5000点整程验证；ACCP-D已有单段结果仅作为独立段证据。';
end
end

function s=localNormalize(raw,name)
s=struct('name',name,'available',false,'qPath',[],'tcpPosePath',[],...
    'timeFromStart',[],'velocities',[],'binding',struct(),'summary',struct());
if isempty(raw) || ~isstruct(raw), s.summary=struct('name',name,'status','MISSING_PATH'); return; end
if isfield(raw,'available') && ~raw.available
    s.summary=struct('name',name,'status','MISSING_PATH','reason',localField(raw,'reason','not supplied')); return;
end
need={'qPath','tcpPosePath','timeFromStart'};
if ~all(isfield(raw,need)), s.summary=struct('name',name,'status','MISSING_REQUIRED_FIELDS'); return; end
Q=double(raw.qPath); t=double(raw.timeFromStart(:)); n=size(Q,1);
if size(Q,2)~=6 || n<2 || any(~isfinite(Q(:))) || numel(t)~=n || any(~isfinite(t)) || any(diff(t)<=0)
    s.summary=struct('name',name,'status','INVALID_PATH_SHAPE_OR_TIME'); return;
end
try
    localPoseArray(raw.tcpPosePath,n);
catch e
    s.summary=struct('name',name,'status','INVALID_TCP_POSE_PATH','reason',e.message); return;
end
s.available=true; s.qPath=Q; s.tcpPosePath=raw.tcpPosePath; s.timeFromStart=t;
if isfield(raw,'velocities') && ~isempty(raw.velocities)
    V=double(raw.velocities); if isequal(size(V),size(Q)) && all(isfinite(V(:))), s.velocities=V; end
end
if isfield(raw,'binding') && isstruct(raw.binding), s.binding=raw.binding; end
s.summary=struct('name',name,'status','AVAILABLE','waypoint_count',n,'duration_sec',t(end),...
    'velocity_present',~isempty(s.velocities),'binding',s.binding);
end

function v=localVersionCheck(segments,expected)
fields=localRequiredBindings(); checks=cell(1,numel(fields)); allPass=true;
for k=1:numel(fields)
    key=fields{k}; vals=cell(1,3); present=false(1,3);
    for j=1:3
        if segments{j}.available && isfield(segments{j}.binding,key) && ...
                ~isempty(segments{j}.binding.(key))
            vals{j}=segments{j}.binding.(key); present(j)=true;
        else
            vals{j}='';
        end
    end
    expectedValue=''; expectedPresent=isfield(expected,key) && ~isempty(expected.(key));
    if expectedPresent, expectedValue=expected.(key); end
    valid=all(present) && isequal(vals{1},vals{2}) && isequal(vals{2},vals{3});
    if expectedPresent, valid=valid && all(cellfun(@(x)isequal(x,expectedValue),vals)); end
    checks{k}=struct('version_key',key,'approach',vals{1},'weld',vals{2},'exit',vals{3},...
        'expected',expectedValue,'present',present,'status',localStatus(valid,all(present),expectedPresent));
    allPass=allPass && valid;
end
v=struct('pass',logical(allPass),'required_keys',{fields},'checks',{checks},...
    'note','Missing version values are not inferred from filenames or timestamps.');
end

function b=localBoundary(a,c,name,cfg)
b=localUnverifiedBoundary(name); b.evaluated=true;
qa=a.qPath(end,:); qb=c.qPath(1,:); dq=qb-qa;
b.joint_delta_rad=dq; b.max_abs_joint_delta_rad=max(abs(dq)); b.joint_delta_norm_rad=norm(dq);
b.position_continuity_pass=b.max_abs_joint_delta_rad<=cfg.jointBoundaryToleranceRad;
[Pa,Ra]=localPoseAt(a.tcpPosePath,size(a.qPath,1),size(a.qPath,1));
[Pc,Rc]=localPoseAt(c.tcpPosePath,size(c.qPath,1),1);
b.tcp_position_delta_mm=1000*norm(Pc-Pa); b.tcp_orientation_delta_deg=localRotationDistance(Ra,Rc);
b.tcp_position_continuity_pass=b.tcp_position_delta_mm<=cfg.tcpBoundaryToleranceMm;
b.orientation_continuity_pass=b.tcp_orientation_delta_deg<=cfg.orientationBoundaryToleranceDeg;
b.position_continuity_pass=b.position_continuity_pass && b.tcp_position_continuity_pass;
b.velocity_delta_rad_per_sec=[]; b.velocity_continuity_pass=false;
b.acceleration_delta_rad_per_sec2=[]; b.acceleration_continuity_pass=false;
if ~isempty(a.velocities) && ~isempty(c.velocities)
    dv=c.velocities(1,:)-a.velocities(end,:); b.velocity_delta_rad_per_sec=dv;
    b.max_abs_velocity_delta_rad_per_sec=max(abs(dv));
    b.velocity_continuity_pass=b.max_abs_velocity_delta_rad_per_sec<=cfg.velocityBoundaryToleranceRadPerSec;
    aa=localEndpointAcceleration(a,'end'); ac=localEndpointAcceleration(c,'start'); da=ac-aa;
    b.acceleration_delta_rad_per_sec2=da; b.max_abs_acceleration_delta_rad_per_sec2=max(abs(da));
    b.acceleration_continuity_pass=b.max_abs_acceleration_delta_rad_per_sec2<=cfg.accelerationBoundaryToleranceRadPerSec2;
end
b.pass=logical(b.position_continuity_pass && b.orientation_continuity_pass &&...
    b.velocity_continuity_pass && b.acceleration_continuity_pass);
b.status=localStatus(b.pass,true,true);
end

function b=localUnverifiedBoundary(name)
b=struct('name',name,'evaluated',false,'status','NOT_VERIFIED','pass',false,...
    'joint_delta_rad',[],'max_abs_joint_delta_rad',[],'joint_delta_norm_rad',[],...
    'tcp_position_delta_mm',[],'tcp_orientation_delta_deg',[],...
    'position_continuity_pass',false,'tcp_position_continuity_pass',false,...
    'orientation_continuity_pass',false,'velocity_delta_rad_per_sec',[],...
    'max_abs_velocity_delta_rad_per_sec',[],'velocity_continuity_pass',false,...
    'acceleration_delta_rad_per_sec2',[],'max_abs_acceleration_delta_rad_per_sec2',[],...
    'acceleration_continuity_pass',false);
end

function [fullPath,err]=localMerge(segments,cfg)
err=''; fullPath=struct('qPath',[],'tcpPosePath',{{}},'timeFromStart',[],'velocities',[]);
for k=1:3
    if isempty(segments{k}.velocities), err='MISSING_VELOCITY_PROFILE'; return; end
end
Q=[]; T=[]; V=[]; poses={};
for k=1:3
    s=segments{k};
    if k>1
        if max(abs(Q(end,:)-s.qPath(1,:)))>cfg.jointBoundaryToleranceRad ||...
                max(abs(V(end,:)-s.velocities(1,:)))>cfg.velocityBoundaryToleranceRadPerSec
            err='POSITION_OR_VELOCITY_BOUNDARY_MISMATCH'; return;
        end
        offset=T(end)-s.timeFromStart(1); idx=2:size(s.qPath,1);
    else
        offset=0; idx=1:size(s.qPath,1);
    end
    Q=[Q;s.qPath(idx,:)]; V=[V;s.velocities(idx,:)]; T=[T;s.timeFromStart(idx)+offset]; %#ok<AGROW>
    pp=localPoseArray(s.tcpPosePath,size(s.qPath,1));
    poses=[poses,pp(idx)]; %#ok<AGROW>
end
if any(diff(T)<=0), err='NON_MONOTONIC_MERGED_TIME'; return; end
fullPath=struct('qPath',Q,'tcpPosePath',{poses},'timeFromStart',T,'velocities',V);
end

function d=localHermiteDerivatives(Q,t,V)
h=diff(t); jerk=(12*Q(1:end-1,:)+6*h.*V(1:end-1,:)-...
    12*Q(2:end,:)+6*h.*V(2:end,:))./(h.^3);
acc0=(-6*Q(1:end-1,:)-4*h.*V(1:end-1,:)+6*Q(2:end,:)-2*h.*V(2:end,:))./(h.^2);
acc1=(6*Q(1:end-1,:)+2*h.*V(1:end-1,:)-6*Q(2:end,:)+4*h.*V(2:end,:))./(h.^2);
jump=acc0(2:end,:)-acc1(1:end-1,:);
d=struct('max_jerk_rad_per_sec3',max(abs(jerk(:))),...
    'rms_jerk_rad_per_sec3',sqrt(sum(sum(jerk.^2,2).*h)/(6*sum(h))),...
    'max_node_acceleration_jump_rad_per_sec2',localMaxAbs(jump),...
    'rms_node_acceleration_jump_rad_per_sec2',localRms(jump),...
    'node_acceleration_jumps_rad_per_sec2',jump,'segment_acceleration_start_rad_per_sec2',acc0,...
    'segment_acceleration_end_rad_per_sec2',acc1,'segment_jerk_rad_per_sec3',jerk);
end

function a=localEndpointAcceleration(s,side)
Q=s.qPath; V=s.velocities; h=s.timeFromStart(2)-s.timeFromStart(1);
if strcmp(side,'end')
    h=s.timeFromStart(end)-s.timeFromStart(end-1); q0=Q(end-1,:); q1=Q(end,:); v0=V(end-1,:); v1=V(end,:);
    a=(6*q0+2*h*v0-6*q1+4*h*v1)/(h*h);
else
    q0=Q(1,:); q1=Q(2,:); v0=V(1,:); v1=V(2,:);
    a=(-6*q0-4*h*v0+6*q1-2*h*v1)/(h*h);
end
end

function poses=localPoseArray(raw,n)
if iscell(raw)
    if numel(raw)~=n, error('TCP pose count does not match qPath.'); end
    poses=raw(:).';
elseif ndims(raw)==3 && size(raw,1)==4 && size(raw,2)==4 && size(raw,3)==n
    poses=cell(1,n);
    for i=1:n, poses{i}=raw(:,:,i); end
else
    error('tcpPosePath must be cell N or 4-by-4-by-N.');
end
for i=1:n
    T=double(poses{i});
    if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || norm(T(4,:)-[0 0 0 1])>1e-7
        error('Invalid 4-by-4 TCP transform at index %d.',i);
    end
end
end

function [P,R]=localPoseAt(raw,n,index)
p=localPoseArray(raw,n); T=double(p{index}); P=T(1:3,4).'; R=T(1:3,1:3);
end
function e=localRotationDistance(A,B)
e=rad2deg(acos(max(-1,min(1,(trace(A'*B)-1)/2))));
end
function fields=localRequiredBindings()
fields={'tcp_calibration_version','urdf_version','gun_geometry_version',...
    'wire_model_version','workpiece_pose_version','collision_scene_version'};
end
function s=localStatus(pass,present,expected)
if pass, s='PASS'; elseif ~present, s='MISSING_VERSION'; elseif ~expected, s='MISSING_EXPECTED_BINDING'; else, s='MISMATCH'; end
end
function x=localField(s,k,d), if isfield(s,k), x=s.(k); else, x=d; end, end
function y=localUnique(x), y=unique(x,'stable'); end
function x=localMaxAbs(a), if isempty(a), x=0; else, x=max(abs(a(:))); end, end
function x=localRms(a), if isempty(a), x=0; else, x=sqrt(mean(a(:).^2)); end, end
