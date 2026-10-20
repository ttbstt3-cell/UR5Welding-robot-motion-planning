function result = generate_actual_axis_ik_candidates(scene, Ttarget, travelDeg, rollDeg, qSeeds, options)
%GENERATE_ACTUAL_AXIS_IK_CANDIDATES Multi-IK candidates with hard tool-axis rules.
%
% Every accepted state satisfies: IK pose tolerance, joint limits, physical
% gun-axis/welding-side constraint, robot-vs-workpiece collision-free, and
% robot self-collision-free.  The WeldTCP body already contains the MATLAB
% wire collision cylinder from sync_ur5_joint_states.m, so the wire is part
% of both collision tests here.

arguments
    scene
    Ttarget (4,4) double
    travelDeg (1,1) double
    rollDeg (1,1) double
    qSeeds double
    options struct = struct()
end

defaults = struct( ...
    "positionToleranceM",1e-4, ...
    "orientationToleranceDeg",0.5, ...
    "sideToleranceDeg",5.0, ...
    "axis_angle_tolerance_deg",[], ...
    "sideMarginM",1e-3, ...
    "duplicateToleranceDeg",0.5, ...
    "checkSelfCollision",true, ...
    "lazyCollision",false, ...
    "reuseCandidateCache",false, ...
    "diagnosticCollectIKCandidates",false, ...
    "diagnosticReferenceQ",[], ...
    "diagnosticMaxJointDeltaDeg",inf, ...
    "disableAxisConstraint",false, ...
    "disable_axis_constraint",false, ...
    "verbose",false);
options = localMergeOptions(defaults, options);
if ~isempty(options.axis_angle_tolerance_deg)
    validateattributes(options.axis_angle_tolerance_deg,{'numeric'}, ...
        {'scalar','real','finite','>=',0,'<=',180});
    options.sideToleranceDeg=double(options.axis_angle_tolerance_deg);
end
options.disableAxisConstraint = logical(options.disableAxisConstraint || ...
    options.disable_axis_constraint);

robot = scene.robot;
qSeeds = double(qSeeds);
if isempty(qSeeds)
    qSeeds = scene.qMat0;
end
if size(qSeeds,2) ~= 6
    error("qSeeds 必须是 K×6。 ");
end

if isfield(scene,"ikSolver")
    ik = scene.ikSolver;
else
    ik = inverseKinematics("RigidBodyTree",robot);
end
weights = [1 1 1 1 1 1];
if isfield(scene,"jointLimits")
    limits = scene.jointLimits;
else
    limits = localJointLimits(robot);
end

% Exact pose candidates may be revisited by coarse/fine refinement.  Reuse
% the accepted IK states for the same target within this run.  The cache is
% a handle object stored in scene, so updates persist across function calls.
cacheKey = localCandidateCacheKey(Ttarget,travelDeg,rollDeg,options.disableAxisConstraint,options.sideToleranceDeg);
if options.reuseCandidateCache && isfield(scene,"ikCandidateCache") && ...
        isKey(scene.ikCandidateCache,cacheKey)
    result = scene.ikCandidateCache(cacheKey);
    return;
end

% Build a target-side reference from the same physical gun axis used by the
% pose generator.  The seam is bottom x wall_y1, with free space y<wallY and
% z>bottomZ.  A valid welding approach points from free space into the corner.
gTargetB = Ttarget(1:3,1:3) * scene.wireAxisTcp(:);
RWB = scene.T_MatBase_Workpiece(1:3,1:3).';
gTargetWP = RWB * gTargetB;
gTargetWP = gTargetWP / norm(gTargetWP);
gunAxisWP = localGetField(scene,"weldGunAxisWP",[0;1;-1]);
gunAxisWP = gunAxisWP(:)/norm(gunAxisWP);

qRaw = zeros(0,6);
stateRaw = repmat(localEmptyState(),0,1);
numIKCalls = 0;
numIKNumericSuccess = 0;
numIKPoseOK = 0;
numSideRejected = 0;
numDirectionRejected = 0;
numWeldingSidePositionRejected = 0;
numJointRejected = 0;
numPositionRejected = 0;
numOrientationRejected = 0;
numJointContinuityRejected = 0;
numEnvRejected = 0;
numSelfRejected = 0;
numRobotSelfRejected = 0;
numGunRejected = 0;
diagnosticCandidates = repmat(localEmptyDiagnosticCandidate(),0,1);

diagnosticReferenceQ = localGetField(options,"diagnosticReferenceQ",[]);
if isempty(diagnosticReferenceQ)
    if isfield(scene,"branchRefQ") && ~isempty(scene.branchRefQ)
        diagnosticReferenceQ = double(scene.branchRefQ(:).');
    else
        diagnosticReferenceQ = double(qSeeds(1,:));
    end
else
    diagnosticReferenceQ = double(diagnosticReferenceQ(:).');
end

for k = 1:size(qSeeds,1)
    numIKCalls = numIKCalls + 1;
    try
        [qSol, solInfo] = ik(scene.tcpBodyName,Ttarget,weights,qSeeds(k,:)); %#ok<ASGLU>
    catch
        continue;
    end

    qSol = double(qSol(:).');
    if numel(qSol) ~= 6 || any(~isfinite(qSol))
        continue;
    end
    numIKNumericSuccess = numIKNumericSuccess + 1;

    % Move an IK solution to the equivalent configuration nearest its seed;
    % this avoids artificial 2*pi jumps while preserving the physical pose.
    qSol = qSeeds(k,:) + localWrapRad(qSol-qSeeds(k,:));
    if any(qSol < limits(:,1).' - 1e-8) || any(qSol > limits(:,2).' + 1e-8)
        numJointRejected = numJointRejected + 1;
        continue;
    end

    Tactual = getTransform(robot,qSol,scene.tcpBodyName);
    posErr = norm(Tactual(1:3,4)-Ttarget(1:3,4));
    Rerr = Ttarget(1:3,1:3).' * Tactual(1:3,1:3);
    c = (trace(Rerr)-1)/2;
    oriErr = acos(max(-1,min(1,c)));
    if posErr > options.positionToleranceM
        numPositionRejected = numPositionRejected + 1;
    end
    if rad2deg(oriErr) > options.orientationToleranceDeg
        numOrientationRejected = numOrientationRejected + 1;
    end
    if posErr > options.positionToleranceM || ...
            rad2deg(oriErr) > options.orientationToleranceDeg
        continue;
    end
    numIKPoseOK = numIKPoseOK + 1;

    diagnosticIndex = 0;
    if options.diagnosticCollectIKCandidates
        diagnosticIndex = numel(diagnosticCandidates) + 1;
        dc = localEmptyDiagnosticCandidate();
        dc.q_rad = qSol;
        dc.q_deg = rad2deg(qSol);
        dc.tcp_pose = Tactual;
        dc.position_error_mm = posErr*1000;
        dc.orientation_error_deg = rad2deg(oriErr);
        dc.joint_delta_from_seed_deg = rad2deg(localWrapRad(qSol-qSeeds(k,:)));
        dc.joint_delta_from_reference_deg = rad2deg(localWrapRad(qSol-diagnosticReferenceQ));
        dc.max_joint_delta_deg = max(abs(dc.joint_delta_from_reference_deg));
        dc.travel_deg = travelDeg;
        dc.roll_deg = rollDeg;
        dc.seed_index = k;
        % Point-1 orientation audit.  The mount axis is the fixed, measured
        % tool/wire axis expressed by the TCP frame; it is transformed into
        % the MATLAB planning world without changing the planner's geometry.
        mountAxisWorld = localUnit(Tactual(1:3,1:3)*scene.wireAxisTcp(:));
        targetAxisWorld = localUnit(scene.T_MatBase_Workpiece(1:3,1:3)*gunAxisWP);
        dc.tcp_rotation_matrix = Tactual(1:3,1:3);
        dc.welding_gun_mount_axis_world = mountAxisWorld(:).';
        dc.target_weld_axis_world = targetAxisWorld(:).';
        dc.axis_angle_deg = acosd(max(-1,min(1,dot(mountAxisWorld,targetAxisWorld))));
        if isfield(scene,"weldTangentWP") && ~isempty(scene.weldTangentWP)
            seamTangentWorld = localUnit(scene.T_MatBase_Workpiece(1:3,1:3)*scene.weldTangentWP(:));
            dc.seam_tangent_world = seamTangentWorld(:).';
            dc.axis_vs_seam_angle_deg = acosd(max(-1,min(1,dot(mountAxisWorld,seamTangentWorld))));
        end
        dc.rejection_reason = "accepted";
        diagnosticCandidates(end+1,1) = dc; %#ok<AGROW>
        if dc.max_joint_delta_deg > options.diagnosticMaxJointDeltaDeg
            numJointContinuityRejected = numJointContinuityRejected + 1;
        end
    end

    % Actual gun axis in Workpiece coordinates, independent of TCP +Z.
    gB = Tactual(1:3,1:3) * scene.wireAxisTcp(:);
    gWP = RWB*gB;
    gWP = gWP/norm(gWP);
    sideDot = dot(gWP,gunAxisWP);

    % Hard welding-side check: the gun head must stay in the U interior and
    % the physical gun axis must point into the bottom/wall corner.
    pTcpWP = RWB*(Tactual(1:3,4)-scene.T_MatBase_Workpiece(1:3,4));
    if isfield(scene,"hullHeadDistanceM") && isfinite(scene.hullHeadDistanceM)
        pHeadB = Tactual(1:3,4)-scene.hullHeadDistanceM*gB;
    else
        pHeadB = Tactual(1:3,4)-0.020*gB;
    end
    pHeadWP = RWB*(pHeadB-scene.T_MatBase_Workpiece(1:3,4));
    % This switch is intentionally diagnostic-only.  The formal default is
    % false, so production ACCP keeps the original axis gate unchanged.
    directionOK = options.disableAxisConstraint || sideDot >= cosd(options.sideToleranceDeg);
    headYOK = pHeadWP(2) < 0.230-options.sideMarginM;
    headZOK = pHeadWP(3) > 0.005+options.sideMarginM;
    sideOK = directionOK && headYOK && headZOK;
    if ~directionOK
        numDirectionRejected = numDirectionRejected + 1;
    end
    if ~headYOK || ~headZOK
        numWeldingSidePositionRejected = numWeldingSidePositionRejected + 1;
    end
    if ~sideOK
        numSideRejected = numSideRejected + 1;
        if diagnosticIndex > 0
            if ~directionOK && (~headYOK || ~headZOK)
                diagnosticCandidates(diagnosticIndex).rejection_reason = "orientation_direction_and_side_position";
            elseif ~directionOK
                diagnosticCandidates(diagnosticIndex).rejection_reason = "orientation_direction";
            else
                diagnosticCandidates(diagnosticIndex).rejection_reason = "welding_side_position";
            end
        end
        continue;
    end

    envColliding = false;
    selfColliding = false;
    % Lazy mode defers only interpolated edge checks.  Node collision must
    % still be checked here; otherwise thousands of colliding IK states can
    % enter the layered graph and make it much larger than the baseline.
    envColliding=check_scene_environment_collision(scene,qSol);
    envColliding = any(envColliding(:));
    if envColliding
        numEnvRejected = numEnvRejected + 1;
        if diagnosticIndex > 0
            diagnosticCandidates(diagnosticIndex).rejection_reason = "workpiece_collision";
        end
        continue;
    end

    if options.checkSelfCollision
        skipPairs = localGetField(scene,"selfCollisionSkipPairs","parent");
        try
            selfColliding = checkCollision(robot,qSol, ...
                "IgnoreSelfCollision","off", ...
                "SkippedSelfCollisions",skipPairs, ...
                "Exhaustive","on");
        catch
            selfColliding = checkCollision(robot,qSol, ...
                "IgnoreSelfCollision","off", ...
                "SkippedSelfCollisions",skipPairs);
        end
        selfColliding = any(selfColliding(:));
    end
    if selfColliding
        numSelfRejected = numSelfRejected + 1;
        if options.diagnosticCollectIKCandidates
            [robotSelf,gunSelf] = localClassifySelfCollision(scene,qSol,skipPairs);
            numRobotSelfRejected = numRobotSelfRejected + double(robotSelf);
            numGunRejected = numGunRejected + double(gunSelf);
            if diagnosticIndex > 0
                if robotSelf && gunSelf
                    diagnosticCandidates(diagnosticIndex).rejection_reason = "robot_self_and_gun_collision";
                elseif gunSelf
                    diagnosticCandidates(diagnosticIndex).rejection_reason = "gun_collision";
                else
                    diagnosticCandidates(diagnosticIndex).rejection_reason = "self_collision";
                end
            end
        end
        continue;
    end

    qRaw(end+1,:) = qSol; %#ok<AGROW>
    st = localEmptyState();
    st.q = qSol;
    st.travelDeg = travelDeg;
    st.rollDeg = rollDeg;
    st.Ttarget = Ttarget;
    st.Tactual = Tactual;
    st.positionErrorM = posErr;
    st.orientationErrorDeg = rad2deg(oriErr);
    st.gunAxisWP = gWP(:).';
    st.sideDot = sideDot;
    st.headPositionWP = pHeadWP(:).';
    st.environmentCollision = false;
    st.selfCollision = false;
    stateRaw(end+1,1) = st; %#ok<AGROW>
end

% Deduplicate equivalent configurations while preserving all distinct IK
% branches.  The same q can be returned from several seeds.
keep = true(size(qRaw,1),1);
for i = 1:size(qRaw,1)
    if ~keep(i), continue; end
    for j = i+1:size(qRaw,1)
        if keep(j)
            dq = localWrapRad(qRaw(i,:)-qRaw(j,:));
            if max(abs(rad2deg(dq))) < options.duplicateToleranceDeg
                keep(j) = false;
            end
        end
    end
end

result = struct();
result.states = stateRaw(keep);
if isempty(result.states)
    result.qCandidates = zeros(0,6);
else
    result.qCandidates = vertcat(result.states.q);
end
result.numIKCalls = numIKCalls;
result.numIKNumericSuccess = numIKNumericSuccess;
result.numIKPoseOK = numIKPoseOK;
result.numSideRejected = numSideRejected;
result.numDirectionRejected = numDirectionRejected;
result.numWeldingSidePositionRejected = numWeldingSidePositionRejected;
result.numJointRejected = numJointRejected;
result.numPositionRejected = numPositionRejected;
result.numOrientationRejected = numOrientationRejected;
result.numJointContinuityRejected = numJointContinuityRejected;
result.numEnvironmentRejected = numEnvRejected;
result.numWorkpieceRejected = numEnvRejected;
result.numSelfRejected = numSelfRejected;
result.numRobotSelfRejected = numRobotSelfRejected;
result.numGunRejected = numGunRejected;
result.numAccepted = numel(result.states);
result.diagnosticCandidates = diagnosticCandidates;
result.axisConstraintDisabled = logical(options.disableAxisConstraint);
result.lazyCollision = options.lazyCollision;
result.targetGunAxisWP = gTargetWP(:).';
result.travelDeg = travelDeg;
result.rollDeg = rollDeg;

if options.reuseCandidateCache && isfield(scene,"ikCandidateCache") && ...
        result.numAccepted > 0
    scene.ikCandidateCache(cacheKey) = result;
end

if options.verbose
    fprintf("  travel=%+.1f roll=%+.1f: IK=%d accepted=%d (side %d env %d self %d)\n", ...
        travelDeg,rollDeg,numIKCalls,result.numAccepted, ...
        numSideRejected,numEnvRejected,numSelfRejected);
end
end

function s = localEmptyState()
s = struct("q",zeros(1,6),"travelDeg",0,"rollDeg",0, ...
    "Ttarget",eye(4),"Tactual",eye(4),"positionErrorM",NaN, ...
    "orientationErrorDeg",NaN,"gunAxisWP",zeros(1,3),"sideDot",NaN, ...
    "headPositionWP",zeros(1,3),"environmentCollision",false, ...
    "selfCollision",false);
end

function s = localEmptyDiagnosticCandidate()
s = struct("q_rad",zeros(1,6),"q_deg",zeros(1,6),"tcp_pose",eye(4), ...
    "tcp_rotation_matrix",eye(3), ...
    "welding_gun_mount_axis_world",zeros(1,3), ...
    "target_weld_axis_world",zeros(1,3), ...
    "seam_tangent_world",zeros(1,3), ...
    "axis_angle_deg",NaN,"axis_vs_seam_angle_deg",NaN, ...
    "position_error_mm",NaN,"orientation_error_deg",NaN, ...
    "joint_delta_from_seed_deg",zeros(1,6), ...
    "joint_delta_from_reference_deg",zeros(1,6), ...
    "max_joint_delta_deg",NaN,"travel_deg",NaN,"roll_deg",NaN, ...
    "seed_index",NaN,"rejection_reason","accepted");
end

function v = localUnit(v)
v = double(v(:));
n = norm(v);
if n < 1e-12 || any(~isfinite(v)), v = [NaN;NaN;NaN]; else, v = v/n; end
end

function [robotSelf,gunSelf] = localClassifySelfCollision(scene,q,skipPairs)
% Diagnostic-only pair classification.  The planner's original self-
% collision result remains the hard gate; this helper only separates robot
% self collision from collisions involving the installed gun/wire bodies.
names = string(scene.robot.BodyNames);
gunMask = localGunBodyMask(names);
robotSelf = false; gunSelf = false;
if ~any(gunMask), robotSelf = true; return; end

baseSkip = localSkipCell(skipPairs);
gunPairs = cell(0,2); nonGunPairs = cell(0,2);
for i=1:numel(names)-1
    for j=i+1:numel(names)
        pair = {char(names(i)),char(names(j))};
        if gunMask(i) || gunMask(j), gunPairs(end+1,:) = pair; %#ok<AGROW>
        else, nonGunPairs(end+1,:) = pair; %#ok<AGROW>
        end
    end
end

% Skip all gun-involving pairs to test only the UR5 body chain.
try
    cRobot = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",[baseSkip;gunPairs],"Exhaustive","on");
catch
    cRobot = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",[baseSkip;gunPairs]);
end
robotSelf = any(cRobot(:));

% Skip all non-gun pairs to test collisions within/against the gun package.
try
    cGun = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",[baseSkip;nonGunPairs],"Exhaustive","on");
catch
    cGun = checkCollision(scene.robot,q,"IgnoreSelfCollision","off", ...
        "SkippedSelfCollisions",[baseSkip;nonGunPairs]);
end
gunSelf = any(cGun(:));
end

function mask = localGunBodyMask(names)
lowerNames = lower(names);
mask = strcmpi(names,"WeldingGunMount") | strcmpi(names,"WeldingGunWire") | ...
    strcmpi(names,"WeldTCP") | contains(lowerNames,"guncollision") | ...
    contains(lowerNames,"weldinggun");
end

function pairs = localSkipCell(value)
if iscell(value), pairs = value; return; end
pairs = cell(0,2);
end

function limits = localJointLimits(robot)
limits = zeros(6,2);
j = 0;
for i = 1:numel(robot.Bodies)
    jt = robot.Bodies{i}.Joint;
    if strcmpi(jt.Type,"revolute") || strcmpi(jt.Type,"continuous")
        j = j+1;
        limits(j,:) = double(jt.PositionLimits);
    end
end
if j ~= 6
    error("UR5 关节数应为 6，实际为 %d。",j);
end
end

function out = localMergeOptions(defaults,in)
out = defaults;
if isempty(in), return; end
names = fieldnames(defaults);
for i = 1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i}))
        out.(names{i}) = in.(names{i});
    end
end
end

function v = localGetField(s,name,default)
if isfield(s,name), v = s.(name); else, v = default; end
end

function x = localWrapRad(x)
x = atan2(sin(x),cos(x));
end

function key = localCandidateCacheKey(Ttarget,travelDeg,rollDeg,axisDisabled,axisTolerance)
if nargin < 4, axisDisabled = false; end
if nargin < 5, axisTolerance = 5; end
v = [Ttarget(:); travelDeg; rollDeg; double(axisDisabled); axisTolerance];
key = sprintf("%.10g,",v);
end
