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
    "sideMarginM",1e-3, ...
    "duplicateToleranceDeg",0.5, ...
    "checkSelfCollision",true, ...
    "lazyCollision",false, ...
    "reuseCandidateCache",false, ...
    "verbose",false);
options = localMergeOptions(defaults, options);

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
cacheKey = localCandidateCacheKey(Ttarget,travelDeg,rollDeg);
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
numIKPoseOK = 0;
numSideRejected = 0;
numJointRejected = 0;
numEnvRejected = 0;
numSelfRejected = 0;

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
    if posErr > options.positionToleranceM || ...
            rad2deg(oriErr) > options.orientationToleranceDeg
        continue;
    end
    numIKPoseOK = numIKPoseOK + 1;

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
    sideOK = sideDot >= cosd(options.sideToleranceDeg) && ...
        pHeadWP(2) < 0.230-options.sideMarginM && ...
        pHeadWP(3) > 0.005+options.sideMarginM;
    if ~sideOK
        numSideRejected = numSideRejected + 1;
        continue;
    end

    envColliding = false;
    selfColliding = false;
    % Lazy mode defers only interpolated edge checks.  Node collision must
    % still be checked here; otherwise thousands of colliding IK states can
    % enter the layered graph and make it much larger than the baseline.
    try
        [envColliding,~,~] = checkCollision(robot,qSol,scene.environment, ...
            "IgnoreSelfCollision","on","Exhaustive","on");
    catch
        envColliding = checkCollision(robot,qSol,scene.environment, ...
            "IgnoreSelfCollision","on");
    end
    envColliding = any(envColliding(:));
    if envColliding
        numEnvRejected = numEnvRejected + 1;
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
result.numIKPoseOK = numIKPoseOK;
result.numSideRejected = numSideRejected;
result.numJointRejected = numJointRejected;
result.numEnvironmentRejected = numEnvRejected;
result.numSelfRejected = numSelfRejected;
result.numAccepted = numel(result.states);
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

function key = localCandidateCacheKey(Ttarget,travelDeg,rollDeg)
v = [Ttarget(:); travelDeg; rollDeg];
key = sprintf("%.10g,",v);
end
