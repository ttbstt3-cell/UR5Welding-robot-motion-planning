function report = check_end_margin_zero_reachability(resultFile)
%CHECK_END_MARGIN_ZERO_REACHABILITY Check zero-trim weld endpoints only.
%
% This diagnostic does not modify the saved plan, scene, controller, or
% calibration. It reuses the current tool/workpiece model and tests the
% same TCP orientations as the validated plan after extending the seam to
% the two inner plate boundaries (endMarginM = 0).

arguments
    resultFile (1,1) string = ""
end

here = fileparts(mfilename("fullpath"));
if strlength(resultFile)==0
    resultFile = fullfile(here,"weld_actual_axis_replan.mat");
end

% sync_ur5_joint_states.m is a script with clear/clc commands. Run it in a
% helper workspace so those commands cannot erase this function's variables.
scene = localLoadScene(fullfile(here,"sync_ur5_joint_states.m"));
s = load(resultFile,"result");
saved = s.result;

scene.weldGunAxisWP = [0;1;-1]/sqrt(2);
scene.selfCollisionSkipPairs = localStructuralSelfCollisionSkips(scene.robot);

qPath = double(saved.qPath);
qPath = unwrap(qPath,[],1);
currentMarginM = double(saved.options.endMarginM);

% The saved seam was generated as [xMin+m, xMax-m]. Remove that same
% amount at both ends to obtain the exact inner-face endpoints.
pStartZeroWP = saved.seam.startWP(:) - currentMarginM*saved.seam.tangentWP(:);
pEndZeroWP   = saved.seam.endWP(:)   + currentMarginM*saved.seam.tangentWP(:);
TWB = scene.T_MatBase_Workpiece;
pStartZeroB = TWB*[pStartZeroWP;1];
pEndZeroB   = TWB*[pEndZeroWP;1];

fprintf("\n========================================\n");
fprintf("ZERO END-MARGIN ENDPOINT DIAGNOSTIC\n");
fprintf("========================================\n");
fprintf("Saved end margin       : %.1f mm\n",currentMarginM*1000);
fprintf("Tested end margin      : 0.0 mm\n");
fprintf("Zero-margin TCP start  : [%.5f %.5f %.5f] m (MATLAB base)\n",pStartZeroB(1:3));
fprintf("Zero-margin TCP end    : [%.5f %.5f %.5f] m (MATLAB base)\n",pEndZeroB(1:3));

% Use the current plan's endpoint orientations. This isolates the effect of
% removing the trim; it does not invent a new work/travel/roll pose.
endpointIndices = [1,size(qPath,1)];
endpointLabels = ["start","end"];
endpointReport = repmat(localEmptyReport(),1,2);

for k = 1:2
    i = endpointIndices(k);
    Ttarget = getTransform(scene.robot,qPath(i,:),scene.tcpBodyName);
    if i==1
        Ttarget(1:3,4) = pStartZeroB(1:3);
    else
        Ttarget(1:3,4) = pEndZeroB(1:3);
    end

    % Keep the tested orientation seed close to the validated endpoint and
    % add deterministic random restarts to expose alternate IK branches.
    rng(20260917+i,"twister");
    qSeeds = [qPath(i,:); scene.qMat0];
    for j = 1:96
        qSeeds(end+1,:) = randomConfiguration(scene.robot); %#ok<AGROW>
    end

    r = generate_actual_axis_ik_candidates(scene,Ttarget,0,0,qSeeds,struct( ...
        "positionToleranceM",1e-4, ...
        "orientationToleranceDeg",0.5, ...
        "sideToleranceDeg",5, ...
        "sideMarginM",1e-3, ...
        "checkSelfCollision",true));

    endpointReport(k).label = endpointLabels(k);
    endpointReport(k).target = Ttarget;
    endpointReport(k).diagnostic = r;

    fprintf("\n%s endpoint (same orientation as saved plan):\n",upper(endpointLabels(k)));
    fprintf("  IK calls / pose-valid : %d / %d\n",r.numIKCalls,r.numIKPoseOK);
    fprintf("  rejected by side      : %d\n",r.numSideRejected);
    fprintf("  rejected by env       : %d\n",r.numEnvironmentRejected);
    fprintf("  rejected by self      : %d\n",r.numSelfRejected);
    fprintf("  accepted collision-free states: %d\n",r.numAccepted);
    fprintf("  result                : %s\n",ternary(r.numAccepted>0,"REACHABLE","NO FEASIBLE STATE"));
end

report = struct();
report.resultFile = resultFile;
report.savedEndMarginM = currentMarginM;
report.testedEndMarginM = 0;
report.startWP = pStartZeroWP;
report.endWP = pEndZeroWP;
report.startBase = pStartZeroB(1:3);
report.endBase = pEndZeroB(1:3);
report.endpoints = endpointReport;
report.pass = all(arrayfun(@(x)x.diagnostic.numAccepted>0,endpointReport));

fprintf("\nZERO-MARGIN ENDPOINT TEST: %s\n",ternary(report.pass,"PASS","FAIL"));
fprintf("Note: this is an endpoint reachability/collision test only; it does\n");
fprintf("not replace a full 80-point zero-margin replanning run.\n");

end

function s = localEmptyReport()
s = struct("label","","target",eye(4),"diagnostic",struct());
end

function pairs = localStructuralSelfCollisionSkips(robot)
names = robot.BodyNames;
pairs = cell(0,2);
for i = 1:min(10,numel(names)-1)
    pairs(end+1,:) = {names{i},names{i+1}}; %#ok<AGROW>
end
pairs(end+1,:) = {"WeldingGunMount","wrist_3_link"};
pairs(end+1,:) = {"WeldingGunMount","ee_link"};
pairs(end+1,:) = {"WeldingGunMount","WeldTCP"};
end

function out = ternary(condition,a,b)
if condition, out = a; else, out = b; end
end

function scene = localLoadScene(syncFile)
run(syncFile);
end
