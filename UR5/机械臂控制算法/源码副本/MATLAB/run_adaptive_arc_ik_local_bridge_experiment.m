function report = run_adaptive_arc_ik_local_bridge_experiment(options)
%RUN_ADAPTIVE_ARC_IK_LOCAL_BRIDGE_EXPERIMENT
% Test-only local forward/backward bridge analysis for failed A-ARC-IK seeds.
%
% It consumes the saved local feasibility diagnostic and the corresponding
% failure MAT files.  It does not change or rerun the global planner.  A
% forward mask comes from the left boundary; a backward mask is propagated
% from the rightmost nonempty local layer.  A bridge is accepted only when
% the same hard pose, joint-step and interpolated collision checks pass.
%
% Example:
%   addpath('/home/ttbstt/jixiebi/UR5/matlab仿真');
%   report = run_adaptive_arc_ik_local_bridge_experiment();

if nargin<1 || isempty(options), options=struct(); end
options=localOptions(options);
D=load(options.diagnosticMat);
R=D.report;
out=cell(0,1);

for k=1:numel(R.cases)
    c=R.cases(k);
    failureFile=localFailureFile(c.seed,options.failureFiles);
    if failureFile==""
        warning('No failure MAT mapping for seed %d.',c.seed); continue;
    end
    fprintf('\n[Bridge %d] seed=%d window=%s\n',k,c.seed,mat2str(c.window));
    rr=load(failureFile).result;
    scene=rr.scene;
    x=localBridgeCase(c,scene,options);
    out{end+1,1}=x; %#ok<AGROW>
    fprintf('  backward reachable: [%s]\n',mat2str(x.backwardReachableCounts));
    fprintf('  bridges tested=%d free=%d\n',x.bridgeTests,x.bridgeFree);
    if x.bridgeFound
        fprintf('  bridge: Point %d -> %d, max dq=%.3f deg, travel %.1f -> %.1f, roll %.1f -> %.1f\n', ...
            x.bridge.fromPoint,x.bridge.toPoint,x.bridge.maxJointDeltaDeg, ...
            x.bridge.fromTravelDeg,x.bridge.toTravelDeg,x.bridge.fromRollDeg,x.bridge.toRollDeg);
    else
        fprintf('  no bridge covering the complete local window.\n');
    end
end

report=struct();
report.generatedAt=string(datetime('now'));
report.options=options;
report.cases=out;
save(options.outputMat,'report','-v7.3');
localWriteCsv(out,options.outputCsv);
fprintf('\nSaved bridge MAT: %s\nSaved bridge CSV: %s\n',options.outputMat,options.outputCsv);
end

function options=localOptions(options)
root='/home/ttbstt/jixiebi/UR5/matlab仿真';
defaults=struct( ...
    'diagnosticMat',fullfile(root,'adaptive_arc_ik_failure_diagnostics.mat'), ...
    'failureFiles',[string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269018.mat')); ...
                     string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269019.mat')); ...
                     string(fullfile(root,'adaptive_arc_ik_recovery_runs','adaptive_recovery_seed20269022.mat'))], ...
    'edgeResolutionDeg',3,'maxJointStepDeg',15,'maxTravelStepDeg',15,'maxRollStepDeg',45, ...
    'outputMat',fullfile(root,'adaptive_arc_ik_local_bridge_experiment.mat'), ...
    'outputCsv',fullfile(root,'adaptive_arc_ik_local_bridge_experiment.csv'));
n=fieldnames(defaults);
for i=1:numel(n)
    if ~isfield(options,n{i}) || isempty(options.(n{i})), options.(n{i})=defaults.(n{i}); end
end
if ischar(options.failureFiles)
    options.failureFiles=string(options.failureFiles);
else
    options.failureFiles=string(options.failureFiles(:));
end
end

function file=localFailureFile(seed,files)
file="";
for i=1:numel(files)
    if contains(char(files(i)),sprintf('seed%d',seed)), file=files(i); return; end
end
end

function x=localBridgeCase(c,scene,options)
x=localEmptyCase();
x.seed=c.seed; x.window=c.window; x.forwardReachableCounts=c.reachableCounts;
x.stateCounts=c.stateCounts; x.layers=c.layers;
N=numel(c.layers);
fwd=c.reachableMasks;
% Use the rightmost layer that actually contains candidates.  If later
% layers are empty, the experiment reports the missing-node condition
% separately instead of calling a partial chain a complete bridge.
rightLayer=find(c.stateCounts>0,1,'last');
if isempty(rightLayer), rightLayer=0; end
x.rightFrontierLayer=rightLayer;
x.completeWindow = (rightLayer==N && N>0);

bwd=cell(N,1);
if rightLayer>0
    bwd{rightLayer}=true(1,numel(c.layers{rightLayer}));
    for i=rightLayer-1:-1:1
        bwd{i}=false(1,numel(c.layers{i}));
        for a=1:numel(c.layers{i})
            for b=1:numel(c.layers{i+1})
                [ok,info]=localEdgeCheck(scene,c.layers{i}(a),c.layers{i+1}(b),options);
                x.edgeDiag(end+1,1)=info; %#ok<AGROW>
                if ok && bwd{i+1}(b), bwd{i}(a)=true; break; end
            end
        end
    end
else
    bwd=cell(N,1);
end
x.backwardMasks=bwd;
x.backwardReachableCounts=zeros(1,N);
for i=1:N, if ~isempty(bwd{i}), x.backwardReachableCounts(i)=sum(bwd{i}); end, end

% Search F_i -> B_(i+1).  This is the local bridge condition requested by
% the experiment.  Rank valid bridges by maximum joint change.
bridges=repmat(localEmptyBridge(),0,1);
for i=1:max(0,rightLayer-1)
    if isempty(fwd{i}) || isempty(bwd{i+1}), continue; end
    for a=1:numel(c.layers{i})
        if ~fwd{i}(a), continue; end
        for b=1:numel(c.layers{i+1})
            if ~bwd{i+1}(b), continue; end
            [ok,info]=localEdgeCheck(scene,c.layers{i}(a),c.layers{i+1}(b),options);
            x.edgeDiag(end+1,1)=info; %#ok<AGROW>
            if ok
                br=localEmptyBridge();
                br.fromPoint=c.layers{i}(a).pointIndex; br.toPoint=c.layers{i+1}(b).pointIndex;
                br.fromIndex=a; br.toIndex=b; br.fromTravelDeg=c.layers{i}(a).travelDeg;
                br.toTravelDeg=c.layers{i+1}(b).travelDeg; br.fromRollDeg=c.layers{i}(a).rollDeg;
                br.toRollDeg=c.layers{i+1}(b).rollDeg; br.maxJointDeltaDeg=info.maxJointDeltaDeg;
                br.qFrom=c.layers{i}(a).q; br.qTo=c.layers{i+1}(b).q;
                br.edgeFree=true; bridges(end+1,1)=br; %#ok<AGROW>
            end
        end
    end
end
x.bridges=bridges; x.bridgeTests=numel(x.edgeDiag);
x.bridgeFree=sum([x.edgeDiag.free]);
if isempty(bridges)
    x.bridgeFound=false; x.completeBridge=false; x.bridge=localEmptyBridge();
else
    [~,j]=min([bridges.maxJointDeltaDeg]); x.bridge=bridges(j);
    x.bridgeFound=true; x.completeBridge=x.completeWindow;
end
x.diagnosis=localBridgeDiagnosis(x);
end

function txt=localBridgeDiagnosis(x)
if x.completeBridge && x.bridgeFound
    txt="LOCAL_BIDIRECTIONAL_BRIDGE_FOUND";
elseif x.bridgeFound && ~x.completeWindow
    txt="PARTIAL_BRIDGE_BUT_RIGHT_LOCAL_LAYER_MISSING";
else
    txt="NO_LOCAL_BIDIRECTIONAL_BRIDGE";
end
end

function [ok,info]=localEdgeCheck(scene,a,b,o)
info=localEmptyEdge(); info.fromPoint=a.pointIndex; info.toPoint=b.pointIndex;
info.fromTravelDeg=a.travelDeg; info.toTravelDeg=b.travelDeg;
info.dTravelDeg=abs(localAngleDiffDeg(a.travelDeg,b.travelDeg));
info.dRollDeg=abs(localAngleDiffDeg(a.rollDeg,b.rollDeg));
dq=atan2(sin(b.q-a.q),cos(b.q-a.q)); info.maxJointDeltaDeg=max(abs(rad2deg(dq)));
if info.dTravelDeg>o.maxTravelStepDeg || info.dRollDeg>o.maxRollStepDeg
    info.rejectedPose=true; ok=false; return;
end
if info.maxJointDeltaDeg>o.maxJointStepDeg
    info.rejectedJoint=true; ok=false; return;
end
info.tests=max(1,ceil(info.maxJointDeltaDeg/o.edgeResolutionDeg))+1;
info.free=localEdgeFree(scene,a.q,b.q,o.edgeResolutionDeg);
info.rejectedCollision=~info.free; ok=info.free;
end

function free=localEdgeFree(scene,q1,q2,resDeg)
dq=atan2(sin(q2-q1),cos(q2-q1)); n=max(1,ceil(max(abs(rad2deg(dq)))/resDeg)); free=true;
for k=0:n
    if ~localFullStateFree(scene,q1+(k/n)*dq), free=false; return; end
end
end

function free=localFullStateFree(scene,q)
try, [cc,~,~]=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on','Exhaustive','on');
catch, cc=checkCollision(scene.robot,q,scene.environment,'IgnoreSelfCollision','on'); end
if any(cc(:)), free=false; return; end
try, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs,'Exhaustive','on');
catch, cs=checkCollision(scene.robot,q,'IgnoreSelfCollision','off','SkippedSelfCollisions',scene.selfCollisionSkipPairs); end
free=~any(cs(:));
end

function d=localAngleDiffDeg(a,b), d=rad2deg(atan2(sin(deg2rad(a-b)),cos(deg2rad(a-b)))); end

function x=localEmptyCase()
x=struct('seed',NaN,'window',zeros(1,0),'stateCounts',zeros(1,0),'layers',{{}}, ...
    'forwardReachableCounts',zeros(1,0),'backwardReachableCounts',zeros(1,0), ...
    'rightFrontierLayer',0,'completeWindow',false,'backwardMasks',{{}}, ...
    'edgeDiag',repmat(localEmptyEdge(),0,1),'edgeTests',0,'edgeFree',0, ...
    'bridges',repmat(localEmptyBridge(),0,1),'bridgeFound',false,'completeBridge',false, ...
    'bridge',localEmptyBridge(),'diagnosis',"");
end

function x=localEmptyEdge()
x=struct('fromPoint',0,'toPoint',0,'fromTravelDeg',0,'toTravelDeg',0,'dTravelDeg',0,'dRollDeg',0, ...
    'maxJointDeltaDeg',0,'tests',0,'free',false,'rejectedPose',false,'rejectedJoint',false,'rejectedCollision',false);
end

function x=localEmptyBridge()
x=struct('fromPoint',0,'toPoint',0,'fromIndex',0,'toIndex',0,'fromTravelDeg',0,'toTravelDeg',0, ...
    'fromRollDeg',0,'toRollDeg',0,'maxJointDeltaDeg',Inf,'qFrom',zeros(1,6),'qTo',zeros(1,6),'edgeFree',false);
end

function localWriteCsv(cases,file)
fid=fopen(file,'w');
fprintf(fid,'seed,rightFrontierLayer,completeWindow,forwardFinal,backwardFinal,bridgeFound,completeBridge,bridgeFrom,bridgeTo,maxJointDeltaDeg,bridgeTests,bridgeFree,diagnosis\n');
for i=1:numel(cases)
    x=cases{i};
    if x.bridgeFound, b=x.bridge; else, b=localEmptyBridge(); end
    fprintf(fid,'%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%d,%d,%s\n',x.seed,x.rightFrontierLayer,x.completeWindow, ...
        x.forwardReachableCounts(end),x.backwardReachableCounts(end),x.bridgeFound,x.completeBridge, ...
        b.fromPoint,b.toPoint,b.maxJointDeltaDeg,x.bridgeTests,x.bridgeFree,char(x.diagnosis));
end
fclose(fid);
end
