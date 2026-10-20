function summary = benchmark_multistart_first_feasible(options)
%BENCHMARK_MULTISTART_FIRST_FEASIBLE
% Industrial-style first-feasible benchmark for Seam A/B/C.
%
% Each task trial has up to maxAttempts independent initializations.  An
% attempt is Forward ACCP followed, only after failure, by Reverse
% ACCP.  The first complete collision-free path ends the trial.
% Stable planners and the Seam A/B/C geometry definitions are not modified.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(mfilename('fullpath'));
% Formal set: disjoint from all previous development and smoke seeds.
defaults=struct( ...
    'seeds',20270601:20270700, ...
    'marginMm',25,'initialPoints',16,'maxAttempts',3,'timeBudgetSec',30, ...
    'maxRefinementRounds',4,'maxPoints',40,'maxSegmentTcpErrorMm',0.5, ...
    'maxIntermediatePoints',4,'resume',true, ...
    'outputDir',fullfile(root,'multistart_first_feasible_runs_fresh'), ...
    'csvFile',fullfile(root,'multistart_first_feasible_fresh.csv'), ...
    'attemptCsvFile',fullfile(root,'multistart_first_feasible_fresh_attempts.csv'), ...
    'matFile',fullfile(root,'multistart_first_feasible_fresh.mat'));
options=localMerge(defaults,options);
options.seeds=double(options.seeds(:).');
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

specs=localSpecsFromJson(fullfile(fileparts(root),'scene_geometry.json'));
trialRows=repmat(localTrialRow(),0,1);
attemptRows=repmat(localAttemptRow(),0,1);

for is=1:numel(specs)
    spec=specs(is);
    fprintf('\n========================================\n');
    fprintf('%s multi-start first-feasible benchmark\n',spec.name);
    fprintf('Trials=%d, max attempts=%d, budget=%.1f s\n', ...
        numel(options.seeds),options.maxAttempts,options.timeBudgetSec);
    anchors=localInitialAnchors(spec,fullfile(fileparts(root),'scene_geometry.json'), ...
        options.initialPoints,options.marginMm/1000);
    fprintf('start=[%.4f %.4f %.4f], end=[%.4f %.4f %.4f], length=%.1f mm\n', ...
        anchors(1,:),anchors(end,:),norm(anchors(end,:)-anchors(1,:))*1000);
    fprintf('tangent=[%.4f %.4f %.4f], gunAxis=[%.4f %.4f %.4f]\n', ...
        spec.tangentWP,spec.gunAxisWP);

    for it=1:numel(options.seeds)
        taskSeed=options.seeds(it);
        trialFile=fullfile(options.outputDir,sprintf('%s_trial%03d.mat',spec.shortName,it));
        if options.resume && isfile(trialFile)
            A=load(trialFile,'trial');
            if isfield(A,'trial')
                trialRows(end+1,1)=A.trial; %#ok<AGROW>
                fprintf('[%s %3d/%3d] CACHED %s attempts=%d\n',spec.shortName,it,numel(options.seeds), ...
                    ternary(A.trial.pass,'PASS','FAIL'),A.trial.attemptsUsed);
                continue;
            end
        end
        tTrial=tic; trial=localTrialRow(); trial.seam=string(spec.name); trial.trialIndex=it; trial.taskSeed=taskSeed;
        % The actual effective length is determined from the anchors after
        % applying the requested end margin (and is different for Seam A/B/C).
        trial.seamLengthMm=norm(anchors(end,:)-anchors(1,:))*1000;
        trial.marginMm=options.marginMm;
        selected=false;
        attemptsUsed=0;
        for ia=1:options.maxAttempts
            attemptsUsed=ia;
            elapsedBefore=toc(tTrial);
            if elapsedBefore>=options.timeBudgetSec
                trial.budgetExceeded=true; break;
            end
            attemptSeed=taskSeed+(ia-1)*1000;
            % One attempt uses a new random initialization, then tries both
            % physical directions before the next initialization.
            fFile=fullfile(options.outputDir,sprintf('%s_trial%03d_attempt%d_forward.mat',spec.shortName,it,ia));
            rFile=fullfile(options.outputDir,sprintf('%s_trial%03d_attempt%d_reverse.mat',spec.shortName,it,ia));
            fOpts=localPlannerOptions(spec,attemptSeed,fFile,options,[]);
            fOpts.roundOutputPrefix=fullfile(options.outputDir,sprintf('%s_trial%03d_attempt%d_fwd_round',spec.shortName,it,ia));
            fprintf('[%s %3d/%3d] attempt %d Forward ... ',spec.shortName,it,numel(options.seeds),ia);
            t=tic; f=replan_weld_seam_generalization_feasibility(fOpts); fWall=toc(t);
            fWithinBudget=(toc(tTrial)<=options.timeBudgetSec);
            a=localAttemptRow(); a.seam=string(spec.name); a.trialIndex=it; a.taskSeed=taskSeed; a.attempt=ia; a.direction="forward";
            a.randomSeed=attemptSeed; a.pass=logical(f.complete); a.withinBudget=fWithinBudget; a.wallTimeSec=fWall; a.algorithmTimeSec=localAlgTime(f);
            a.ikCalls=localIkCalls(f); a.finalPoints=localNum(f,'finalPointCount',NaN); a.tcpErrorMm=localFinalTcp(f); a.maxJointJumpDeg=localNum(f,'maxJointJumpDeg',NaN); a.failureStage=localFailureStage(f);
            attemptRows(end+1,1)=a; %#ok<AGROW>
            trial.forwardAttempts=trial.forwardAttempts+1; trial.forwardWallTimeSec=trial.forwardWallTimeSec+fWall;
            trial.forwardAlgorithmTimeSec=trial.forwardAlgorithmTimeSec+a.algorithmTimeSec; trial.totalIkCalls=trial.totalIkCalls+a.ikCalls;
            if a.pass && ~a.withinBudget, trial.latePassAttempts=trial.latePassAttempts+1; end
            fprintf('%s %.2fs',ternary(a.pass && a.withinBudget,'PASS',ternary(a.pass,'PASS-LATE','FAIL')),fWall);
            if a.pass && a.withinBudget
                selected=true; trial.selectedDirection="forward"; trial.timeToFirstFeasibleSec=toc(tTrial); trial.totalWallTimeSec=trial.timeToFirstFeasibleSec; trial.pass=true; trial.attemptsUsed=ia; trial.selectedFinalPoints=a.finalPoints; trial.selectedTcpErrorMm=a.tcpErrorMm; trial.selectedMaxJointJumpDeg=a.maxJointJumpDeg; break;
            end
            if toc(tTrial)>=options.timeBudgetSec
                trial.budgetExceeded=true; break;
            end
            rOpts=localPlannerOptions(spec,attemptSeed,rFile,options,flipud(anchors));
            rOpts.roundOutputPrefix=fullfile(options.outputDir,sprintf('%s_trial%03d_attempt%d_rev_round',spec.shortName,it,ia));
            fprintf(' -> Reverse ... ');
            t=tic; r=replan_weld_seam_generalization_feasibility(rOpts); rWall=toc(t);
            rWithinBudget=(toc(tTrial)<=options.timeBudgetSec);
            a=localAttemptRow(); a.seam=string(spec.name); a.trialIndex=it; a.taskSeed=taskSeed; a.attempt=ia; a.direction="reverse";
            a.randomSeed=attemptSeed; a.pass=logical(r.complete); a.withinBudget=rWithinBudget; a.wallTimeSec=rWall; a.algorithmTimeSec=localAlgTime(r);
            a.ikCalls=localIkCalls(r); a.finalPoints=localNum(r,'finalPointCount',NaN); a.tcpErrorMm=localFinalTcp(r); a.maxJointJumpDeg=localNum(r,'maxJointJumpDeg',NaN); a.failureStage=localFailureStage(r);
            attemptRows(end+1,1)=a; %#ok<AGROW>
            trial.reverseAttempts=trial.reverseAttempts+1; trial.reverseWallTimeSec=trial.reverseWallTimeSec+rWall;
            trial.reverseAlgorithmTimeSec=trial.reverseAlgorithmTimeSec+a.algorithmTimeSec; trial.totalIkCalls=trial.totalIkCalls+a.ikCalls;
            if a.pass && ~a.withinBudget, trial.latePassAttempts=trial.latePassAttempts+1; end
            fprintf('%s %.2fs',ternary(a.pass && a.withinBudget,'PASS',ternary(a.pass,'PASS-LATE','FAIL')),rWall);
            if a.pass && a.withinBudget
                selected=true; trial.selectedDirection="reverse"; trial.timeToFirstFeasibleSec=toc(tTrial); trial.totalWallTimeSec=trial.timeToFirstFeasibleSec; trial.pass=true; trial.attemptsUsed=ia; trial.selectedFinalPoints=a.finalPoints; trial.selectedTcpErrorMm=a.tcpErrorMm; trial.selectedMaxJointJumpDeg=a.maxJointJumpDeg; break;
            end
            if toc(tTrial)>=options.timeBudgetSec
                trial.budgetExceeded=true; break;
            end
            fprintf('\n');
        end
        if ~selected
            trial.attemptsUsed=attemptsUsed;
            trial.totalWallTimeSec=toc(tTrial); trial.timeToFirstFeasibleSec=NaN;
        end
        trial.pass=logical(selected); trial.outputFile=string(trialFile);
        save(trialFile,'trial','-v7.3');
        trialRows(end+1,1)=trial; %#ok<AGROW>
        fprintf(' => %s, attempts=%d, total=%.2fs\n',ternary(trial.pass,'PASS','FAIL'),trial.attemptsUsed,trial.totalWallTimeSec);
    end
end

TT=struct2table(trialRows); AT=struct2table(attemptRows);
writetable(TT,options.csvFile); writetable(AT,options.attemptCsvFile);
summary=struct('name','Multi-start first-feasible ACCP benchmark', ...
    'seeds',options.seeds,'maxAttempts',options.maxAttempts,'timeBudgetSec',options.timeBudgetSec, ...
    'specs',specs,'trials',trialRows,'attempts',attemptRows, ...
    'csvFile',options.csvFile,'attemptCsvFile',options.attemptCsvFile, ...
    'matFile',options.matFile,'created',char(datetime('now')));
save(options.matFile,'summary','-v7.3');
fprintf('\nFirst-feasible summary:\n');
for is=1:numel(specs)
    idx=strcmp(string(TT.seam),specs(is).name); pass=TT.pass(idx);
    t=TT.timeToFirstFeasibleSec(idx); t=t(isfinite(t));
    fprintf('%s: PASS %d/%d, median first-feasible %.3f s, P90 %.3f s\n', ...
        specs(is).name,sum(pass),sum(idx),localPercentile(t,50),localPercentile(t,90));
end
fprintf('Saved: %s\n',options.csvFile);
end

function o=localPlannerOptions(spec,seed,file,options,positions)
o=struct('initialPoints',options.initialPoints,'seamSpec',spec, ...
    'initialSeamPositionsWP',positions,'maxRefinementRounds',options.maxRefinementRounds, ...
    'maxPoints',options.maxPoints,'maxSegmentTcpErrorMm',options.maxSegmentTcpErrorMm, ...
    'maxIntermediatePoints',options.maxIntermediatePoints,'endMarginM',options.marginMm/1000, ...
    'randomSeed',seed,'enablePostRecovery',true,'outputFile',file, ...
    'animate',false,'publish',false);
end

function specs=localSpecsFromJson(file)
D=jsondecode(fileread(file)); p=D.workpiece.collision_proxies; if iscell(p), p=[p{:}]; end
required=["WorkpieceCollision_bottom","WorkpieceCollision_wall_x0","WorkpieceCollision_wall_x1","WorkpieceCollision_wall_y1"];
names=string({p.name}); if any(~ismember(required,names)), error('Missing Workpiece collision proxies.'); end
b=localProxy(p,"WorkpieceCollision_bottom"); x0=localProxy(p,"WorkpieceCollision_wall_x0"); x1=localProxy(p,"WorkpieceCollision_wall_x1"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bmin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2; bmax=double(b.pose_workpiece.position(:))+double(b.size_m(:))/2;
ymax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2; z=bmax(3);
specs=repmat(localSpec(),1,3);
specs(1)=localMake("bottom_x_wall_y1","Seam_A",2,double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2,-1,[0;1;-1]/sqrt(2),"U-interior: y < wall_y1 inner face, z > bottom top face",bmin,bmax,ymax,z);
specs(2)=localMake("bottom_x_wall_x0","Seam_B",1,double(x0.pose_workpiece.position(1))+double(x0.size_m(1))/2,+1,[-1;0;-1]/sqrt(2),"U-interior: x > wall_x0 inner face, z > bottom top face",bmin,bmax,ymax,z);
specs(3)=localMake("bottom_x_wall_x1","Seam_C",1,double(x1.pose_workpiece.position(1))-double(x1.size_m(1))/2,-1,[1;0;-1]/sqrt(2),"U-interior: x < wall_x1 inner face, z > bottom top face",bmin,bmax,ymax,z);
end

function s=localMake(name,shortName,axis,coord,freeDir,g,desc,bmin,bmax,ymax,z)
s=localSpec(); s.name=name; s.shortName=shortName; s.wallAxis=axis; s.wallInnerCoordinate=coord; s.freeDirection=freeDir; s.gunAxisWP=g/norm(g); s.description=desc; s.tangentWP=([1 0 0]);
if axis==1, s.tangentWP=[0 1 0]; end
s.lengthMm=NaN; s.bottomTopZ=z; s.bottomSize=(bmax-bmin).';
end
function s=localSpec()
s=struct('name',"",'shortName',"",'wallAxis',2,'wallInnerCoordinate',NaN,'freeDirection',-1,'gunAxisWP',zeros(3,1),'description',"",'tangentWP',zeros(1,3),'lengthMm',NaN,'bottomTopZ',NaN,'bottomSize',zeros(1,3));
end
function P=localInitialAnchors(spec,jsonFile,N,margin)
D=jsondecode(fileread(jsonFile)); p=D.workpiece.collision_proxies; if iscell(p), p=[p{:}]; end
b=localProxy(p,"WorkpieceCollision_bottom"); x0=localProxy(p,"WorkpieceCollision_wall_x0"); x1=localProxy(p,"WorkpieceCollision_wall_x1"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bmin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2; z=double(b.pose_workpiece.position(3))+double(b.size_m(3))/2;
ymax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2; y0=bmin(2)+margin; yL=ymax-margin;
if yL<=y0, error('%s: margin too large.',spec.name); end
if spec.wallAxis==2
    % Match localInferSeam: the usable x interval is between the inner
    % faces of wall_x0/wall_x1, then trimmed by the end margin.
    xMin=max(bmin(1),double(x0.pose_workpiece.position(1))+double(x0.size_m(1))/2);
    xMax=min(double(b.pose_workpiece.position(1))+double(b.size_m(1))/2,double(x1.pose_workpiece.position(1))-double(x1.size_m(1))/2);
    x0eff=xMin+margin; xLeff=xMax-margin;
    if xLeff<=x0eff, error('%s: margin leaves no effective x seam.',spec.name); end
    P=[linspace(x0eff,xLeff,N).',repmat(spec.wallInnerCoordinate,N,1),repmat(z,N,1)];
else
    P=[repmat(spec.wallInnerCoordinate,N,1),linspace(y0,yL,N).',repmat(z,N,1)];
end
end
function r=localTrialRow()
r=struct('seam',"",'trialIndex',NaN,'taskSeed',NaN,'seamLengthMm',NaN,'marginMm',NaN,'pass',false,'attemptsUsed',0,'forwardAttempts',0,'reverseAttempts',0,'budgetExceeded',false,'latePassAttempts',0,'selectedDirection',"",'timeToFirstFeasibleSec',NaN,'totalWallTimeSec',NaN,'forwardWallTimeSec',0,'reverseWallTimeSec',0,'forwardAlgorithmTimeSec',0,'reverseAlgorithmTimeSec',0,'totalIkCalls',0,'selectedFinalPoints',NaN,'selectedTcpErrorMm',NaN,'selectedMaxJointJumpDeg',NaN,'outputFile',"");
end
function r=localAttemptRow()
r=struct('seam',"",'trialIndex',NaN,'taskSeed',NaN,'attempt',NaN,'direction',"",'randomSeed',NaN,'pass',false,'withinBudget',false,'wallTimeSec',NaN,'algorithmTimeSec',NaN,'ikCalls',NaN,'finalPoints',NaN,'tcpErrorMm',NaN,'maxJointJumpDeg',NaN,'failureStage',"");
end
function p=localProxy(a,n), i=find(strcmp(string({a.name}),n),1); if isempty(i),error('Missing proxy %s',n);end;p=a(i);end
function t=localAlgTime(r), t=localNum(r,'feasibilityAdaptiveAlgorithmTimeSec',0)+localNum(r,'precisionAlgorithmTimeSec',0);end
function n=localIkCalls(r), n=localNum(r,'feasibilityAdaptiveIkCalls',0)+localNum(r,'adaptiveIkCalls',0);end
function v=localNum(s,n,d),if isfield(s,n)&&~isempty(s.(n)),v=double(s.(n));else,v=d;end,end
function v=localFinalTcp(s),v=NaN;if isfield(s,'finalTcpMetrics')&&isfield(s.finalTcpMetrics,'segmentPositionErrorMm')&&~isempty(s.finalTcpMetrics.segmentPositionErrorMm),v=max(s.finalTcpMetrics.segmentPositionErrorMm);end,end
function stage=localFailureStage(s),stage="failed";if isfield(s,'postRecoveryUsed')&&s.postRecoveryUsed,stage="post_recovery";end;if isfield(s,'refinementRounds')&&~isempty(s.refinementRounds),rr=s.refinementRounds;for k=numel(rr):-1:1,if isfield(rr(k),'recovery')&&strlength(string(rr(k).recovery))>0,stage=string(rr(k).recovery);return;end,end,end,end
function v=localPercentile(x,p),if isempty(x),v=NaN;else,x=sort(x(:));v=x(max(1,min(numel(x),ceil(p/100*numel(x)))));end,end
function out=localMerge(d,c),out=d;f=fieldnames(c);for k=1:numel(f),out.(f{k})=c.(f{k});end,end
function y=ternary(c,a,b),if c,y=a;else,y=b;end,end
