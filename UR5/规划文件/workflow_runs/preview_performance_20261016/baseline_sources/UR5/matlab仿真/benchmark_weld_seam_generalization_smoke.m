function summary = benchmark_weld_seam_generalization_smoke(options)
%BENCHMARK_WELD_SEAM_GENERALIZATION_SMOKE
% Validate Seam B/C geometry and run five Forward/Reverse smoke seeds per
% seam.  This is intentionally separate from the stable Seam A planner.

if nargin < 1 || isempty(options), options=struct(); end
root=fileparts(mfilename('fullpath'));
defaults=struct('seeds',20270301:20270305,'marginMm',25, ...
    'initialPoints',16,'outputDir',fullfile(root,'seam_generalization_smoke_runs'), ...
    'csvFile',fullfile(root,'seam_generalization_smoke.csv'), ...
    'matFile',fullfile(root,'seam_generalization_smoke.mat'));
options=localMerge(defaults,options);
if ~isfolder(options.outputDir), mkdir(options.outputDir); end

specs=localSpecsFromJson(fullfile(fileparts(root),'scene_geometry.json'));
rows=repmat(localRow(),0,1);
for is=1:numel(specs)
    spec=specs(is);
    anchors=localInitialAnchors(spec,fullfile(fileparts(root),'scene_geometry.json'),options.initialPoints,options.marginMm/1000);
    fprintf('\n========================================\n');
    fprintf('%s smoke test\n',spec.name);
    fprintf('start WP = [%.4f %.4f %.4f] m\n',anchors(1,:));
    fprintf('end   WP = [%.4f %.4f %.4f] m\n',anchors(end,:));
    fprintf('length = %.3f mm, margin = %.1f mm\n',norm(anchors(end,:)-anchors(1,:))*1000,options.marginMm);
    fprintf('tangent = [%.4f %.4f %.4f]\n',spec.tangentWP);
    fprintf('gun axis = [%.4f %.4f %.4f]\n',spec.gunAxisWP);
    fprintf('side = %s\n',spec.description);

    for ik=1:numel(options.seeds)
        seed=options.seeds(ik);
        stem=sprintf('%s_seed%d',spec.shortName,seed);
        fFile=fullfile(options.outputDir,[stem '_forward.mat']);
        rFile=fullfile(options.outputDir,[stem '_reverse.mat']);
        fOpts=localPlannerOptions(spec,seed,fFile,options,[]);
        fprintf('[%s %d/%d] Forward seed=%d ... ',spec.shortName,ik,numel(options.seeds),seed);
        t=tic; f=replan_weld_seam_generalization_feasibility(fOpts); fw=toc(t);
        fprintf('%s (%.2fs)',ternary(f.complete,'PASS','FAIL'),fw);
        r=struct('complete',false); rw=NaN;
        if ~f.complete
            fprintf(' -> Reverse ... ');
            rOpts=localPlannerOptions(spec,seed,rFile,options,flipud(anchors));
            t=tic; r=replan_weld_seam_generalization_feasibility(rOpts); rw=toc(t);
            fprintf('%s (%.2fs)',ternary(r.complete,'PASS','FAIL'),rw);
        end
        row=localRow(); row.seam=string(spec.name); row.seed=seed;
        row.forwardPass=logical(f.complete); row.reverseTried=~f.complete;
        row.reversePass=logical(r.complete); row.cascadePass=row.forwardPass||row.reversePass;
        row.forwardTimeSec=fw; row.reverseTimeSec=rw; row.cascadeTimeSec=fw+ternary(row.reverseTried,rw,0);
        row.forwardFinalPoints=localNum(f,'finalPointCount',NaN); row.reverseFinalPoints=localNum(r,'finalPointCount',NaN);
        row.forwardTcpErrorMm=localFinalTcp(f); row.reverseTcpErrorMm=localFinalTcp(r);
        row.forwardMaxJumpDeg=localNum(f,'maxJointJumpDeg',NaN); row.reverseMaxJumpDeg=localNum(r,'maxJointJumpDeg',NaN);
        row.failureStage=localFailureStage(f); if row.cascadePass, row.failureStage="PASS"; end
        rows(end+1,1)=row; %#ok<AGROW>
        fprintf('\n');
    end
end

T=struct2table(rows); writetable(T,options.csvFile);
summary=struct('specs',specs,'seeds',options.seeds,'rows',rows, ...
    'csvFile',options.csvFile,'matFile',options.matFile,'created',char(datetime('now')));
save(options.matFile,'summary','-v7.3');
fprintf('\nSmoke summary:\n');
for is=1:numel(specs)
    idx=strcmp(string(T.seam),specs(is).name);
    fprintf('%s: Forward %d/%d, cascade %d/%d\n',specs(is).name, ...
        sum(T.forwardPass(idx)),sum(idx),sum(T.cascadePass(idx)),sum(idx));
end
fprintf('Saved: %s\n',options.csvFile);
end

function specs=localSpecsFromJson(file)
D=jsondecode(fileread(file)); p=D.workpiece.collision_proxies;
if iscell(p), p=[p{:}]; end
names=string({p.name});
required=["WorkpieceCollision_bottom","WorkpieceCollision_wall_x0", ...
    "WorkpieceCollision_wall_x1","WorkpieceCollision_wall_y1"];
if any(~ismember(required,names)), error('Missing Workpiece collision proxy names.'); end
b=localProxy(p,"WorkpieceCollision_bottom"); x0=localProxy(p,"WorkpieceCollision_wall_x0");
x1=localProxy(p,"WorkpieceCollision_wall_x1"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bottomMin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2;
bottomMax=double(b.pose_workpiece.position(:))+double(b.size_m(:))/2;
z=bottomMax(3); yMax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2;
specs=repmat(localSpec(),1,2);
specs(1)=localMake("bottom_x_wall_x0","Seam B",x0,+1,-1,1, ...
    double(x0.pose_workpiece.position(1))+double(x0.size_m(1))/2, ...
    [-0.7071067811865475;0;-0.7071067811865475], ...
    "U-interior: x > wall_x0 inner face, z > bottom top face",bottomMin,bottomMax,yMax,z);
specs(2)=localMake("bottom_x_wall_x1","Seam C",x1,-1,+1,1, ...
    double(x1.pose_workpiece.position(1))-double(x1.size_m(1))/2, ...
    [0.7071067811865475;0;-0.7071067811865475], ...
    "U-interior: x < wall_x1 inner face, z > bottom top face",bottomMin,bottomMax,yMax,z);
end

function s=localMake(name,shortName,wall,freeDir,axisSign,axis,coord,g,desc,bmin,bmax,ymax,z)
s=localSpec(); s.name=name; s.shortName=shortName; s.wallAxis=axis;
s.wallInnerCoordinate=coord; s.freeDirection=freeDir; s.gunAxisWP=g/norm(g);
s.description=desc; s.wallProxy=string(wall.name); s.wallPosition=double(wall.pose_workpiece.position(:)).';
s.wallSize=double(wall.size_m(:)).'; s.bottomSize=double(bmax-bmin).';
s.tangentWP=[0 1 0]; s.lengthMm=(ymax-bmin(2))*1000; s.axisSign=axisSign; s.bottomTopZ=z;
end

function s=localSpec()
s=struct('name',"",'shortName',"",'wallAxis',1,'wallInnerCoordinate',NaN, ...
    'freeDirection',1,'gunAxisWP',zeros(3,1),'description',"",'wallProxy',"", ...
    'wallPosition',zeros(1,3),'wallSize',zeros(1,3),'bottomSize',zeros(1,3), ...
    'tangentWP',zeros(1,3),'lengthMm',NaN,'axisSign',1,'bottomTopZ',NaN);
end

function P=localInitialAnchors(spec,jsonFile,N,margin)
D=jsondecode(fileread(jsonFile)); p=D.workpiece.collision_proxies; if iscell(p), p=[p{:}]; end
b=localProxy(p,"WorkpieceCollision_bottom"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bmin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2;
z=max(double(b.pose_workpiece.position(3))+double(b.size_m(3))/2);
ymax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2;
y0=bmin(2)+margin; yL=ymax-margin;
if yL<=y0, error('%s: margin too large.',spec.name); end
x=spec.wallInnerCoordinate;
P=[repmat(x,N,1),linspace(y0,yL,N).',repmat(z,N,1)];
end

function o=localPlannerOptions(spec,seed,file,options,positions)
o=struct('initialPoints',options.initialPoints,'seamSpec',spec, ...
    'initialSeamPositionsWP',positions,'maxRefinementRounds',4,'maxPoints',40, ...
    'maxSegmentTcpErrorMm',0.5,'maxIntermediatePoints',4,'endMarginM',options.marginMm/1000, ...
    'randomSeed',seed,'enablePostRecovery',true,'outputFile',file, ...
    'roundOutputPrefix',fullfile(options.outputDir,sprintf('%s_seed%d_round',spec.shortName,seed)), ...
    'animate',false,'publish',false);
end

function x=localProxy(p,name)
idx=find(strcmp(string({p.name}),name),1); if isempty(idx), error('Missing proxy %s',name); end; x=p(idx);
end
function r=localRow()
r=struct('seam',"",'seed',NaN,'forwardPass',false,'reverseTried',false,'reversePass',false,'cascadePass',false, ...
    'forwardTimeSec',NaN,'reverseTimeSec',NaN,'cascadeTimeSec',NaN,'forwardFinalPoints',NaN,'reverseFinalPoints',NaN, ...
    'forwardTcpErrorMm',NaN,'reverseTcpErrorMm',NaN,'forwardMaxJumpDeg',NaN,'reverseMaxJumpDeg',NaN,'failureStage',"");
end
function v=localNum(s,n,d), if isfield(s,n)&&~isempty(s.(n)),v=double(s.(n));else,v=d;end,end
function v=localFinalTcp(s), v=NaN; if isfield(s,'finalTcpMetrics')&&isfield(s.finalTcpMetrics,'segmentPositionErrorMm')&&~isempty(s.finalTcpMetrics.segmentPositionErrorMm),v=max(s.finalTcpMetrics.segmentPositionErrorMm);end,end
function stage=localFailureStage(s), stage="failed"; if isfield(s,'postRecoveryUsed')&&s.postRecoveryUsed,stage="post_recovery";end; if isfield(s,'refinementRounds')&&~isempty(s.refinementRounds),rr=s.refinementRounds;for k=numel(rr):-1:1,if isfield(rr(k),'recovery')&&strlength(string(rr(k).recovery))>0,stage=string(rr(k).recovery);return;end,end,end,end
function out=localMerge(d,c),out=d;f=fieldnames(c);for k=1:numel(f),out.(f{k})=c.(f{k});end,end
function y=ternary(c,a,b),if c,y=a;else,y=b;end,end
