function summary = benchmark_task_level_arc_vs_gao(options)
%BENCHMARK_TASK_LEVEL_ARC_VS_GAO
% Fair task-level first-feasible comparison on Seam A/B/C.
%
% A task is one complete weld seam.  A trial stops at the first trajectory
% that passes node, interpolated-edge, robot/gun/wire/environment/self-
% collision, TCP and joint-continuity checks.  A single run uses the 60 s
% maximum budget; PASS@10/20/30/60 s is derived from its first-feasible
% wall-clock time, so the planner is not rerun four times with different
% stopping budgets.  This is valid only because both planners use the same
% search parameters at every reported budget.
%
% The continuation side is the frozen Forward -> Reverse A-ARC-IK cascade.
% The Gao side is the independent Gao-inspired AR-RRT* implementation.  No
% prior qPath, checkpoint, state graph or result file is used as an input.
% This experiment does not modify ROS2, CoppeliaSim or the stable planner.

if nargin < 1 || isempty(options), options = struct(); end
% The shared scene-loader creates a figure even when planner animation is
% disabled.  Hide those figures during formal batch experiments and restore
% the caller's default when the benchmark returns.
oldFigureVisible = get(0,"DefaultFigureVisible");
set(0,"DefaultFigureVisible","off");
figureVisibilityCleanup = onCleanup(@()set(0,"DefaultFigureVisible",oldFigureVisible)); %#ok<NASGU>
localCloseHiddenFigures();
here = fileparts(mfilename("fullpath"));
defaults = struct( ...
    "seeds",20270901:20270930, ...
    "timeBudgetsSec",[10 20 30 60],"maxBudgetSec",60, ...
    "runContinuation",true,"runGao",true,"resume",true, ...
    "outputDir",fullfile(here,"task_level_arc_vs_gao_runs"), ...
    "csvFile",fullfile(here,"task_level_arc_vs_gao.csv"), ...
    "matFile",fullfile(here,"task_level_arc_vs_gao.mat"));
options = localMerge(defaults,options);
options.seeds = double(options.seeds(:).');
if ~isabsolute(options.outputDir)
    options.outputDir = fullfile(here,options.outputDir);
end
if ~isabsolute(options.csvFile)
    options.csvFile = fullfile(here,options.csvFile);
end
if ~isabsolute(options.matFile)
    options.matFile = fullfile(here,options.matFile);
end
if ~isfolder(options.outputDir), mkdir(options.outputDir); end
specs = localSpecsFromJson(fullfile(fileparts(here),"scene_geometry.json"));

rows = repmat(localRow(),0,1);
fprintf("\n============================================================\n");
fprintf("TASK-LEVEL FIRST-FEASIBLE: A-ARC-IK vs Gao-AR-RRT*\n");
fprintf("Tasks per seam=%d, max wall budget=%.1f s, report budgets=[%s]\n", ...
    numel(options.seeds),options.maxBudgetSec,sprintf("%.0f ",options.timeBudgetsSec));
fprintf("PASS@budget is derived from first-feasible wall time of one max-budget run.\n");

for is = 1:numel(specs)
    spec = specs(is);
    anchors = localInitialAnchors(spec,fullfile(fileparts(here),"scene_geometry.json"),16,0.025);
    fprintf("\n--- %s: length %.1f mm ---\n",spec.name,norm(anchors(end,:)-anchors(1,:))*1000);
    for it = 1:numel(options.seeds)
        taskSeed = options.seeds(it);
        if options.runContinuation
            file = fullfile(options.outputDir,sprintf("%s_arc_%03d.mat",spec.shortName,it));
            if options.resume && isfile(file)
                A = load(file,"row");
                if isfield(A,"row")
                    cached=A.row;
                    if strcmp(string(cached.algorithm),"A-ARC-IK") && strlength(string(cached.selectedDirection))>0
                        rf=fullfile(options.outputDir,sprintf("%s_arc_%03d_%s.mat",spec.shortName,it,char(cached.selectedDirection)));
                        if isfile(rf), Q=load(rf,"result"); if isfield(Q,"result"), cached=edgeMetrics(Q.result,cached); end, end
                    end
                    rows(end+1,1)=cached; %#ok<AGROW>
                    fprintf("[%s ARC %02d] cached %s %.3fs\n",spec.shortName,it, ...
                        ternary(A.row.passWithinBudget,"PASS","FAIL"),A.row.tffWallSec);
                else
                    r = localRunArc(spec,anchors,taskSeed,options,file,it);
                    rows(end+1,1)=r; %#ok<AGROW>
                end
            else
                r = localRunArc(spec,anchors,taskSeed,options,file,it);
                rows(end+1,1)=r; %#ok<AGROW>
            end
        end
        if options.runGao
            file = fullfile(options.outputDir,sprintf("%s_gao_%03d.mat",spec.shortName,it));
            if options.resume && isfile(file)
                A = load(file,"row");
                if isfield(A,"row")
                    cached=A.row;
                    if isfield(A,"r"), cached=edgeMetrics(A.r,cached); end
                    rows(end+1,1)=cached; %#ok<AGROW>
                    fprintf("[%s Gao %02d] cached %s %.3fs\n",spec.shortName,it, ...
                        ternary(A.row.passWithinBudget,"PASS","FAIL"),A.row.tffWallSec);
                else
                    r = localRunGao(spec,taskSeed,options,file,it);
                    rows(end+1,1)=r; %#ok<AGROW>
                end
            else
                r = localRunGao(spec,taskSeed,options,file,it);
                rows(end+1,1)=r; %#ok<AGROW>
            end
        end
    end
end

if isempty(rows), error("没有产生任何实验行。"); end
T = struct2table(rows);
writetable(T,options.csvFile);
summary = struct(); summary.rows = rows; summary.specs = specs;
summary.options = options; summary.csvFile = options.csvFile;
summary.created = char(datetime("now"));
summary.curves = localCurves(rows,specs,options.timeBudgetsSec);
save(options.matFile,"summary","-v7.3");

fprintf("\n================ FAIR SUMMARY ================\n");
for is=1:numel(specs)
    for ia=1:2
        alg = ternary(ia==1,"A-ARC-IK","Gao-AR-RRT*");
        m = strcmp(string({rows.seam}),specs(is).name) & strcmp(string({rows.algorithm}),alg);
        if ~any(m), continue; end
        t=[rows(m).tffWallSec]; pass=[rows(m).passWithinBudget];
        t=t(isfinite(t) & pass);
        fprintf("%s | %-12s |",specs(is).shortName,alg);
        for b=1:numel(options.timeBudgetsSec)
            fprintf(" %g s: %d/%d (%.1f%%) |",options.timeBudgetsSec(b), ...
                sum([rows(m).complete] & [rows(m).tffWallSec]<=options.timeBudgetsSec(b)),sum(m), ...
                100*sum([rows(m).complete] & [rows(m).tffWallSec]<=options.timeBudgetsSec(b))/sum(m));
        end
        fprintf(" median=%.3fs P90=%.3fs\n",localPercentile(t,50),localPercentile(t,90));
    end
end
fprintf("Saved CSV: %s\nSaved MAT: %s\n",options.csvFile,options.matFile);
end

function row = localRunArc(spec,anchors,taskSeed,options,file,it)
row=localRow(); row.seam=spec.name; row.algorithm="A-ARC-IK"; row.trialIndex=it; row.taskSeed=taskSeed;
tTask=tic; selected=[]; direction=""; totalIk=0; totalWall=0; totalAlg=0; last=[];
for id=1:2
    if toc(tTask)>options.maxBudgetSec, break; end
    if id==1, pos=anchors; direction="forward"; else, pos=flipud(anchors); direction="reverse"; end
    out=fullfile(options.outputDir,sprintf("%s_arc_%03d_%s.mat",spec.shortName,it,direction));
    o=struct("initialPoints",size(pos,1),"initialSeamPositionsWP",pos, ...
        "seamSpec",spec,"endMarginM",0.025,"maxRefinementRounds",4, ...
        "maxPoints",40,"maxSegmentTcpErrorMm",0.5,"maxIntermediatePoints",4, ...
        "randomSeed",taskSeed,"enablePostRecovery",true,"outputFile",out, ...
        "roundOutputPrefix",fullfile(options.outputDir,sprintf("%s_arc_%03d_%s_round",spec.shortName,it,direction)), ...
        "animate",false,"publish",false);
    fprintf("[%s ARC %02d] %s ... ",spec.shortName,it,upper(direction));
    t=tic; last=replan_weld_seam_generalization_feasibility(o); w=toc(t); totalWall=totalWall+w;
    localCloseHiddenFigures();
    totalAlg=totalAlg+localAlg(last); totalIk=totalIk+localIk(last);
    ok=localPass(last);
    fprintf("%s %.3fs\n",ternary(ok,"PASS","FAIL"),w);
    if ok && toc(tTask)<=options.maxBudgetSec
        selected=last; row.tffWallSec=toc(tTask); break;
    end
    if ok && toc(tTask)>options.maxBudgetSec, row.latePass=true; end
end
row.complete=~isempty(selected) && localPass(selected);
row.passWithinBudget=row.complete && row.tffWallSec<=options.maxBudgetSec;
row.selectedDirection=string(direction); row.wallSec=totalWall; row.algorithmSec=totalAlg; row.ikCalls=totalIk;
row=edgeMetrics(selected,row); row.outputFile=string(file);
save(file,"row","-v7.3");
fprintf("[%s ARC %02d] => %s, TFF=%.3fs\n",spec.shortName,it, ...
    ternary(row.passWithinBudget,"PASS","FAIL"),row.tffWallSec);
end

function row = localRunGao(spec,taskSeed,options,file,it)
row=localRow(); row.seam=spec.name; row.algorithm="Gao-AR-RRT*"; row.trialIndex=it; row.taskSeed=taskSeed;
here=fileparts(mfilename("fullpath"));
relDir=erase(string(options.outputDir),string([here filesep]));
if relDir==string(options.outputDir), relDir="task_level_arc_vs_gao_runs"; end
o=struct("seamSpec",spec,"numPoints",80,"endMarginM",0.025, ...
    "timeBudgetSec",options.maxBudgetSec,"maxIterations",100000, ...
    "samplesPerLayer",12,"maxStatesPerLayer",12,"maxStartStates",8, ...
    "maxIKSeeds",8,"neighborParents",8,"goalBias",0.20, ...
    "animate",false,"outputFile",fullfile(relDir,sprintf("%s_gao_%03d_result.mat",spec.shortName,it)), ...
    "randomSeed",taskSeed);
fprintf("[%s Gao %02d] RUN ... ",spec.shortName,it);
t=tic; r=replan_weld_actual_axis_gao_ar_rrtstar(o); wall=toc(t);
localCloseHiddenFigures();
fprintf("%s %.3fs\n",ternary(localPass(r),"PASS","FAIL"),wall);
row.complete=localPass(r); row.passWithinBudget=row.complete && wall<=options.maxBudgetSec;
row.tffWallSec=ternary(row.passWithinBudget,wall,NaN); row.wallSec=wall; row.algorithmSec=r.algorithmElapsedSec;
row.ikCalls=localGetStats(r,"ikCalls"); row.edgeTests=localGetStats(r,"edgeTests");
row.maxJointJumpDeg=localGet(r,"maxJointJumpDeg",NaN); row.maxNodeTcpErrorMm=localTcp(r,"maxNodePositionErrorMm");
row.maxSegmentTcpErrorMm=localTcp(r,"maxSegmentPositionErrorMm"); row.nodeCollisionFree=logical(localGet(r,"nodeCollisionFree",false));
row.edgeCollisionFree=logical(localGet(r,"edgeCollisionFree",false)); row.outputFile=string(file);
save(file,"row","r","-v7.3");
end

function tf=localPass(r)
tf=isstruct(r) && isfield(r,"complete") && logical(r.complete) && ...
    localGet(r,"nodeCollisionFree",false) && localGet(r,"edgeCollisionFree",false);
end
function t=localAlg(r), t=localGet(r,"algorithmElapsedSec",localGet(r,"feasibilityAdaptiveWallTimeSec",NaN)); end
function n=localIk(r)
if isfield(r,"continuationStats"), n=localGet(r.continuationStats,"ikCalls",NaN); elseif isfield(r,"stats"), n=localGet(r.stats,"ikCalls",NaN); else, n=NaN; end
end
function n=localGetStats(r,f), if isfield(r,"stats"), n=localGet(r.stats,f,NaN); else, n=NaN; end, end
function x=localTcp(r,f)
x=NaN;
if isfield(r,"finalTcpMetrics")
    m=r.finalTcpMetrics;
    if strcmp(f,"maxSegmentPositionErrorMm") && isfield(m,"segmentPositionErrorMm"), x=max(m.segmentPositionErrorMm); return; end
    if strcmp(f,"maxNodePositionErrorMm") && isfield(m,"nodePositionErrorMm"), x=max(m.nodePositionErrorMm); return; end
    if isfield(m,f), x=m.(f); return; end
end
if isfield(r,"tcpMetrics") && isfield(r.tcpMetrics,f), x=r.tcpMetrics.(f); end
end
function row=edgeMetrics(r,row)
if isempty(r), return; end
row.complete=localPass(r); row.nodeCollisionFree=localGet(r,"nodeCollisionFree",false); row.edgeCollisionFree=localGet(r,"edgeCollisionFree",false);
row.maxJointJumpDeg=localGet(r,"maxJointJumpDeg",NaN); row.maxNodeTcpErrorMm=localTcp(r,"maxNodePositionErrorMm"); row.maxSegmentTcpErrorMm=localTcp(r,"maxSegmentPositionErrorMm");
end

function c=localCurves(rows,specs,budgets)
c=repmat(struct("seam","","algorithm","","budgetSec",NaN,"passCount",0,"total",0,"rate",0),0,1);
for i=1:numel(specs)
    algs=["A-ARC-IK","Gao-AR-RRT*"];
    for ia=1:numel(algs)
        a=algs(ia);
        m=strcmp(string({rows.seam}),specs(i).name) & strcmp(string({rows.algorithm}),a);
        for ib=1:numel(budgets)
            b=budgets(ib); z=struct(); z.seam=specs(i).name; z.algorithm=a;
            z.budgetSec=b; z.total=sum(m); z.passCount=sum(m & [rows.complete] & [rows.tffWallSec]<=b);
            z.rate=z.passCount/max(1,z.total); c(end+1)=z; %#ok<AGROW>
        end
    end
end
end

function row=localRow()
row=struct("seam","","algorithm","","trialIndex",NaN,"taskSeed",NaN,"complete",false,"passWithinBudget",false,"latePass",false,"tffWallSec",NaN,"wallSec",NaN,"algorithmSec",NaN,"selectedDirection","","ikCalls",NaN,"edgeTests",NaN,"nodeCollisionFree",false,"edgeCollisionFree",false,"maxJointJumpDeg",NaN,"maxNodeTcpErrorMm",NaN,"maxSegmentTcpErrorMm",NaN,"outputFile","");
end
function x=localGet(s,f,d), if isfield(s,f), x=s.(f); else, x=d; end, end
function out=localMerge(d,u), out=d; f=fieldnames(u); for k=1:numel(f), out.(f{k})=u.(f{k}); end, end
function localCloseHiddenFigures()
f=findall(0,"Type","figure","Visible","off");
if ~isempty(f), close(f); end
end
function y=ternary(tf,a,b), if tf,y=a;else,y=b;end,end
function p=localPercentile(x,q)
x=x(isfinite(x));
if isempty(x), p=NaN; return; end
x=sort(x(:)); idx=1+round((numel(x)-1)*q/100); idx=max(1,min(numel(x),idx)); p=x(idx);
end
function tf=isabsolute(p), s=string(p); tf=startsWith(s,filesep) || (strlength(s)>=2 && extractBetween(s,2,2)==":"); end

function specs=localSpecsFromJson(file)
D=jsondecode(fileread(file)); p=D.workpiece.collision_proxies; if iscell(p), p=[p{:}]; end
b=localProxy(p,"WorkpieceCollision_bottom"); x0=localProxy(p,"WorkpieceCollision_wall_x0"); x1=localProxy(p,"WorkpieceCollision_wall_x1"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bmin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2; bmax=double(b.pose_workpiece.position(:))+double(b.size_m(:))/2; ymax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2; z=bmax(3);
specs(1)=localSpec("bottom_x_wall_y1","Seam_A",2,double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2,-1,[0;1;-1]/sqrt(2),"U-interior: y < wall_y1 inner face, z > bottom top face");
specs(2)=localSpec("bottom_x_wall_x0","Seam_B",1,double(x0.pose_workpiece.position(1))+double(x0.size_m(1))/2,1,[-1;0;-1]/sqrt(2),"U-interior: x > wall_x0 inner face, z > bottom top face");
specs(3)=localSpec("bottom_x_wall_x1","Seam_C",1,double(x1.pose_workpiece.position(1))-double(x1.size_m(1))/2,-1,[1;0;-1]/sqrt(2),"U-interior: x < wall_x1 inner face, z > bottom top face");
for k=1:3, specs(k).bottomMin=bmin; specs(k).bottomMax=bmax; specs(k).yMax=ymax; specs(k).z=z; end
end
function s=localSpec(name,short,axis,coord,freeDir,g,desc), s=struct("name",name,"shortName",short,"wallAxis",axis,"wallInnerCoordinate",coord,"freeDirection",freeDir,"gunAxisWP",g/norm(g),"description",desc); end
function p=localProxy(a,n), p=a(strcmp(string({a.name}),n)); end
function A=localInitialAnchors(spec,jsonFile,N,margin)
D=jsondecode(fileread(jsonFile)); p=D.workpiece.collision_proxies; if iscell(p), p=[p{:}]; end; b=localProxy(p,"WorkpieceCollision_bottom"); x0=localProxy(p,"WorkpieceCollision_wall_x0"); x1=localProxy(p,"WorkpieceCollision_wall_x1"); y1=localProxy(p,"WorkpieceCollision_wall_y1");
bmin=double(b.pose_workpiece.position(:))-double(b.size_m(:))/2; z=double(b.pose_workpiece.position(3))+double(b.size_m(3))/2; ymax=double(y1.pose_workpiece.position(2))-double(y1.size_m(2))/2; A=zeros(N,3);
if spec.wallAxis==2, xmin=max(bmin(1),double(x0.pose_workpiece.position(1))+double(x0.size_m(1))/2)+margin; xmax=min(double(b.pose_workpiece.position(1))+double(b.size_m(1))/2,double(x1.pose_workpiece.position(1))-double(x1.size_m(1))/2)-margin; A(:,1)=linspace(xmin,xmax,N); A(:,2)=spec.wallInnerCoordinate; else, y0=bmin(2)+margin; y1v=ymax-margin; A(:,2)=linspace(y0,y1v,N); A(:,1)=spec.wallInnerCoordinate; end; A(:,3)=z;
end
