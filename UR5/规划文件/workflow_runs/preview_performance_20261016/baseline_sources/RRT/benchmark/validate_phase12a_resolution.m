function report = validate_phase12a_resolution(options)
%validate_phase12a_resolution Recheck saved Phase 12A paths only.
%
% No planner is called here.  Each persisted qPath is checked with the same
% UR5 scene at 2, 1, and 0.5 degree joint-edge interpolation resolutions.

if nargin < 1 || isempty(options), options=struct(); end
here=fileparts(mfilename('fullpath'));
root=fileparts(here);
defaults=struct('inputFile',fullfile(root,'results','phase12a_functional.mat'), ...
    'outputFile',fullfile(root,'results','phase12a_resolution_sensitivity.mat'), ...
    'csvFile',fullfile(root,'results','phase12a_resolution_sensitivity.csv'), ...
    'wideCsvFile',fullfile(root,'results','phase12a_resolution_sensitivity_wide.csv'), ...
    'resolutionsDeg',[2 1 0.5],'saveResult',true);
options=mergeOptions(defaults,options);
addpath(genpath(root));
s=load(options.inputFile,'report');
source=s.report;
if ~isfield(source,'paths') || numel(source.paths)~=numel(source.rows)
    error(['The input report has no persisted qPaths. Re-run the exact ', ...
        'Phase 12A-1 cases once with the updated runner before sensitivity checking.']);
end
scene=loadUR5JointScene(source.sceneScript);

rows=repmat(emptyRow(),0,1);
for i=1:numel(source.rows)
    qPath=source.paths{i};
    for r=1:numel(options.resolutionsDeg)
        row=emptyRow(); row.caseID=source.rows(i).caseID;
        row.algorithm=source.rows(i).algorithm;
        row.seed=source.rows(i).seed;
        if isfield(source,'plannerResolutionDeg')
            row.plannerResolutionDeg=source.plannerResolutionDeg;
        elseif isfield(source,'options') && isfield(source.options,'jointEdgeResolutionRad')
            row.plannerResolutionDeg=rad2deg(source.options.jointEdgeResolutionRad);
        end
        row.resolutionDeg=options.resolutionsDeg(r);
        row.pathStateCount=size(qPath,1);
        row.endpointValid=~isempty(qPath) && size(qPath,2)==6;
        row.nodeLimitValid=row.endpointValid && all(arrayfun(@(k) ...
            jointWithinLimits(qPath(k,:),scene.jointLimits),1:size(qPath,1)));
        row.edgeCount=max(0,size(qPath,1)-1);
        row.collisionChecks=0; row.collisionFree=true;
        opts=struct('wrapJoints',true);
        opts.selfCollisionSkipPairs=scene.selfCollisionSkipPairs;
        if row.endpointValid
            for k=1:size(qPath,1)-1
                [free,collisionIndex,count]=collisionFreeJointEdge(scene.robot,qPath(k,:), ...
                    qPath(k+1,:),scene.environment,deg2rad(row.resolutionDeg),opts);
                row.collisionChecks=row.collisionChecks+count;
                if ~free
                    row.collisionFree=false;
                    row.failureEdgeIndex=k;
                    row.failureSampleIndex=collisionIndex;
                    row.failureEdgeStartQ=qPath(k,:);
                    row.failureEdgeEndQ=qPath(k+1,:);
                    dq=atan2(sin(qPath(k+1,:)-qPath(k,:)), ...
                        cos(qPath(k+1,:)-qPath(k,:)));
                    nseg=max(1,ceil(max(abs(dq))/deg2rad(row.resolutionDeg)));
                    qFailure=qPath(k,:)+((collisionIndex-1)/nseg)*dq;
                    [~,detail]=collisionFreeJointState(scene.robot,qFailure, ...
                        scene.environment,scene.selfCollisionSkipPairs);
                    row.environmentCollision=detail.environmentCollision;
                    row.selfCollision=detail.selfCollision;
                    if detail.environmentCollision && detail.selfCollision
                        row.failureCollisionType="environment+self";
                    elseif detail.environmentCollision
                        row.failureCollisionType="environment";
                    elseif detail.selfCollision
                        row.failureCollisionType="self";
                    else
                        row.failureCollisionType="unknown";
                    end
                    row.failureConfigurationQ=qFailure;
                    break;
                end
            end
        else
            row.collisionFree=false;
        end
        row.pass=row.endpointValid && row.nodeLimitValid && row.collisionFree;
        rows(end+1,1)=row; %#ok<AGROW>
    end
end

report=struct('created',string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'inputFile',options.inputFile,'resolutionsDeg',options.resolutionsDeg, ...
    'rows',rows,'wideRows',makeWideRows(rows,options.resolutionsDeg), ...
    'allPass',all([rows.pass]), ...
    'note','No planner was called; only persisted Phase 12A paths were rechecked.');
if options.saveResult
    save(options.outputFile,'report','-v7.3');
    writeRows(rows,options.csvFile);
    writeWideRows(report.wideRows,options.wideCsvFile,options.resolutionsDeg);
end
end

function row=emptyRow()
row=struct('caseID',"",'algorithm',"",'seed',NaN,'plannerResolutionDeg',NaN, ...
    'resolutionDeg',NaN, ...
    'pathStateCount',0,'endpointValid',false,'nodeLimitValid',false, ...
    'edgeCount',0,'collisionChecks',0,'collisionFree',false,'pass',false, ...
    'failureEdgeIndex',NaN,'failureSampleIndex',NaN, ...
    'environmentCollision',false,'selfCollision',false, ...
    'failureCollisionType',"",'failureEdgeStartQ',zeros(1,6), ...
    'failureEdgeEndQ',zeros(1,6),'failureConfigurationQ',zeros(1,6));
end
function out=mergeOptions(defaults,in)
out=defaults; names=fieldnames(defaults);
for i=1:numel(names)
    if isfield(in,names{i}) && ~isempty(in.(names{i})), out.(names{i})=in.(names{i}); end
end
end
function writeRows(rows,file)
fid=fopen(file,'w'); if fid<0, error('Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s', ['caseID,algorithm,seed,plannerResolutionDeg,resolutionDeg,', ...
    'pathStateCount,endpointValid,', ...
    'nodeLimitValid,edgeCount,collisionChecks,collisionFree,pass,', ...
    'failureEdgeIndex,failureSampleIndex,environmentCollision,selfCollision',char(10)]);
for i=1:numel(rows)
    x=rows(i);
    fprintf(fid,'"%s","%s",%d,%.9g,%.9g,%d,%d,%d,%d,%d,%d,%d,%g,%g,%d,%d%s', ...
        x.caseID,x.algorithm,x.seed,x.plannerResolutionDeg,x.resolutionDeg,x.pathStateCount, ...
        x.endpointValid,x.nodeLimitValid,x.edgeCount,x.collisionChecks, ...
        x.collisionFree,x.pass,x.failureEdgeIndex,x.failureSampleIndex, ...
        x.environmentCollision,x.selfCollision,char(10));
end
end

function wide=makeWideRows(rows,resolutionsDeg)
keys=repmat(struct('caseID',"",'algorithm',"",'seed',NaN),0,1);
for i=1:numel(rows)
    exists=arrayfun(@(k) string(k.caseID)==rows(i).caseID && ...
        string(k.algorithm)==rows(i).algorithm && k.seed==rows(i).seed,keys);
    if ~any(exists)
        keys(end+1,1)=struct('caseID',rows(i).caseID,'algorithm',rows(i).algorithm, ...
            'seed',rows(i).seed); %#ok<AGROW>
    end
end
wide=repmat(wideRowTemplate(resolutionsDeg),0,1);
for i=1:numel(keys)
    x=wideRowTemplate(resolutionsDeg);
    x.caseID=keys(i).caseID; x.algorithm=keys(i).algorithm; x.seed=keys(i).seed;
    for r=1:numel(resolutionsDeg)
        idx=find(arrayfun(@(z) string(z.caseID)==x.caseID && ...
            string(z.algorithm)==x.algorithm && z.seed==x.seed && ...
            z.resolutionDeg==resolutionsDeg(r),rows),1);
        if isempty(idx)
            x.status(r)="MISSING";
        elseif rows(idx).pass
            x.status(r)="PASS";
        else
            x.status(r)="FAIL";
        end
    end
    wide(end+1,1)=x; %#ok<AGROW>
end
end

function row=wideRowTemplate(resolutionsDeg)
row=struct('caseID',"",'algorithm',"",'seed',NaN, ...
    'status',strings(1,numel(resolutionsDeg)));
end

function writeWideRows(rows,file,resolutionsDeg)
fid=fopen(file,'w'); if fid<0, error('Cannot write %s',file); end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'caseID,algorithm,seed');
for r=1:numel(resolutionsDeg)
    fprintf(fid,',%gdeg',resolutionsDeg(r));
end
fprintf(fid,'%s',char(10));
for i=1:numel(rows)
    x=rows(i);
    fprintf(fid,'"%s","%s",%d',x.caseID,x.algorithm,x.seed);
    for r=1:numel(resolutionsDeg)
        fprintf(fid,',%s',x.status(r));
    end
    fprintf(fid,'%s',char(10));
end
end
