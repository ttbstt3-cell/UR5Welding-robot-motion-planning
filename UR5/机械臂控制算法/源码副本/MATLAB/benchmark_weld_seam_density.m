function summary = benchmark_weld_seam_density(options)
%BENCHMARK_WELD_SEAM_DENSITY Compare sparse cold-start seam discretizations.
%
% This experiment changes only options.numPoints passed to the existing
% continuation planner.  The planner itself is not modified.  Each run
% reconstructs the scene, creates a fresh IK solver/cache, and uses a fixed
% random seed.  The dense path metrics remain available for every run.

if nargin < 1 || isempty(options), options = struct(); end
defaults = struct( ...
    "pointCounts",[8 12 16 20 40 80], ...
    "runCount",5, ...
    "baseSeed",20267000, ...
    "outputDir","adaptive_density_runs", ...
    "summaryFile","adaptive_density_benchmark.mat", ...
    "csvFile","adaptive_density_benchmark.csv");
options = localMergeOptions(defaults, options);

matlabDir = fileparts(mfilename("fullpath"));
outDir = options.outputDir;
if isabsolute(outDir)
    if ~startsWith(outDir, matlabDir + filesep)
        error("outputDir 必须位于 MATLAB 仿真目录内。");
    end
    outDir = extractAfter(outDir, matlabDir + filesep);
end
outDirFull = fullfile(matlabDir, outDir);
if ~isfolder(outDirFull), mkdir(outDirFull); end

template = localEmptyRow();
rows = repmat(template, numel(options.pointCounts)*options.runCount, 1);
row = 0;

fprintf("\nADAPTIVE WELD SEAM DENSITY BENCHMARK\n");
fprintf("Point counts: [%s], runs per count: %d\n", ...
    sprintf("%d ", options.pointCounts), options.runCount);
fprintf("Only options.numPoints changes; continuation algorithm is unchanged.\n\n");

for ip = 1:numel(options.pointCounts)
    nPoints = options.pointCounts(ip);
    for ir = 1:options.runCount
        row = row + 1;
        % Use the same seed set for every point count so density groups are
        % paired cold-start comparisons rather than different populations.
        seed = options.baseSeed + ir;
        outputFile = fullfile(outDir, sprintf("continuation_n%d_r%02d.mat", nPoints, ir));
        % Match the stable fair-continuation profile; only numPoints
        % changes between density groups.
        plannerOptions = struct( ...
            "numPoints",nPoints, ...
            "animate",false, ...
            "publish",false, ...
            "enableFallback",false, ...
            "beamWidth",1, ...
            "firstPointMaxStates",2, ...
            "maxPoseSeeds",3, ...
            "maxPoseNeighbors",9, ...
            "numRandomSeedsFirstPoint",3, ...
            "coarseTravelDeg",0, ...
            "coarseRollDeg",0:45:315, ...
            "progressivePoseNeighbors",true, ...
            "progressiveInitialPoseNeighbors",5, ...
            "progressiveMaxPoseNeighbors",9, ...
            "randomSeed",seed, ...
            "outputFile",outputFile);
        fprintf("[%d/%d] numPoints=%d seed=%d\n", row, numel(rows), nPoints, seed);
        t = tic;
        result = replan_weld_actual_axis_continuation(plannerOptions);
        wall = toc(t);
        rows(row) = localRowFromResult(result, nPoints, seed, wall);
    end
end

rows = rows(1:row);
summary = struct();
summary.options = options;
summary.rows = rows;
summary.created = char(datetime("now"));
summary.outputDir = outDirFull;

summaryFile = options.summaryFile;
if ~isabsolute(summaryFile), summaryFile = fullfile(matlabDir, summaryFile); end
csvFile = options.csvFile;
if ~isabsolute(csvFile), csvFile = fullfile(matlabDir, csvFile); end
save(summaryFile, "summary", "-v7.3");
writetable(struct2table(rows), csvFile);

fprintf("\nDENSITY SUMMARY\n");
for ip = 1:numel(options.pointCounts)
    nPoints = options.pointCounts(ip);
    m = [rows.numPoints] == nPoints;
    pass = m & [rows.complete];
    times = [rows.algorithmTimeSec];
    times = times(pass & isfinite(times));
    if isempty(times)
        medTime = NaN; meanTime = NaN;
    else
        medTime = median(times); meanTime = mean(times);
    end
    ik = [rows.ikCalls]; ik = ik(m & isfinite(ik));
    seg = [rows.maxSegmentTcpErrorMm]; seg = seg(pass & isfinite(seg));
    fprintf("  %3d points: PASS %d/%d (%.1f%%), time median/mean %.4f/%.4f s, IK median %.1f, segment TCP max %.4f mm\n", ...
        nPoints, sum(pass), sum(m), 100*sum(pass)/max(1,sum(m)), ...
        medTime, meanTime, median(ik), max(seg));
end
fprintf("Saved MAT: %s\nSaved CSV: %s\n", summaryFile, csvFile);
end

function row = localEmptyRow()
row = struct( ...
    "numPoints",NaN,"seed",0,"algorithmTimeSec",NaN,"wallTimeSec",NaN, ...
    "complete",false,"nodeCollisionFree",false,"edgeCollisionFree",false, ...
    "layersWithStates",NaN,"furthestPoint",NaN,"ikCalls",NaN, ...
    "ikPoseSuccess",NaN,"ikSuccessRatePct",NaN,"edgeTests",NaN, ...
    "edgeCollisionFreeTests",NaN,"maxJointJumpDeg",NaN, ...
    "maxNodeTcpErrorMm",NaN,"maxSegmentTcpErrorMm",NaN, ...
    "maxSegmentTcpOrientationDeg",NaN,"fallbackUsed",false);
end

function row = localRowFromResult(r, numPoints, seed, wall)
row = localEmptyRow();
row.numPoints = numPoints;
row.seed = seed;
row.algorithmTimeSec = r.algorithmElapsedSec;
row.wallTimeSec = wall;
row.complete = logical(r.complete);
row.nodeCollisionFree = logical(r.nodeCollisionFree);
row.edgeCollisionFree = logical(r.edgeCollisionFree);
if isfield(r,"stateBank")
    nonempty = ~cellfun(@isempty, r.stateBank);
    row.layersWithStates = sum(nonempty);
    k = find(nonempty, 1, "last");
    if isempty(k), row.furthestPoint = 0; else, row.furthestPoint = k; end
end
if isfield(r,"continuationStats")
    s = r.continuationStats;
    row.ikCalls = localGet(s,"ikCalls",NaN);
    row.ikPoseSuccess = localGet(s,"ikPoseSuccess",NaN);
    row.edgeTests = localGet(s,"edgeTests",NaN);
    row.edgeCollisionFreeTests = localGet(s,"edgeCollisionFree",NaN);
end
row.ikSuccessRatePct = 100*row.ikPoseSuccess/max(1,row.ikCalls);
row.maxJointJumpDeg = localGet(r,"maxJointJumpDeg",NaN);
if isfield(r,"tcpMetrics")
    row.maxNodeTcpErrorMm = localGet(r.tcpMetrics,"maxNodePositionErrorMm",NaN);
    row.maxSegmentTcpErrorMm = localGet(r.tcpMetrics,"maxSegmentPositionErrorMm",NaN);
    row.maxSegmentTcpOrientationDeg = localGet(r.tcpMetrics,"maxSegmentOrientationErrorDeg",NaN);
end
row.fallbackUsed = localGet(r,"fallbackUsed",false);
end

function x = localGet(s, f, default)
if isfield(s,f), x = s.(f); else, x = default; end
end

function out = localMergeOptions(defaults, in)
out = defaults;
fields = fieldnames(defaults);
for k = 1:numel(fields)
    f = fields{k};
    if isfield(in,f) && ~isempty(in.(f)), out.(f) = in.(f); end
end
end

function tf = isabsolute(p)
p = string(p);
tf = startsWith(p, filesep) || ...
    (strlength(p) >= 2 && extractBetween(p,2,2) == ":");
end
