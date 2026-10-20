function summary=run_frozen_accp_repeats(source,out,cacheEnabled,n)
% Cold solver/scene per iteration. Exact same saved Home/seed/options.
root='/home/ttbstt/jixiebi'; here=fullfile(root,'UR5','matlab仿真'); addpath(here);
addpath(genpath(fullfile(root,'RRT')));
if ~cacheEnabled
    % Diagnostic shadow differs only by Enabled=false: retain the independent
    % immutable-limit hoist, so this isolates memoization from that speedup.
    addpath(fullfile(fileparts(mfilename('fullpath')),'disabled_cache'),'-begin');
end
if ~isfolder(out), mkdir(out); end
s=load(source,'result'); options=s.result.options; clear s;
for k=1:numel(options.performanceBinding.collision_rule_sources)
    src=options.performanceBinding.collision_rule_sources{k};
    src.sha256=hmi_workflow_file_hash(src.path); options.performanceBinding.collision_rule_sources{k}=src;
end
options.performanceExactQueryCache=true; options.deferTimeParameterization=true;
options.initialStateBank=[]; options.publish=false; options.animate=false;
summary=struct('source',source,'exact_cache_enabled',cacheEnabled,'seed',options.randomSeed,...
    'frozen_scene_script',options.sceneScript,'runs',{{}},'executionReady',false,'robot_motion','NO');
for k=1:n
    run=fullfile(out,sprintf('run_%02d',k)); assert(~isfolder(run),'Refuse overwrite'); mkdir(run);
    options.outputFile=fullfile(run,'preview_accp.mat');
    diary(fullfile(run,'matlab.log')); clock=tic;
    try
        r=replan_weld_actual_axis_adaptive_core(options);
        entry=struct('iteration',k,'wall_sec',toc(clock),'configuration_complete',r.configurationComplete,...
            'geometry_complete',r.adaptiveCorrectionComplete,'nodes',size(r.qPathExecution,1),...
            'sparse_nodes',size(r.qPathInitial,1),'node_collision_free',r.nodeCollisionFree,...
            'edge_collision_free',r.edgeCollisionFree,'failed_refinement_edges',find(~[r.adaptiveSegmentReport.correctionAccepted]),...
            'max_tcp_error_mm',r.tcpMetrics.maxSegmentPositionErrorMm,...
            'ik_calls',r.continuationStats.ikCalls,'configuration_sec',r.configurationSearchTimeSec,...
            'refinement_sec',r.precisionAlgorithmTimeSec,'queries',r.configurationPerformanceQueries,...
            'executionReady',false);
        if r.adaptiveCorrectionComplete, entry.classification='PASS';
        elseif r.configurationComplete, entry.classification='PRECISION_FAILURE';
        else, entry.classification='NO_CONNECTED_CANDIDATE'; end
        entry.sparse_sha=hmi_workflow_file_hash(options.outputFile); % whole artifact identity, not numeric hash
        save(fullfile(run,'numeric_comparison.mat'),'-struct','r','qPathInitial','qPathExecution','tcpPosePath','-v7');
    catch err
        entry=struct('iteration',k,'wall_sec',toc(clock),'classification','EXCEPTION',...
            'failure_identifier',err.identifier,'failure_reason',err.message,'executionReady',false);
    end
    summary.runs{end+1}=entry;
    f=fopen(fullfile(out,'summary.json'),'w'); fwrite(f,jsonencode(summary,'PrettyPrint',true),'char'); fclose(f);
    fprintf('FROZEN_REPEAT_DONE cache=%d run=%d sec=%.3f classification=%s\n',cacheEnabled,k,entry.wall_sec,entry.classification);
    diary off;
end
end
