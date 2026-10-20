function report=compare_accp_stability_runs(root)
groups={'cache_on','cache_off'}; base=[]; rows={};
for g=1:numel(groups)
    for k=1:5
        path=fullfile(root,groups{g},sprintf('run_%02d',k),'preview_accp.mat');
        s=load(path,'result'); r=s.result;
        if isempty(base), base=r; end
        opts=r.options; ref=base.options;
        opts=rmfield(opts,'outputFile'); ref=rmfield(ref,'outputFile');
        same=isequal(r.qPathInitial,base.qPathInitial) &&...
            isequal(r.cartesianRefinement.qPathExecution,base.cartesianRefinement.qPathExecution) &&...
            isequal(r.cartesianRefinement.tcpTargetExecution,base.cartesianRefinement.tcpTargetExecution);
        assert(same && isequal(opts,ref),'Frozen numeric outputs / options differ');
        assert(~r.adaptiveCorrectionComplete && r.configurationComplete && ~r.executionReady);
        assert(logical(r.configurationPerformanceQueries.enabled)==strcmp(groups{g},'cache_on'));
        first=r.candidateSelectionDiagnostics{1};
        assert(max(abs(deg2rad(first.q_realhome_deg(:))-r.scene.qMat0(:)))<1e-12,...
            'External Home file changed from frozen request');
        rows{end+1}=struct('group',groups{g},'run',k,'same_numeric_path_and_targets',same,...
            'same_options_except_output',isequal(opts,ref),'geometry_complete',r.adaptiveCorrectionComplete,...
            'exact_cache_enabled',r.configurationPerformanceQueries.enabled); %#ok<AGROW>
    end
end
report=struct('pass',true,'scope','10 cold runs; same saved scene/Home/seed/parameters; only exact memoization differs',...
    'rows',{rows},'executionReady',false,'robot_motion','NO');
f=fopen(fullfile(root,'frozen_cache_comparison.json'),'w'); c=onCleanup(@()fclose(f)); fwrite(f,jsonencode(report,'PrettyPrint',true),'char');
end
