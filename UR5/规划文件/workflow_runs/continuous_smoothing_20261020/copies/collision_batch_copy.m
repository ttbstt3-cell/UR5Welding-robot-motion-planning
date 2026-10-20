function [envFlags,selfFlags,gaps,info]=collision_batch_copy(scene,Q,workers)
% Same exhaustive state coverage; no approximate keys/cross-request cache.
if nargin<3, workers=4; end
n=size(Q,1); envFlags=false(n,1); selfFlags=envFlags; gaps=nan(n,1);
info=struct('mode','serial_combined','workers',1,'startup_sec',0,'query_sec',0,...
 'processed_states',0,'combined_calls',0,'parallel_unavailable_reason','');
pool=[]; cleanup=[]; %#ok<NASGU>
if workers>1 && n>=5000
    if exist('parpool','file') && license('test','Distrib_Computing_Toolbox')
        existing=gcp('nocreate');
        if isempty(existing)
            clock=tic;
            try
                pool=parpool('Processes',workers);
                cleanup=onCleanup(@()delete(pool));
                info.mode='parallel_process_combined'; info.workers=pool.NumWorkers;
            catch e
                info.parallel_unavailable_reason=['Pool startup failed before any state: ' e.message];
            end
            info.startup_sec=toc(clock);
        else
            info.parallel_unavailable_reason='External pool retained unchanged; serial audit used';
        end
    else
        info.parallel_unavailable_reason='Parallel toolbox unavailable; serial audit used';
    end
end
clock=tic;
if ~isempty(pool)
    % Any worker error propagates. No partially completed audit is accepted.
    parfor k=1:n
        [~,g,e,s]=combined_collision_copy(scene,Q(k,:));
        envFlags(k)=e; selfFlags(k)=s; gaps(k)=g;
    end
else
    for k=1:n
        [~,g,e,s]=combined_collision_copy(scene,Q(k,:));
        envFlags(k)=e; selfFlags(k)=s; gaps(k)=g;
    end
end
info.query_sec=toc(clock); info.processed_states=n; info.combined_calls=n;
end
