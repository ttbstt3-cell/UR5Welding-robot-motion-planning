function cancellation_fixture(out)
pool=parpool('Processes',4); cleanup=onCleanup(@()delete(pool)); %#ok<NASGU>
spmd, pid=feature('getpid'); end
workerPids=zeros(1,pool.NumWorkers);
for k=1:numel(workerPids), workerPids(k)=pid{k}; end
r=struct('parent_pid',feature('getpid'),'worker_pids',workerPids,'executionReady',false);
fid=fopen(fullfile(out,'owned_pids.json'),'w'); fwrite(fid,jsonencode(r),'char'); fclose(fid);
pause(180); % Offline cancellation fixture only; never a production wait.
end
