function test_parallel_collision
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真')); addpath(genpath(fullfile(fileparts(root),'RRT'))); addpath(fullfile(here,'copies'));
request=jsondecode(fileread(fullfile(fileparts(here),'prepare_20261019_152229_336637','request.json')));
raw=load(request.rawFile); z=load(raw.sceneFile,'scene'); scene=z.scene;
r=struct('license_available',license('test','Distrib_Computing_Toolbox'),'pass',false,'executionReady',false);
if r.license_available
    rng(183,'twister'); lim=scene.jointLimits; Q=lim(:,1).'+rand(80,6).*(lim(:,2)-lim(:,1)).';
    serial=zeros(80,3); parallel=serial;
    for k=1:80
        [~,g,e,s]=combined_collision_copy(scene,Q(k,:)); serial(k,:)=[g e s];
    end
    clock=tic; pool=parpool('Processes',4); r.startup_sec=toc(clock);
    cleanup=onCleanup(@()delete(pool));
    clock=tic;
    parfor k=1:80
        [~,g,e,s]=combined_collision_copy(scene,Q(k,:)); parallel(k,:)=[g e s];
    end
    r.query_sec=toc(clock); r.pass=isequaln(serial,parallel); r.workers=pool.NumWorkers;
end
fid=fopen(fullfile(here,'parallel_collision_support.json'),'w'); cleanupFile=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(r,'PrettyPrint',true),'char'); disp(r);
end
