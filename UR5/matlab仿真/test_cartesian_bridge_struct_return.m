function report=test_cartesian_bridge_struct_return(outputDir)
% Offline return-schema regression against the production bridge function.
% The private function is exposed only in a generated test-only source copy.
% Search/IK/collision source, options and saved REAL scene remain unchanged.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
if nargin<1
    outputDir=fullfile(root,'规划文件','workflow_runs','bridge_struct_fix_20261014');
end
if ~isfolder(outputDir), mkdir(outputDir); end
addpath(here); addpath(outputDir); setenv('RRT_HEADLESS','1');
coreFile=fullfile(here,'replan_weld_actual_axis_adaptive_core.m');
source=fileread(coreFile); coreHash=hmi_workflow_file_hash(coreFile);
before=hmi_planning_dependency_snapshot(root);
protectedHome=fullfile(root,'ur5_hmi','config','hmi_config.yaml');
homeHashBefore=hmi_workflow_file_hash(protectedHome);
recipe=fullfile(root,'规划文件','real_anchor_runs',...
    'accp_bridge_comparison_20261013_improved_full_cold_v2.mat');
sourceRun=fullfile(root,'规划文件','workflow_runs',...
    'realhome_update_regression_20261014_v2','hmi_current_home_preview');
requestFile=fullfile(sourceRun,'request.json');
sceneFile=fullfile(sourceRun,'raw_scene.mat');
request=jsondecode(fileread(requestFile));
old=load(recipe,'result'); options=old.result.options;
sceneScript=fullfile(outputDir,'saved_scene_test.m');
writeText(sceneScript,sprintf("s=load('%s','scene'); scene=s.scene;\n",strrep(sceneFile,"'","''")));
options.sceneScript=sceneScript; options.outputFile=fullfile(outputDir,'MUST_NOT_PUBLISH.mat');
options.publish=false; options.animate=false; options.randomSeed=request.seed;
options.initialStateBank=[]; options.initialStartLayer=1;
options.branchReferenceQ=reshape(request.qStart,1,6);
options.configurationOnlyDiagnostic=false; options.disableAxisConstraint=false;
options.disable_axis_constraint=false;
% A public wrapper plus an otherwise byte-identical production source exposes
% the existing private bridge implementation. No stubbed IK/collision calls.
wrapper=sprintf(['function [bridge,summary]=bridge_test_harness(scene,seam,pointIndex,prev,options,pools)\n'...
    '[bridge,summary]=localCartesianBridgeFallback(scene,seam,pointIndex,prev,options,pools);\nend\n']);
writeText(fullfile(outputDir,'bridge_test_harness.m'),[wrapper source]);
generated=fileread(fullfile(outputDir,'bridge_test_harness.m'));
assert(strcmp(generated(numel(wrapper)+1:end),source),...
    'Public test harness must contain byte-identical production core');
fixtureFile=fullfile(outputDir,'point14_input_fixture.mat');
needle=sprintf(['            [bridge,bridgeStats] = localCartesianBridgeFallback( ...\n'...
    '                scene,seam,i,stateBank{i-1},options,stepStats.diagnosticCandidatePools);']);
assert(numel(strfind(source,needle))==1,'Production fallback call is not uniquely identifiable');
observer=sprintf(['            pointIndex=i; prev=stateBank{i-1}; rng_state=rng;\n'...
    '            candidatePools=stepStats.diagnosticCandidatePools;\n'...
    '            save(''%s'',''scene'',''seam'',''pointIndex'',''prev'',''options'', ...\n'...
    '                ''candidatePools'',''stateBank'',''layerStats'',''stepStats'',''rng_state'',''-v7.3'');\n'...
    '            error(''TEST:BridgeInputCaptured'',''Captured real failure before private bridge call'');\n'],...
    strrep(fixtureFile,"'","''"));
captureSource=strrep(source,...
    'function result = replan_weld_actual_axis_adaptive_core(options)',...
    'function result = bridge_capture_core(options)');
captureSource=strrep(captureSource,needle,[observer needle]);
writeText(fullfile(outputDir,'bridge_capture_core.m'),captureSource);
rehash;
% Empty prior layer must return explicit failure rather than struct creation
% exception. It also verifies both formerly duplicated fields are present.
[emptyBridge,emptySummary]=bridge_test_harness(struct(),struct(),14,[],options,{});
assert(~emptyBridge.success && strcmp(emptyBridge.failure_reason,'NO_PREVIOUS_LAYER'));
assert(isempty(emptyBridge.paths) && emptyBridge.inserted_layer_count==0);
assert(isfield(emptyBridge,'selected_level') && isfield(emptyBridge,'selected_fractions'));
fprintf('BRIDGE_EMPTY_PREVIOUS_LAYER_RETURN_PASS\n');
diary(fullfile(outputDir,'point14_capture_and_replay.log'));
clock=tic;
captured=false;
try
    bridge_capture_core(options);
catch err
    if strcmp(err.identifier,'TEST:BridgeInputCaptured'), captured=true;
    else, rethrow(err); end
end
captureElapsed=toc(clock);
assert(captured && isfile(fixtureFile),'Current REAL snapshot did not reach a failed bridge layer');
input=load(fixtureFile);
assert(input.pointIndex==14,'Expected exact saved Point14 NO_CONNECTED_CANDIDATE');
assert(isempty(input.stateBank{14}) && ~isempty(input.prev));
assert(isequal(input.scene.qMat0,reshape(request.qStart,1,6)));
for name=fieldnames(options).'
    assert(isequaln(input.options.(name{1}),options.(name{1})),...
        'Captured production option differs from exact HMI recipe: %s',name{1});
end
fprintf('BRIDGE_POINT14_INPUT_CAPTURE_PASS elapsed=%.3f seconds\n',captureElapsed);
clock=tic;
rng(input.rng_state);
[bridge,summary]=bridge_test_harness(input.scene,input.seam,input.pointIndex,...
    input.prev,input.options,input.candidatePools);
replayElapsed=toc(clock);
assert(isstruct(bridge) && isscalar(bridge));
assert(islogical(bridge.success) && isscalar(bridge.success));
assert(isfield(bridge,'failure_reason') && isfield(bridge,'paths'));
insertedPoses={}; insertedQ=zeros(0,6);
for kp=1:numel(bridge.paths)
    states=bridge.paths(kp).intermediate_states;
    for ks=1:numel(states)
        assert(isequal(size(states(ks).Ttarget),[4 4]) && all(isfinite(states(ks).Ttarget(:))));
        assert(numel(states(ks).q)==6 && all(isfinite(states(ks).q)));
        insertedPoses{end+1,1}=states(ks).Ttarget; %#ok<AGROW>
        insertedQ(end+1,:)=reshape(states(ks).q,1,6); %#ok<AGROW>
    end
end
if bridge.success
    assert(strlength(string(bridge.failure_reason))==0 && ~isempty(insertedQ));
else
    assert(strlength(string(bridge.failure_reason))>0 && isempty(insertedQ));
end
after=hmi_planning_dependency_snapshot(root);
changes=hmi_planning_dependency_changes(before,after);
assert(changes.pass,'Protected scene dependencies changed during test');
assert(strcmp(homeHashBefore,hmi_workflow_file_hash(protectedHome)),'Saved Home changed');
assert(strcmp(coreHash,hmi_workflow_file_hash(coreFile)),'Production core changed during replay');
report=struct('test_status','PASS','point_index',input.pointIndex,...
    'trigger','NO_CONNECTED_CANDIDATE','bridge_success',bridge.success,...
    'failure_reason',bridge.failure_reason,'inserted_poses',{insertedPoses},...
    'inserted_q_nodes',insertedQ,'returned_path_count',numel(bridge.paths),...
    'capture_elapsed_sec',captureElapsed,'bridge_elapsed_sec',replayElapsed,...
    'bridge',bridge,'summary',summary,'empty_previous_layer',emptyBridge,...
    'empty_previous_layer_summary',emptySummary,'production_core_sha256',coreHash,...
    'request_sha256',hmi_workflow_file_hash(requestFile),...
    'scene_snapshot_sha256',hmi_workflow_file_hash(sceneFile),...
    'recipe_sha256',hmi_workflow_file_hash(recipe),...
    'fixture_file',fixtureFile,'protected_scene_dependencies',before,...
    'protected_dependencies_unchanged',changes.pass,...
    'saved_home_sha256_before',homeHashBefore,...
    'saved_home_sha256_after',hmi_workflow_file_hash(protectedHome),...
    'production_source_copy_identical',true,'rng_state_restored',true,...
    'captured_options_match_exact_hmi_recipe',true,...
    'fixture_options',input.options,'robot_motion','NO','executionReady',false,...
    'test_scope','Return contract only; no full weld or execution acceptance');
save(fullfile(outputDir,'point14_bridge_return.mat'),'bridge','summary',...
    'insertedPoses','insertedQ','report','-v7.3');
writeText(fullfile(outputDir,'bridge_return_report.json'),jsonencode(report,'PrettyPrint',true));
fprintf('BRIDGE_POINT14_RETURN_SCHEMA_PASS success=%d inserted=%d reason=%s elapsed=%.3f\n',...
    bridge.success,size(insertedQ,1),string(bridge.failure_reason),replayElapsed);
diary off;
end

function writeText(path,text)
fid=fopen(path,'w'); assert(fid>=0,'Cannot write test artifact');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,text,'char');
end
