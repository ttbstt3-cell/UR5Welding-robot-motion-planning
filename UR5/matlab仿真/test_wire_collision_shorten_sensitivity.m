function report=test_wire_collision_shorten_sensitivity(outputDir)
% Point-1 diagnostic only. Visual wire/TCP remain full length; only the
% MATLAB collisionCylinder is shortened for each isolated comparison.
here=fileparts(mfilename('fullpath')); root=fileparts(here);
if nargin<1
    outputDir=fullfile(root,'planning_runs',['wire_collision_shorten_' datestr(now,'yyyymmdd_HHMMSSFFF')]);
end
if ~isfolder(outputDir), mkdir(outputDir); end
baseline=fullfile(root,'规划文件','real_anchor_runs','point1_axis_ab_current_20261013','current_core.mat');
if ~isfile(baseline)
    baseline=fullfile(root,'规划文件','real_anchor_runs','Areal_confirmed_20261013','real_seam_round_01.mat');
end
S=load(baseline,'result'); base=S.result;
baseOptions=base.options;
baseOptions.sceneScript=fullfile(here,'sync_ur5_real_scene.m');
baseOptions.configurationOnlyDiagnostic=true;
baseOptions.candidate_diagnostics=true;
baseOptions.candidate_scoring_mode=false;
baseOptions.disableAxisConstraint=false;
baseOptions.disable_axis_constraint=false;
baseOptions.publish=false; baseOptions.animate=false;
baseOptions.checkSelfCollision=true;
shorten=[0 3 5 10]; rows=cell(1,numel(shorten));
for i=1:numel(shorten)
    mm=shorten(i); runDir=fullfile(outputDir,sprintf('shorten_%02dmm',mm));
    if ~isfolder(runDir), mkdir(runDir); end
    opts=baseOptions; opts.wire_collision_shorten_mm=mm;
    opts.outputFile=fullfile(runDir,'point1_core.mat');
    fprintf('\n=== wire_collision_shorten_mm = %.1f ===\n',mm);
    core=replan_weld_actual_axis_adaptive_core(opts);
    result=core; %#ok<NASGU>
    save(fullfile(runDir,'result.mat'),'result','-v7.3');
    auditDir=fullfile(runDir,'pair_audit');
    diagnose_accp_point1_collision_pairs(fullfile(runDir,'result.mat'),auditDir,'all_ik');
    d=core.firstPointDiagnostics;
    auditJson=jsondecode(fileread(fullfile(auditDir,'collision_pairs.json')));
    candidates=auditJson.candidates;
    wireCount=0; workpieceCount=0;
    for k=1:numel(candidates)
        pairs=candidates(k).colliding_pairs;
        if iscell(pairs), pairs=[pairs{:}]; end
        labels=string.empty;
        if ~isempty(pairs)
            labels=string({pairs.object_a})+" <-> "+string({pairs.object_b});
        end
        wireCount=wireCount+any(contains(labels,"WireCollision"));
        workpieceCount=workpieceCount+any(contains(labels,"WorkpieceCollision"));
    end
    row=struct('wire_collision_shorten_mm',mm, ...
        'visual_wire_length_mm',core.scene.wireLengthM*1000, ...
        'collision_wire_length_mm',core.scene.wireCollisionLengthM*1000, ...
        'ik_count',d.ik_success,'wire_collision_count',wireCount, ...
        'workpiece_collision_count',workpieceCount, ...
        'final_feasible_candidate_count',d.accepted_after_all_filters, ...
        'axis_constraint_preserved',~core.options.disableAxisConstraint, ...
        'tcp_unchanged',true,'robot_motion','NO', ...
        'calibration_binding',core.scene.calibrationBinding, ...
        'pair_audit',fullfile(auditDir,'collision_pairs.json'));
    rows{i}=row;
    writeJson(fullfile(runDir,'summary.json'),row);
    fprintf('SHORTEN %.1f mm: IK=%d wire=%d workpiece=%d feasible=%d visual=%.6f collision=%.6f mm\n', ...
        mm,row.ik_count,row.wire_collision_count,row.workpiece_collision_count, ...
        row.final_feasible_candidate_count,row.visual_wire_length_mm,row.collision_wire_length_mm);
end
report=struct('schema_version','WIRE_COLLISION_SHORTEN_SENSITIVITY_v1', ...
    'description','Diagnostic-only collisionCylinder shortening; visual wire and TCP unchanged', ...
    'shorten_values_mm',shorten,'results',{rows},'robot_motion','NO', ...
    'formal_collision_filter_unchanged',true,'tcp_unchanged',true);
writeJson(fullfile(outputDir,'comparison.json'),report);
fid=fopen(fullfile(outputDir,'comparison.csv'),'w');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'shorten_mm,ik_count,wire_collision_count,workpiece_collision_count,final_feasible_count,visual_wire_length_mm,collision_wire_length_mm\n');
for i=1:numel(rows)
    r=rows{i}; fprintf(fid,'%.1f,%d,%d,%d,%d,%.9f,%.9f\n',r.wire_collision_shorten_mm,r.ik_count,...
        r.wire_collision_count,r.workpiece_collision_count,r.final_feasible_candidate_count,...
        r.visual_wire_length_mm,r.collision_wire_length_mm);
end
end

function writeJson(path,data)
fid=fopen(path,'w'); if fid<0, error('Cannot write %s',path); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
payload=jsonencode(data,'PrettyPrint',true);
fwrite(fid,payload,'char');
end
