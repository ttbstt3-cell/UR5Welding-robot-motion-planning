function raw=assemble_hmi_preview_trajectory(segments,binding,sceneFile,outputFile)
% Preserve every stage node, including discontinuous/duplicate junctions.
% The concatenated clock is for animation only; execution uses stage clocks.
qPath=[]; tcpPosePath={}; segmentLabel={}; timeFromStart=[]; breaks=[];
for k=1:numel(segments)
    s=segments{k}; n=size(s.qPath,1);
    assert(n>=2 && size(s.qPath,2)==6 && all(isfinite(s.qPath(:))),'Empty/malformed stage path');
    assert(numel(s.tcpPosePath)==n && numel(s.timeFromStart)==n,'Stage row mismatch');
    offset=0;
    if ~isempty(timeFromStart), offset=timeFromStart(end)+0.25; breaks(end+1)=size(qPath,1)+1; end %#ok<AGROW>
    qPath=[qPath;s.qPath]; tcpPosePath=[tcpPosePath,s.tcpPosePath(:).']; %#ok<AGROW>
    segmentLabel=[segmentLabel,repmat({s.label},1,n)]; %#ok<AGROW>
    timeFromStart=[timeFromStart;s.timeFromStart(:)-s.timeFromStart(1)+offset]; %#ok<AGROW>
end
tcpPath=cell2mat(cellfun(@(T)T(1:3,4).',tcpPosePath,'UniformOutput',false).');
robotState=qPath; weldingPosePath=tcpPosePath;
joint_names={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
metadata=struct('source','MATLAB_HMI_PREVIEW','trajectory_name','full_raw_trajectory',...
    'description','Simulation/display only; stage boundaries are not repaired.',...
    'preview_only',true,'executionReady',false,'workflow_stage','PREVIEW',...
    'calibration_binding',binding,'tcp_frame','base_link','timing_scope','DISPLAY_ONLY',...
    'segment_break_indices',breaks,'collision_verified',false,'collision_check_pass',false);
metadata.scene_snapshot_sha256=hmi_workflow_file_hash(sceneFile);
raw=struct('qPath',qPath,'tcpPosePath',{tcpPosePath},'tcpPath',tcpPath,...
    'timeFromStart',timeFromStart,'segments',{segments},'metadata',metadata,'sceneFile',sceneFile);
save(outputFile,'qPath','tcpPosePath','tcpPath','timeFromStart','robotState','weldingPosePath',...
    'segmentLabel','joint_names','metadata','segments','sceneFile','-v7');
end
