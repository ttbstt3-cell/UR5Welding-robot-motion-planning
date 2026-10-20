function export_derivative_analysis
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(fileparts(here)));
addpath(fullfile(root,'matlab仿真'));
previous=fullfile(fileparts(here),'continuous_refactor_20261019'); result=struct();
for name={'current','historical'}
    caseName=name{1}; z=load(fullfile(previous,caseName,'parameterization.mat')); before=z.candidate;
    z=load(fullfile(here,[caseName '_spline'],'candidate.mat')); after=z.candidate;
    b=load(fullfile(previous,caseName,'geometry.mat')); geometry=b.g;
    assert(isequal(before.qPath,after.qPath) && isequal(before.tcpPosePath,after.tcpPosePath));
    pair={before,after}; labels={'before','after'};
    for k=1:2
        c=pair{k}; d=accp_cubic_derivative_metrics(c.qPath,c.timeFromStart,c.velocities);
        a=[(1:size(c.qPath,1)-1).',c.timeFromStart(1:end-1),c.timeFromStart(2:end),...
            c.velocities(1:end-1,:),c.velocities(2:end,:),d.acceleration_start,d.acceleration_end,d.segment_jerk];
        fields={'edge','time_start_sec','time_end_sec'};
        for group={'v_start','v_end','a_start','a_end','jerk'}
            for j=1:6, fields{end+1}=sprintf('%s_J%d',group{1},j); end %#ok<AGROW>
        end
        writetable(array2table(a,'VariableNames',fields),fullfile(here,[caseName '_' labels{k} '_derivatives_offline.csv']));
        [peak,index]=max(abs(d.segment_jerk(:))); [e,j]=ind2sub(size(d.segment_jerk),index);
        caseInfo.(labels{k})=struct('peak_jerk',peak,'peak_joint',j,'peak_edge',e,...
            'peak_time_interval',c.timeFromStart(e:e+1).','peak_segment',c.segmentLabel{e+1},...
            'peak_acceleration',d.max_acceleration,'rms_acceleration',d.rms_acceleration,...
            'rms_segment_jerk',d.rms_segment_jerk,'max_acceleration_jump',d.max_acceleration_jump,...
            'interface_acceleration_jump',max(abs(d.node_acceleration_jump(c.joins-1,:)),[],'all'),...
            'rest_end_acceleration',max(abs([d.acceleration_start(1,:),d.acceleration_end(end,:)])),...
            'duration',c.timeFromStart(end),'nodes',size(c.qPath,1));
    end
    caseInfo.all_q_exact=isequal(before.qPath,after.qPath); caseInfo.all_tcp_exact=isequal(before.tcpPosePath,after.tcpPosePath);
    caseInfo.geometry_reference_exact=isequal(before.pathTimeReference.poses,after.pathTimeReference.poses) && isequal(before.pathTimeReference.s,after.pathTimeReference.s);
    caseInfo.weld47_exact=isequal(after.qPath(after.maps{2},:),geometry.originalQ(geometry.originalMaps{2},:));
    result.(caseName)=caseInfo;
end
z=load(fullfile(here,'current','baseline_trace.mat'),'trace');
result.exact_state_cache=struct('samples',size(z.trace.Q,1),'unique_joint_states',size(unique(z.trace.Q,'rows'),1),...
 'note','No rounded-angle or cross-request cache was introduced');
result.executionReady=false; result.robot_motion='NO';
fid=fopen(fullfile(here,'derivative_analysis.json'),'w'); cleanup=onCleanup(@()fclose(fid)); fwrite(fid,jsonencode(result,'PrettyPrint',true),'char');
disp(result.exact_state_cache);
end
