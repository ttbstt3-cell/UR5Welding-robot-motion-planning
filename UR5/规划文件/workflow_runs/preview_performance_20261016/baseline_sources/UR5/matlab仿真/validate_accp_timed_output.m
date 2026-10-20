function gate = validate_accp_timed_output(comparisonSource, options)
%VALIDATE_ACCP_TIMED_OUTPUT Apply an explicit, offline timing-output gate.
% This consumes the report produced by compare_accp_time_parameterizations and
% never changes ACCP search, models, or the ROS execution path.  A result is
% execution-ready only when the caller explicitly configures a finite
% time-sync tolerance and all independent checks pass.
if nargin < 1 || isempty(comparisonSource), error('comparisonSource is required.'); end
if nargin < 2 || isempty(options), options=struct(); end
cfg=struct('scheme','positions_only_linear','geometricAcceptanceMm',0.5,...
    'timeSyncAcceptanceMm',Inf);
names=fieldnames(options);
for i=1:numel(names), if isfield(cfg,names{i}), cfg.(names{i})=options.(names{i}); end, end

if ischar(comparisonSource) || isstring(comparisonSource)
    fid=fopen(char(comparisonSource),'r'); if fid<0, error('Cannot read %s',comparisonSource); end
    c=onCleanup(@()fclose(fid)); report=jsondecode(fread(fid,Inf,'*char').');
elseif isstruct(comparisonSource)
    report=comparisonSource;
else
    error('comparisonSource must be a JSON path or report struct.');
end
idx=find(strcmp({report.schemes.name},cfg.scheme),1);
gate=struct('schema_version','ACCP_TIMED_OUTPUT_GATE_v1','robot_motion','NO',...
    'scheme',cfg.scheme,'geometric_acceptance_mm',cfg.geometricAcceptanceMm,...
    'time_sync_acceptance_mm',cfg.timeSyncAcceptanceMm,'execution_ready',false);
if isempty(idx)
    gate.failure_reason=sprintf('TIMED_OUTPUT_GATE_FAILED: scheme %s is absent.',cfg.scheme); return;
end
s=report.schemes(idx);
gate.metrics=struct('contour_max_mm',s.contour.max_error_mm,...
    'contour_rms_mm',s.contour.rms_error_mm,'time_sync_max_mm',s.time_sync.max_error_mm,...
    'time_sync_rms_mm',s.time_sync.rms_error_mm,'orientation_max_deg',s.orientation.max_error_deg,...
    'collision_pass',s.validation.collision_pass,'joint_limits_pass',s.validation.joint_limits_pass,...
    'sample_count',s.validation.sample_count,'duration_sec',s.duration_sec);
checks={};
if s.contour.max_error_mm > cfg.geometricAcceptanceMm, checks{end+1}=sprintf('contour %.6f mm > %.6f mm',s.contour.max_error_mm,cfg.geometricAcceptanceMm); end
if ~s.validation.collision_pass, checks{end+1}='dense collision validation failed'; end
if ~s.validation.joint_limits_pass, checks{end+1}='joint-limit validation failed'; end
if ~isfinite(cfg.timeSyncAcceptanceMm), checks{end+1}='time-sync gate is not configured with a finite tolerance';
elseif s.time_sync.max_error_mm > cfg.timeSyncAcceptanceMm, checks{end+1}=sprintf('time-sync %.6f mm > %.6f mm',s.time_sync.max_error_mm,cfg.timeSyncAcceptanceMm); end
gate.execution_ready=isempty(checks);
if gate.execution_ready, gate.failure_reason=''; else, gate.failure_reason=['TIMED_OUTPUT_GATE_FAILED: ' strjoin(checks,'; ')]; end
end
