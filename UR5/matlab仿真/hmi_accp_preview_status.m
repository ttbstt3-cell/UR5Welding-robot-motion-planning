function status=hmi_accp_preview_status(weld)
% Classify actual ACCP stages, never promote a sparse/diagnostic path to PASS.
status=struct('classification','OUTPUT_SCHEMA_FAILURE','geometry_complete',false,...
    'configuration_complete',false,'preweld_available',false,'failed_point',[],...
    'recovered_points',[],'failed_refinement_edges',[],'failed_dense_edges',[],...
    'failed_hard_nodes',[],'reason','','executionReady',false);
required={'configurationComplete','adaptiveCorrectionComplete','qPath',...
    'nodeCollisionFree','edgeCollisionFree','cartesianRefinement'};
for k=1:numel(required)
    if ~isfield(weld,required{k})
        status.reason=['Missing ACCP result field: ' required{k}]; return;
    end
end
status.configuration_complete=logical(weld.configurationComplete);
if isfield(weld,'cartesianBridge')
    status.recovered_points=weld.cartesianBridge.recovered_point_indices;
end
if ~status.configuration_complete
    status.classification='NO_CONNECTED_CANDIDATE';
    if isfield(weld,'stateBank'), status.failed_point=find(cellfun(@isempty,weld.stateBank),1); end
    if isfield(weld,'firstPointDiagnostics') && isfield(weld.firstPointDiagnostics,'ik_success') &&...
            weld.firstPointDiagnostics.ik_success==0
        status.classification='IK_FAILURE';
    end
    status.reason=sprintf('%s: no complete connected weld configuration chain',status.classification);
    return;
end
ref=weld.cartesianRefinement;
if ~isfield(ref,'complete') || ~isfield(ref,'segmentReport') || ~isfield(ref,'finalTcpMetrics')
    status.reason='Configuration exists, but complete precision-stage evidence is missing'; return;
end
status.failed_refinement_edges=find(~[ref.segmentReport.correctionAccepted]);
status.failed_dense_edges=find(~ref.finalTcpMetrics.segmentCollisionFree);
status.failed_hard_nodes=find(~ref.finalTcpMetrics.nodeHardConstraintsPass);
if ~weld.adaptiveCorrectionComplete || ~ref.complete
    status.classification='PRECISION_FAILURE';
    if ~ref.finalTcpMetrics.nodeCollisionFree, status.classification='COLLISION_FAILURE'; end
    % segmentCollisionFree also includes the original axis/side predicates;
    % do not mislabel an axis rejection as a physical collision.
    status.reason=sprintf('%s: configuration connected; refinement edges=%s, hard nodes=%s, max TCP error=%.6f mm',...
        status.classification,mat2str(status.failed_refinement_edges),mat2str(status.failed_hard_nodes),...
        max(ref.finalTcpMetrics.segmentPositionErrorMm));
    return;
end
if size(weld.qPath,1)<2 || size(weld.qPath,2)~=6 || any(~isfinite(weld.qPath(:))) ||...
        ~weld.nodeCollisionFree || ~weld.edgeCollisionFree || ~isempty(status.failed_hard_nodes)
    status.reason='ACCP success flags contradict its path or hard-validation evidence'; return;
end
status.classification='PASS'; status.geometry_complete=true; status.preweld_available=true;
status.reason='Configuration, Cartesian precision and hard geometry validation complete; timing/execution separate';
end
