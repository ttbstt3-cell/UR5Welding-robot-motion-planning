function fig = visualize_accp_point20_edge_audit(auditFile, inputFile, candidateIndex)
%VISUALIZE_ACCP_POINT20_EDGE_AUDIT
% Offline visualization for the Point19 -> Point20 dense edge audit.
% It shows the robot (including the real WeldingGun/Wire bodies), workpiece
% collision boxes, Point19, the first colliding interpolated pose, and the
% Point20 endpoint. It never publishes or sends a robot command.
%
% Example:
%   visualize_accp_point20_edge_audit([],[],1)

if nargin < 1 || isempty(auditFile)
    auditFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        '规划文件','real_anchor_runs','full_accp_wire3_20261013', ...
        'point20_local_recovery_20261013', ...
        'point19_point20_edge_audits.json');
end
if nargin < 2 || isempty(inputFile)
    inputFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        '规划文件','real_anchor_runs','full_accp_wire3_20261013', ...
        'real_seam_round_recovery_beam3.mat');
end
if nargin < 3 || isempty(candidateIndex), candidateIndex = 1; end

A = jsondecode(fileread(auditFile));
if candidateIndex < 1 || candidateIndex > numel(A.audits)
    error('candidateIndex must be between 1 and %d.', numel(A.audits));
end
audit = A.audits(candidateIndex);
S = load(inputFile,'result');
scene = S.result.scene;
prev = S.result.stateBank{19}(1);

firstIndex = audit.first_collision_sample_index;
if firstIndex > 0
    qFirst = audit.first_collision_q_rad(:).';
    firstLabel = sprintf('first collision (s=%.3f)', ...
        audit.first_collision_fraction);
else
    qFirst = audit.q_end_rad(:).';
    firstLabel = 'no collision in sampled edge';
end
qList = {prev.q(:).', qFirst, audit.q_end_rad(:).'};

fig = figure('Name',sprintf('ACCP Point19-20 edge audit: %s',audit.label), ...
    'Color','w');
tl = tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
for k = 1:3
    ax = nexttile(tl,k);
    show(scene.robot,qList{k},'Parent',ax,'Visuals','on', ...
        'Collisions','on','Frames','off','PreservePlot',false);
    hold(ax,'on');
    for e = 1:numel(scene.environment)
        show(scene.environment{e},'Parent',ax);
    end
    Ttcp = getTransform(scene.robot,qList{k},scene.tcpBodyName);
    p = Ttcp(1:3,4);
    plot3(ax,p(1),p(2),p(3),'ro','MarkerFaceColor','r','MarkerSize',7);
    text(ax,p(1),p(2),p(3),' TCP','Color',[0.75 0 0], ...
        'FontWeight','bold','Interpreter','none');
    axis(ax,'equal');
    grid(ax,'on');
    view(ax,135,25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    if k == 1
        title(ax,'Point19');
    elseif k == 2
        title(ax,firstLabel,'Interpreter','none');
    else
        title(ax,'Point20 endpoint');
    end
end
title(tl,sprintf('Point19 -> Point20 dense audit | %d samples | %s', ...
    audit.sample_count, strjoin(audit.first_collision_pairs,'; ')), ...
    'Interpreter','none');
end

