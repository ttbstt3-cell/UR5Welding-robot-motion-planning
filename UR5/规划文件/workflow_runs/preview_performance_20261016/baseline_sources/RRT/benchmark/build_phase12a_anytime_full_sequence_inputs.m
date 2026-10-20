function files = build_phase12a_anytime_full_sequence_inputs(reportFile,legacyFile,outputDirectory)
%BUILD_PHASE12A_ANYTIME_FULL_SEQUENCE_INPUTS Build animation-compatible MATs.
% Only the two free-space RRT segments are replaced. The local PreWeld and
% PostWeld links and the existing 80-point weld path stay unchanged.

if nargin < 1 || isempty(reportFile)
    root=fileparts(fileparts(mfilename('fullpath')));
    reportFile=fullfile(root,'results','phase12a_anytime_seamA_two_successful.mat');
end
if nargin < 2 || isempty(legacyFile)
    root=fileparts(fileparts(mfilename('fullpath')));
    legacyFile=fullfile(root,'results','phase12a_seam_legacy_rrtconnect_repro.mat');
end
if nargin < 3 || isempty(outputDirectory)
    root=fileparts(fileparts(mfilename('fullpath')));
    outputDirectory=fullfile(root,'results');
end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
s=load(reportFile,'report'); r=s.report;
s=load(legacyFile,'result'); source=s.result;
files=strings(1,2);
for ai=1:2
    result=source;
    result.fullPass=true;
    result.approach.qPath=r.paths{1,ai};
    result.exit.qPath=r.paths{2,ai};
    result.approach.success=true; result.approach.pass=true;
    result.exit.success=true; result.exit.pass=true;
    if ai==1, name='RRTConnect'; else, name='BiRRT'; end
    result.generatedFor=string(r.algorithmLabels(ai));
    result.generatedAt=string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
    file=fullfile(outputDirectory,sprintf('phase12a_anytime_%s_full_sequence_input.mat',name));
    save(file,'result','-v7.3'); files(ai)=string(file);
end
fprintf('FULL_SEQUENCE_INPUTS_DONE\n'); disp(files);
end
