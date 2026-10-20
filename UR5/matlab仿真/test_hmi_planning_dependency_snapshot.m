function report=test_hmi_planning_dependency_snapshot(outputDir)
% File-identity fixtures only. Does not touch any production model/config.
if ~isfolder(outputDir), mkdir(outputDir); end
fixtureRoot=tempname(outputDir); mkdir(fixtureRoot);
mesh=fullfile(fixtureRoot,'collision_fixture.stl');
data=fullfile(fixtureRoot,'real_gun_geometry','collision_fixture.json');
writeFixture(mesh,'collision-A'); writeFixture(data,'{"vertices":"A"}');
writeFixture(fullfile(fixtureRoot,'real_robot_description_calibrated.urdf'),...
    '<robot name="fixture"><link name="base"><collision><geometry><mesh filename="collision_fixture.stl"/></geometry></collision></link></robot>');
writeFixture(fullfile(fixtureRoot,'real_gun_geometry','real_gun_geometry.json'),...
    '{"collision":{"data_file":"real_gun_geometry/collision_fixture.json"}}');
first=hmi_planning_dependency_snapshot(fixtureRoot);
same=hmi_planning_dependency_changes(first,hmi_planning_dependency_snapshot(fixtureRoot));
assert(same.pass && isempty(same.changed_files));
writeFixture(mesh,'collision-B');
second=hmi_planning_dependency_snapshot(fixtureRoot);
meshDiff=hmi_planning_dependency_changes(first,second);
assert(~meshDiff.pass && any(strcmp(meshDiff.changed_files,mesh)));
writeFixture(data,'{"vertices":"B"}');
third=hmi_planning_dependency_snapshot(fixtureRoot);
dataDiff=hmi_planning_dependency_changes(second,third);
assert(~dataDiff.pass && any(strcmp(dataDiff.changed_files,data)));
tcp=fullfile(fixtureRoot,'real_tcp_calibration_active.json'); writeFixture(tcp,'{"version":"NEW"}');
created=hmi_planning_dependency_changes(third,hmi_planning_dependency_snapshot(fixtureRoot));
assert(~created.pass && any(strcmp(created.changed_files,tcp)));
writeFixture(fullfile(fixtureRoot,'real_gun_geometry','real_gun_geometry.json'),...
    '{"collision":{"data_file":"real_gun_geometry/changed_reference.json"}}');
references=hmi_planning_dependency_changes(third,hmi_planning_dependency_snapshot(fixtureRoot));
assert(~references.pass && any(contains(string(references.changed_files),'changed_reference.json')));
report=struct('pass',true,'robot_motion','NO','executionReady',false,'fixture_root',fixtureRoot,...
    'unchanged_snapshot',same,'collision_mesh_content_change',meshDiff,...
    'referenced_collision_data_change',dataDiff,'missing_dependency_created',created,...
    'reference_list_change',references,'production_files_written',false);
fid=fopen(fullfile(outputDir,'dependency_snapshot_test_report.json'),'w'); c=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(report,'PrettyPrint',true),'char');
fprintf('HMI_DEPENDENCY_SNAPSHOT_PASS unchanged/content/creation/reference tests\n');
end
function writeFixture(file,value)
folder=fileparts(file); if ~isfolder(folder), mkdir(folder); end
fid=fopen(file,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,value,'char');
end
