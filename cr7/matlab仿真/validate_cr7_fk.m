function validate_cr7_fk()
scene=load_cr7_scene();samples=jsondecode(fileread(fullfile(scene.root,'simulation','fk_samples.json')));
positionError=0;rotationError=0;
for k=1:numel(samples)
 T=getTransform(scene.robot,samples(k).q(:)','WeldTCP');truth=[reshape(samples(k).tcp_matrix,4,3)';0 0 0 1];
 positionError=max(positionError,norm(T(1:3,4)-truth(1:3,4)));
 rotationError=max(rotationError,norm(T(1:3,1:3)-truth(1:3,1:3),'fro'));
end
assert(positionError<1e-8 && rotationError<1e-8,'CR7 FK mismatch');
fprintf('FK PASS: %d poses, position %.3g m, rotation matrix %.3g\n',numel(samples),positionError,rotationError);
fid=fopen(fullfile(scene.root,'simulation','fk_validation.txt'),'w');fprintf(fid,'FK PASS: %d poses, position %.3g m, rotation matrix %.3g\n',numel(samples),positionError,rotationError);fclose(fid);
end
