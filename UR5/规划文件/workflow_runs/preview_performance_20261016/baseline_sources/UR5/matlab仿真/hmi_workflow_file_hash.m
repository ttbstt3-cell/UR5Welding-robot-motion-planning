function h=hmi_workflow_file_hash(path)
fid=fopen(path,'r'); assert(fid>=0,'Missing input file: %s',path); c=onCleanup(@()fclose(fid));
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(fread(fid,Inf,'*uint8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
