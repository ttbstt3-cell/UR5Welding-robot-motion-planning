function limits=hmi_load_robot_motion_limits(file,expectedHash)
% Read-only controller/URDF ceilings. NOT installation safety certification.
if nargin<2, expectedHash=''; end
here=fileparts(mfilename('fullpath')); root=fileparts(here);
profile=jsondecode(fileread(file)); digest=hmi_workflow_file_hash(file);
assert(isempty(expectedHash)||strcmp(digest,expectedHash),'LIMITS:Stale','Motion limit profile changed');
assert(profile.schema_version==1 && strcmp(profile.source,'UR_PRIMARY_READ_ONLY_CONFIGURATION'),...
    'LIMITS:Schema','Invalid motion limit source/schema');
names={'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
assert(isequal(cellstr(string(profile.joint_names(:))),names.'),'LIMITS:JointOrder','Invalid joint order');
urdf=fullfile(root,'real_robot_description_calibrated.urdf');
assert(strcmp(hmi_workflow_file_hash(urdf),profile.urdf_sha256),'LIMITS:Stale','URDF changed');
hex=char(profile.controller.raw_configuration_hex);
assert(mod(numel(hex),2)==0 && all(ismember(lower(hex),'0123456789abcdef')),'LIMITS:Packet','Invalid raw hex');
bytes=uint8(sscanf(hex,'%2x').');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes);
rawHash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
assert(strcmp(rawHash,profile.controller.raw_configuration_sha256) && numel(bytes)>=445 && bytes(5)==6,...
    'LIMITS:Packet','Invalid controller packet/hash');
values=swapbytes(typecast(bytes(6:197),'double')); table=reshape(values,2,12).';
pos=table(1:6,:); motion=table(7:12,:);
assert(all(isfinite(table),'all') && all(pos(:,1)<pos(:,2)) && all(motion>0,'all'),...
    'LIMITS:Packet','Invalid configuration values');
assert(isequal(pos,profile.controller.position_limits_rad) &&...
    isequal(motion(:,1),profile.controller.velocity_rad_s(:)) &&...
    isequal(motion(:,2),profile.controller.acceleration_rad_s2(:)),...
    'LIMITS:Packet','Decoded controller limits differ from profile');
doc=xmlread(urdf); joints=doc.getElementsByTagName('joint'); lowerLimits=zeros(6,1); upperLimits=lowerLimits; velocity=lowerLimits;
for j=1:6
    found=false;
    for k=0:joints.getLength()-1
        joint=joints.item(k);
        if strcmp(char(joint.getAttribute('name')),names{j}) && joint.hasAttribute('type')
            item=joint.getElementsByTagName('limit').item(0);
            lowerLimits(j)=str2double(item.getAttribute('lower'));
            upperLimits(j)=str2double(item.getAttribute('upper'));
            velocity(j)=str2double(item.getAttribute('velocity')); found=true; break;
        end
    end
    assert(found,'LIMITS:Model','URDF joint missing');
end
lowerLimits=max(lowerLimits,pos(:,1)); upperLimits=min(upperLimits,pos(:,2)); velocity=min(velocity,motion(:,1));
evidencePresent=isfield(profile,'pendant_evidence'); normalTcp=[];
if evidencePresent
    evidence=profile.pendant_evidence;
    for kind={'general','joint'}
        path=fullfile(root,evidence.([kind{1} '_photo']));
        assert(strcmp(hmi_workflow_file_hash(path),evidence.([kind{1} '_photo_sha256'])),...
            'LIMITS:Evidence','Pendant photo changed');
    end
    speeds=deg2rad(evidence.normal_joint_velocity_deg_s(:)); normalTcp=evidence.normal_tcp_speed_m_s;
    assert(numel(speeds)==6 && all(isfinite(speeds)) && all(speeds>0) &&...
        isscalar(normalTcp) && isfinite(normalTcp) && normalTcp>0,'LIMITS:Evidence','Invalid pendant speeds');
    velocity=min(velocity,speeds);
end
assert(all(isfinite([lowerLimits;upperLimits;velocity])) && all(lowerLimits<upperLimits) && all(velocity>0),'LIMITS:Model','Invalid model intersection');
limits=struct('profile_sha256',digest,'profile_path',char(file),'source',profile.source,...
    'joint_names',{names},'urdf_sha256',profile.urdf_sha256,...
    'position_lower_rad',lowerLimits,'position_upper_rad',upperLimits,'velocity_rad_s',velocity,...
    'acceleration_rad_s2',motion(:,2),'scalar_velocity_rad_s',min(velocity),...
    'scalar_acceleration_rad_s2',min(motion(:,2)),'installation_safety_verified',false,...
    'normal_joint_speed_evidence_present',evidencePresent,'normal_tcp_speed_m_s',normalTcp,...
    'payload_acceleration_verified',false,'status','OFFLINE_LIMITS_AVAILABLE_EXECUTION_INCREASE_PENDING',...
    'note','Configuration ceilings, not safety settings or recommended operating speeds');
end
