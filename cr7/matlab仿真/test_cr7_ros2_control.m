% Simulation-only round trip; record feedback and verify the 2-degree move.
node=ros2node('/matlab_cr7_test');sub=ros2subscriber(node,'/cr7/joint_states','sensor_msgs/JointState');
msg=receive(sub,10);scene=load_cr7_scene();[found,order]=ismember(scene.jointNames,msg.name);assert(all(found));
q=reshape(double(msg.position(order)),1,6);target=q;target(1)=target(1)+deg2rad(2);
publish_cr7_trajectory([q;target;target;q],[0;2;3;5]);
timer=tic;history=[];
while toc(timer)<6
 msg=receive(sub,2);[found,order]=ismember(scene.jointNames,msg.name);
 if all(found),history(end+1,:)=reshape(double(msg.position(order)),1,6);end %#ok<SAGROW>
end
excursion=max(abs(history(:,1)-q(1)));finalError=max(abs(history(end,:)-q));
assert(abs(excursion-deg2rad(2))<deg2rad(.1) && finalError<deg2rad(.1),'CR7 feedback mismatch');
fid=fopen(fullfile(scene.root,'simulation','ros2_validation.txt'),'w');
fprintf(fid,'MATLAB -> ROS2 -> ZMQ -> CR7 -> ROS2 -> MATLAB PASS\n%d feedback samples; J1 excursion %.6f deg; final error %.6f deg\n',size(history,1),rad2deg(excursion),rad2deg(finalError));fclose(fid);
save(fullfile(scene.root,'simulation','ros2_roundtrip.mat'),'history','q','target');
disp('CR7 ROS2 round trip PASS');clear sub node;
