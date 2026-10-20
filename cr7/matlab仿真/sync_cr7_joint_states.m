clear;clc;
scene=load_cr7_scene();cr7Display=show_cr7_scene(scene);
cr7Node=ros2node('/matlab_cr7');
cr7Sub=ros2subscriber(cr7Node,'/cr7/joint_states','sensor_msgs/JointState');
disp('Listening to /cr7/joint_states; close the figure to stop.');
while isgraphics(cr7Display.figure)
 try,msg=receive(cr7Sub,2);catch,drawnow;continue;end
 [found,order]=ismember(scene.jointNames,msg.name);
 if all(found)
  q=double(msg.position(order))';update_cr7_view(scene,cr7Display,q);
 end
end
clear cr7Sub cr7Node;
