function update_cr7_view(scene,display,q)
for i=1:numel(display.actors)
 local=display.local{i};T=local.matrix;
 if local.robot && display.owners(i)>0
  T=getTransform(scene.robot,q,sprintf('link%d',display.owners(i)))*T;
 end
 v=local.vertices*T(1:3,1:3)'+T(1:3,4)';set(display.actors{i},'Vertices',v);
end
T=getTransform(scene.robot,q,scene.tcpBodyName);
W=T*scene.T_tcp_wire;ends=W*[0 0;0 0;-.009 .009;1 1];
set(display.wire,'XData',ends(1,:),'YData',ends(2,:),'ZData',ends(3,:));
set(display.tcp,'XData',T(1,4),'YData',T(2,4),'ZData',T(3,4));drawnow limitrate;
end
