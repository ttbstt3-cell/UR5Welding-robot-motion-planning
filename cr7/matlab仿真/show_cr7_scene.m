function display=show_cr7_scene(scene,q)
if nargin<2,q=scene.qCurrent;end
fig=figure('Name','CR7-7 MATLAB / CoppeliaSim','Color','w');ax=axes(fig);hold(ax,'on');
display=struct('figure',fig,'axes',ax,'actors',{{}},'owners',[],'local',{{}});
for i=1:numel(scene.geometry.shapes)
 s=scene.geometry.shapes(i);
 if contains(s.name,'_respondable') || ~s.visible,continue;end
 mesh=load(s.mesh_file);
 p=patch(ax,'Vertices',mesh.vertices,'Faces',mesh.faces,'FaceColor',s.color(:)', ...
 'EdgeColor','none','FaceLighting','gouraud');
 display.actors{end+1}=p;display.owners(end+1)=s.owner;
 display.local{end+1}=struct('vertices',mesh.vertices,'matrix',[reshape(s.matrix,4,3)';0 0 0 1],'robot',s.robot);
end
display.wire=plot3(ax,[0 0],[0 0],[0 0],'Color',[.95 .75 .15],'LineWidth',2);
display.tcp=plot3(ax,0,0,0,'r.','MarkerSize',22);
update_cr7_view(scene,display,q);axis(ax,'equal');grid(ax,'on');view3d(ax);viewAngle=[135 25];view(ax,viewAngle);
axis(ax,[-.8 .8 -.8 .8 -.1 1.85]);title(ax,'CR7-7 / UR5 welding environment');
xlabel(ax,'X (m)');ylabel(ax,'Y (m)');zlabel(ax,'Z (m)');camlight(ax,'headlight');
end
function view3d(ax)
axis(ax,'vis3d');
end
