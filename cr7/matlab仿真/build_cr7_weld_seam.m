function seam=build_cr7_weld_seam(numPoints,endMarginM)
% Same inner back-wall / bottom seam as the UR5 actual-axis baseline.
if nargin<1,numPoints=80;end
if nargin<2,endMarginM=.025;end
assert(numPoints>=2 && numPoints==fix(numPoints));assert(endMarginM>=0 && endMarginM<.1125);
scene=load_cr7_scene();L=.2349999994039536;t=.005;
p0=[t+endMarginM;L-t;t];p1=[L-t-endMarginM;L-t;t];
s=linspace(0,1,numPoints)';points=(1-s).*p0'+s.*p1';
P=(scene.T_MatBase_Workpiece*[points,ones(numPoints,1)]')';
seam=struct('positionsWP',points,'positionsMat',P(:,1:3),'startWP',p0,'endWP',p1, ...
 'tangentWP',[1;0;0],'numPoints',numPoints,'endMarginM',endMarginM,'plateThicknessM',t);
end
