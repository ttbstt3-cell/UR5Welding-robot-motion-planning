function drawPlannerEdge(p1,p2,color,pauseTime)
% drawPlannerEdge  Draw one tree edge outside the planning timer.

plot([p1(1) p2(1)],[p1(2) p2(2)],'-', ...
    'Color',color,'LineWidth',0.7);
drawnow;

fig = gcf;
if isgraphics(fig)
    recordPlannerGifFrame(fig,false);
end

if pauseTime > 0
    pause(pauseTime);
end

end
