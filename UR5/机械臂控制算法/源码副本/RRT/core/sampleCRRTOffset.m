function [offsetPoint,radius,theta] = sampleCRRTOffset(xNearest,xNew)
% sampleCRRTOffset  Uniformly sample one point on the 2-D offset circle.

xNearest = reshape(xNearest,1,[]);
xNew = reshape(xNew,1,[]);
if numel(xNearest) ~= 2 || numel(xNew) ~= 2
    error('xNearest and xNew must be 1-by-2 points.');
end

radius = norm(xNew-xNearest);
theta = 2*pi*rand;
offsetPoint = xNearest+radius*[cos(theta),sin(theta)];

end
