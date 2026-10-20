function potential = computeImprovedAPFRepulsivePotential( ...
    point,target,obstacles,config)
% computeImprovedAPFRepulsivePotential  Eq.(3.17) in the 2-D model.

point = reshape(point,1,[]);
target = reshape(target,1,[]);
goalDistance = norm(point-target);
potential = 0;
for i = 1:size(obstacles,1)
    [distance,~,~] = distanceToRectangleObstacle( ...
        point,obstacles(i,:),config.clearance);
    if distance <= config.apfInfluenceDistance
        safeDistance = max(distance,config.apfForceTolerance);
        base = 1/safeDistance-1/config.apfInfluenceDistance;
        potential = potential+0.5*config.apfKrep*base^2* ...
            goalDistance^config.apfRepulsiveExponent;
    end
end

end
