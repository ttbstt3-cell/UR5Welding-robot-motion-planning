function [force,details] = computeImprovedAPFDirection( ...
    point,target,obstacles,config)
% computeImprovedAPFDirection  APF force used for APF-Bi-RRT* expansion.
%
% Attraction is the standard vector expression.  The repulsive force is
% derived analytically as -gradient(Urep*) from Eq.(3.17); the supplied paper
% notes do not list the complete vector formula explicitly.

point = reshape(point,1,[]);
target = reshape(target,1,[]);
goalVector = target-point;
goalDistance = norm(goalVector);
kattMot = config.apfKattMin+config.apfLambda*goalDistance;
attractiveForce = kattMot*goalVector;
repulsiveForce = [0 0];
repulsivePotential = 0;
activeObstacleCount = 0;

for i = 1:size(obstacles,1)
    [distance,~,repulsiveDirection] = distanceToRectangleObstacle( ...
        point,obstacles(i,:),config.clearance);
    if distance > config.apfInfluenceDistance
        continue;
    end

    activeObstacleCount = activeObstacleCount+1;
    safeDistance = max(distance,config.apfForceTolerance);
    influence = 1/safeDistance-1/config.apfInfluenceDistance;
    basePotential = 0.5*config.apfKrep*influence^2;
    goalScale = goalDistance^config.apfRepulsiveExponent;
    repulsivePotential = repulsivePotential+basePotential*goalScale;

    % grad(basePotential) points toward decreasing obstacle distance.  The
    % negative gradient therefore has the direction below.  The second term
    % comes from differentiating the goal-distance multiplier.
    gradBase = -config.apfKrep*influence/(safeDistance^2)* ...
        repulsiveDirection;
    if goalDistance > config.apfForceTolerance
        gradGoalScale = config.apfRepulsiveExponent* ...
            goalDistance^(config.apfRepulsiveExponent-2)* ...
            (point-target);
    else
        gradGoalScale = [0 0];
    end
    gradPotential = goalScale*gradBase+basePotential*gradGoalScale;
    repulsiveForce = repulsiveForce-gradPotential;
end

force = attractiveForce+repulsiveForce;
details.attractiveForce = attractiveForce;
details.repulsiveForce = repulsiveForce;
details.repulsivePotential = repulsivePotential;
details.totalPotential = 0.5*kattMot*goalDistance^2+repulsivePotential;
details.kattMot = kattMot;
details.goalDistance = goalDistance;
details.activeObstacleCount = activeObstacleCount;

end
