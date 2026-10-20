function [threshold,details] = computeDynamicGoalBias(config)
% computeDynamicGoalBias  Evaluate the APF-Bi-RRT* Eq.(3.13) threshold.
%
% Adopted expression:
% m_mot = pMax - (pMax-pMin)*(Vobs/Vtotal)^k.
% In this 2-D project Vobs/Vtotal is an area ratio, not a 3-D volume ratio.

[obstacleMeasure,workspaceMeasure] = computeObstacleUnionArea( ...
    config.obstacles,config.mapBounds);
occupancy = obstacleMeasure/max(workspaceMeasure,eps);
occupancy = min(max(occupancy,0),1);
threshold = config.apfGoalBiasMax- ...
    (config.apfGoalBiasMax-config.apfGoalBiasMin)* ...
    occupancy^config.apfGoalBiasExponent;

details.obstacleMeasure = obstacleMeasure;
details.workspaceMeasure = workspaceMeasure;
details.occupancy = occupancy;
details.pMax = config.apfGoalBiasMax;
details.pMin = config.apfGoalBiasMin;
details.exponent = config.apfGoalBiasExponent;

end
