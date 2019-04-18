function [value, isterminal, direction] = eventsSSDynamics3DRIOxMinus(t, x, thetaLimits, hAlpha, leg, stanceFoot, terrainVector, forceVector, sigmaScale, thetaLimits2, halpha2, Letax, Letay, allStep )
% This function checks events for all of the perturbed pre-impact states
% that will be used for robust impact optimization.
% BAG20141003.
% BAG20150526 Updated for extended controller.
% BAG20150510 Updated to work for NHVC 3D.
% BAG20150218 Updated to work with 2D AMBVC.
% BAG20141024 Use relative swing foot position as opposed to s to determine
% swing leg height.

% Potential improvements:
% Policy for deciding when to check terrain, currently does not 
% consistently handle case where robot is significantly yawed.

% Initialize variables.
q=x(1:9); dq=x(10:18);
onesVector = ones(size(terrainVector));
zeroVector = 0*onesVector;

% Calculate center of mass position and foot height.
qPcm = [0; q(2:9)]; % Remove yaw for theta calculation.
[swingFoot, pcm] = points3D( qPcm,leg,stanceFoot );

% Check impact manifold.
if (swingFoot(2) < 0) % Don't want events to take place as swing leg is rising.
    value = zeroVector;
else % If s is past boundary, value is height of swing leg.
    value = swingFoot(3)*onesVector-terrainVector;
end

isterminal = [zeroVector(1:end-1); 1];
direction = -onesVector;

% % For debugging.
% display(s)

return