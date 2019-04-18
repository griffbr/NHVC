function [value, isterminal, direction] = eventsSSDynamics3D( t,x,thetaLimits,hAlpha,leg,stanceFoot,terrainVector,forceVector,sigmaScale,thetaLimits2,halpha2,Letax,Letay,allStep )
% Events function made to work with SSDynamics2D. Primary event is when 
% swing foot hits ground.
% BAG20150122.
% BAG20150526 Updated to work with extended controller.
% BAG20150510 Updated for NHVC 3D.
% BAG20150216 Updated for AMBVC.
% BAG20150128. Updated to work with angular momentum outputs. Also changed
% non-zeno locking event to be when the center of mass passes over the
% stance foot.

% Initialize variables.
q=x(1:9); dq=x(10:18);

% Calculate center of mass position and foot height.
qPcm = [0; q(2:9)]; % Remove yaw for theta calculation.
[swingFoot] = points3D( qPcm,leg,stanceFoot );

% Check impact manifold.
if (swingFoot(2) < stanceFoot(2)) % Don't want events to take place as swing leg is rising.
    value = [0];
else % If s is past boundary, value is height of swing leg.
    value = swingFoot(3)-terrainVector;
    % display(value)
end

isterminal= [1];
direction = [-1];

return