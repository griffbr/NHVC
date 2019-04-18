function [dx] = SSDynamics3D( t,x,thetaLimits,hAlpha,leg,stanceFoot,terrainVector,forceVector,sigmaScale,thetaLimits2,halpha2,allStep )
% Function for computing the single support continuous dynamics using ode
% in MATLAB. Based on rigid 3D model of ATRIAS.
% BAG20140730
% BAG20150526 Updated for extended control.
% BAG20150510 Updated to work for NHVC 3D.
% BAG20150201 Updated to work with extended hAlpha map.
% BAG20141222 Updated to account for lateral forces on CoM (disturbance).

%% ddq 
q=x(1:9); dq=x(10:18); 

% Lagrange
[D,Cdq,G,B] = lagrange3D(q,dq,leg);
H = Cdq+G; % Can have more complicated H. (yaw friction and springs)

% Call controller function to find u.
[u] = control3D( q,dq,D,B,H,thetaLimits,hAlpha,leg,sigmaScale,thetaLimits2,halpha2,allStep );

ddq = D\(B*u + -H);
    
dx = [dq; ddq];

%% Misc

%% Debugging

end

