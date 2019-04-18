function [dx,u,q,dq,ddq,y,dy,s,ds,theta,dtheta,KE,J_cm,dJ_cm,sigma,dsigma,ddsigma,hq,dhqdq,hsigma,pcm,vcm] = SSDynamics3DData( t,x,thetaLimits,hAlpha,leg,stanceFoot,terrainVector,forceVector,sigmaScale,thetaLimits2,halpha2,allStep )
% Function for computing the single support continuous dynamics using ode
% in MATLAB. Based on rigid 3D model of ATRIAS.
% BAG20140730
% BAG20150510 Updated to work for NHVC 3D.
% BAG20150201 Updated to work with extended hAlpha map.
% BAG20141222 Updated to account for lateral forces on CoM (disturbance).

%% ddq 
q=x(1:9); dq=x(10:18); 
    % etaxHat=x(19); etayHat=x(20);

% Lagrange
[D,Cdq,G,B] = lagrange3D(q,dq,leg);
H = Cdq+G; % Can have more complicated H. (yaw friction and springs)

% Call controller function to find u.
[ u,y,dy,s,ds,theta,dtheta,sigma,dsigma,ddsigma,hq,dhqdq,hsigma,pcm,vcm ] = ... 
    control3D( q,dq,D,B,H,thetaLimits,hAlpha,leg,sigmaScale,thetaLimits2,halpha2,allStep );

% % Add forces to CoM.
% [~, ~, Jac_cm, ~] =  velAccel2D_mex(q,dq,leg);

% Calculate ddq and dx updated to account for forceVector.
% ddq = D\(B*u + -H + Jac_cm'*forceVector); % ddq = D\(B*u + -H);(Previous)
ddq = D\(B*u + -H);
dx = [dq; ddq];
    % dx = [dq; ddq; etaxHatDot; etayHatDoty];

%% Added for data

KE = 1/2*dq'*D*dq;

if leg;
    [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,J_cm,dJ_cm] ...
        = Cfcn_Left_Jacobians(q,dq);
else;
    [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,J_cm,dJ_cm] ...
        = Cfcn_Right_Jacobians(q,dq); end

%% Debugging
% display(terrainVector)
% quickView3D13DOF( q,leg,1 ) % Visualization during simulation.
% pause(0.01)
end

