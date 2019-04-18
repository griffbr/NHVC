function [ u,y,dy,s,ds,theta,dtheta,sigma,dsigma,ddsigma,h0,dhqdq,hsigma,pcm,vcm ] = control3D( q,dq,D,B,H,pcmxLimits,halpha,leg,k,thetaLimits2,halpha2,allStep )
% This is a function which takes the state of the robot and returns the
% control output u. Current work uses feedback control from desired
% outputs that are a function of planar s or theta.
% BAG20140805
% BAG20150901 Updated to use virtual spring.
% BAG20150719 Updated for ROLO.
% BAG20150510 Updated for NHVC 3D.
% BAG20150216 Updated for AMBVC.
% BAG20150201 Updated to work with extended hAlpha.
% BAG20150127 Updated to use external function to provide partial
% derivative for calculating u.
% BAG20150122 Updated to work for 2D control.

% Initialize variables
% epsilon = 0.003;
epsilon = 0.05;
% epsilon = 0.1;
kpn = 1; %(wn^2)
kdn = 1.8; % 2*zeta*wn
kp = kpn/epsilon^2;
kd = kdn/epsilon;

% Collect partial derivatives for control calculation.
if allStep.optimizationOutput;
[s,ds,theta,dtheta,h0,y,dy,dh0dq,dhqdq,~,~,d2h0dq2_qdot2,ddhsigmaddsigma_sigmadot2,...
    dhsigmadsigma,sigma,dsigma,ddsigma,hsigma,pcm,vcm,d2hdesdq2_qdot2] = ...
    outputNHVC_mex( q,dq,leg,pcmxLimits,halpha,k,thetaLimits2,halpha2,allStep.cp );
else
[s,ds,theta,dtheta,h0,y,dy,dh0dq,dhqdq,~,~,d2h0dq2_qdot2,ddhsigmaddsigma_sigmadot2,...
    dhsigmadsigma,sigma,dsigma,ddsigma,hsigma,pcm,vcm,d2hdesdq2_qdot2] = ...
    outputNHVC( q,dq,leg,pcmxLimits,halpha,k,thetaLimits2,halpha2,allStep.cp );
end

% Calculate u.
[ u ] = calculateU( dhqdq,D,B,H,d2h0dq2_qdot2,d2hdesdq2_qdot2,dhsigmadsigma,ddsigma,ddhsigmaddsigma_sigmadot2,kp,y,kd,dy );

% Saturate u to 6Nm for legs and 3Nm for hips.
% u = [u1R; u2R; u3R; u1L; u2L; u3L];
u([1 2 4 5]) = min(u([1 2 4 5]),4); u([1 2 4 5]) = max(u([1 2 4 5]),-4);
u([3 6]) = min(u([3 6]),2); u([3 6]) = max(u([3 6]),-2);

end

