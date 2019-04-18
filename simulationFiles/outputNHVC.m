function [sOut,ds,theta,dtheta,h0,y,dy,dh0dq,dhqdq,phipyRange,dsdq,d2h0dq2_qdot2,ddhsigmaddsigma_sigmadot2,dhsigmadsigma,sigma,dsigma,ddsigma,hsigma,pcm,vcm,d2hdesdq2_qdot2] = outputNHVC_ijrr( q, dq, leg, phipyLimits, halpha, k, phipyLimits2, halpha2, cp)
% This function is designed to provide control outputs in a fashion more
% like the experimental controller, but have outputs exactly matched to
% rigid 9 link model.
% BAG201505
% BAG20150928 Updated to use 3D gait phase.
% BAG20150908 Updated to work with virtual springs and new outputs.
% BAG20150725 Updated with new control strategy.
% BAG20150720 Updated to control relative stance hip angle.
% BAG20150716 Updated to work with new PHip NHVC outputs designed today.
% BAG20150522 Updated to work with extended Bezier.
% BAG20150519 Updated to use angular mometum exactly instead of calculating
% through D.

%% q
    q(1) = 0; % Zero yaw for control.
    % Swap q if necessary.
    if leg % If left leg.
        % Need a swap from L to R.
        % q=[qzT; qyT; qxT; q1R; q2R; q3R; q1L; q2L; q3L];
        swap = [-1 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
                0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1;
                0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0];
        % Find swapped coordinates.
        q=swap*q; dq=swap*dq; xSgn = -1;
    else; swap = zeros(9,9); xSgn = 1; end;
    % Assign q and dq Rigid to standard coordinate symbolic values used below.
    qzT=q(1);    dqzT=dq(1);
    qyT=q(2);    dqyT=dq(2);
    qxT=q(3);    dqxT=dq(3);
    q1R=q(4);    dq1R=dq(4);
    q2R=q(5);    dq2R=dq(5);
    q1L=q(7);    dq1L=dq(7);
    q2L=q(8);    dq2L=dq(8);
    qgr1R=q(4);  dqgr1R=dq(4);
    qgr2R=q(5);  dqgr2R=dq(5);
    q3R=q(6);    dq3R=dq(6);
    qgr1L=q(7);  dqgr1L=dq(7);
    qgr2L=q(8);  dqgr2L=dq(8);
    q3L=q(9);    dq3L=dq(9);

    q9Link = q; dq9Link = dq;
    q = [qzT; qyT; qxT; q1R; q2R; q1L; q2L; qgr1R; qgr2R; q3R; qgr1L; qgr2L; q3L];
    dq = [dqzT; dqyT; dqxT; dq1R; dq2R; dq1L; dq2L; dqgr1R; dqgr2R; dq3R; dqgr1L; dqgr2L; dq3L];

    pcm = zeros(3,1); vcm = zeros(3,1);
    
%% qbar

L2 = 0.5; L4 = 0.5;

% 160330 cpq3R*q3R - qyT
% qbar = [phipy;
%         qKAST;
%         qLAST;
%         cpq3R*q3R - qyT;
%         q3LAbs;
%         qLASW;
%         qKASW];

cpq3R = cp(1);

qbar = [L2*sin(q2R + qxT) + L4*sin(q1R + qxT);
    qgr2R - qgr1R;
    qgr1R/2 + qgr2R/2;
    cpq3R*q3R - qyT;
    q3L - qyT;
    qgr1L/2 + qgr2L/2;
    qgr2L - qgr1L];

dqbardq = [0,    0,    L2*cos(q2R + qxT) + L4*cos(q1R + qxT),    L4*cos(q1R + qxT),    L2*cos(q2R + qxT),    0,    0,    0,    0,    0,    0,    0,    0;
    0,    0,    0,    0,    0,    0,    0,    -1,    1,    0,    0,    0,    0;
    0,    0,    0,    0,    0,    0,    0,    1/2,    1/2,    0,    0,    0,    0;
    0,    -1,    0,    0,    0,    0,    0,    0,    0,    cpq3R,    0,    0,    0;
    0,    -1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1/2,    1/2,    0;
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    -1,    1,    0];

d2qbardq2_qdot2 = [- dqxT*(dqxT*(L2*sin(q2R + qxT) + L4*sin(q1R + qxT)) + L2*dq2R*sin(q2R + qxT) + L4*dq1R*sin(q1R + qxT)) - dq1R*(L4*dq1R*sin(q1R + qxT) + L4*dqxT*sin(q1R + qxT)) - dq2R*(L2*dq2R*sin(q2R + qxT) + L2*dqxT*sin(q2R + qxT));
    0;
    0;
    0;
    0;
    0;
    0];

d2h0dq2_qdot2 = d2qbardq2_qdot2(2:end);

%% Sigma (for NHVC)
    % Use theoretical calculation for sigma (with coordinate and sign convention corrections)
    [ sigma, dsigma, ddsigma ] = sigmaCalc( q9Link, dq9Link );
    kexp = 30;
    if sigma(2)>0
        ddsigma(2) = ddsigma(2) * exp(-kexp/sigma(2)^2) * (2*kexp/sigma(2)^2 + 1) + ...
            dsigma(2)^2 * exp(-kexp/sigma(2)^2) * ((4*kexp^2)/sigma(2)^5 - (2*kexp)/sigma(2)^3);
        dsigma(2) = dsigma(2) * exp(-kexp/sigma(2)^2) * (2*kexp/sigma(2)^2 + 1);
        sigma(2) = sigma(2) * exp(-kexp/sigma(2)^2);
    else; sigma(2) = 0; dsigma(2) = 0; ddsigma(2) = 0; end;

    sigmax = sigma(1); sigmay = sigma(2); dsigmax = dsigma(1); dsigmay = dsigma(2);
         
%% Mechanical phase variable calculations.
    theta = qbar(1); dthetadq = dqbardq(1,:); dtheta = dthetadq*dq;
    if theta<=phipyLimits(2) % Check to see if extended Bezier should be used.
        sOutput = 0; useExtHalpha = 0;
    else; phipyLimits = phipyLimits2; useExtHalpha = 1; sOutput = 1; end
    phipyRange = phipyLimits(2)-phipyLimits(1);
    dsdtheta = 1/phipyRange;
    s = (theta - phipyLimits(1))/phipyRange;
    ds = dsdtheta*dtheta;

%% hd,sigma (3rd order)
    k1x = k(1); k2x = k(2); k3x = k(3); k1y = k(4); k2y = k(5); k3y = k(6);
    hsigma = zeros(6,1); dhsigmadsigma = zeros(6,3); ddhsigmaddsigma_sigmadot2 = zeros(6,1);
    % qLASW based on sagittal:
    hsigma(5) = k1x*sigmax + k2x*sigmax^2 + k3x*sigmax^3;
    dhsigmadsigma(5,1) = k1x + 2*k2x*sigmax  + 3*k3x*sigmax^2;
    ddhsigmaddsigma_sigmadot2(5) =  dsigmax^2*(2*k2x + 6*k3x*sigmax);
    % qHIPSW based on frontal:
    hsigma(4) = k1y*sigmay + k2y*sigmay^2 + k3y*sigmay^3;
    dhsigmadsigma(4,2) = k1y + 2*k2y*sigmay + 3*k3y*sigmay^2;
    ddhsigmaddsigma_sigmadot2(4) =  dsigmay^2*(2*k2y + 6*k3y*sigmay);

%% Calculate Bezier
if ~useExtHalpha
    bez = bezier(halpha, s);
    dhdesds = bezierd(halpha, s); % Used more than once, use variable to store.
    beza = beziera(halpha, s);
else; bez = bezier(halpha2, s); dhdesds = bezierd(halpha2, s); beza = beziera(halpha2, s); end

%% Calculate y and dy.
    h0 = qbar(2:end); dh0dq = dqbardq(2:end,:);
    y = h0 - hsigma - bez;
    dy = dh0dq*dq - dhsigmadsigma*dsigma - dhdesds*ds;

%% u partial derivatives. (see derivation on BAG20150127 page 24)
    dsdq = dsdtheta*dthetadq;
    dhqdq = dh0dq; % - dhdesdq;
    d2hdesdq2_qdot2 = beza*(dsdq*dq)^2;

%% Transform dhdq for 9DOF simulator.
    dhqdq = dhqdq(:,[1 2 3 8 9 10 11 12 13]);
    dh0dq = dh0dq(:,[1 2 3 8 9 10 11 12 13]);

%% Output of function.
    % Constant decoupling in optimization:
    if leg % Swap outputs as needed.
        dhqdq = dhqdq*swap; dh0dq = dh0dq*swap; end
    sOut = s + sOutput;   

end

