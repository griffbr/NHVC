function [qPlus, dqPlus, impF, velFoot] = impact3D(qMinus, dqMinus, leg, yawRestricted)
% Applies impact map to 3D states. Rigid model with constrained yaw.
% Original function from Jessy Grizzle and Kaveh Hamed.
% BAG20140730
% BAG20150509 Updated for NHVC simulator that is yaw constrained.
% BAG20141223 Adding functionality to not have restricted yaw velocity at
% impact.

if nargin<4; yawRestricted = 0; end

y = []; A = []; b = []; qPlus = []; dqPlus = []; impF = []; velFoot = [];
xSt  = 0; ySt  = 0; zSt  = 0; dxSt = 0; dySt = 0; dzSt = 0; % St = Stance
qe   = [ qMinus;  xSt;  ySt;  zSt];
dqe  = [dqMinus; dxSt; dySt; dzSt];

if leg
    [De,E] = Cfcn_ATRIAS3D_ImpactModel_StanceLeft_FixParams(qe); % De 16x16 E 4x16
    % revised by Kaveh to consider symmtery
    swap = [-1 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1;
    0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0];
    S  = [swap zeros(9,3); zeros(3,9) diag([-1 1 1])];
    [DeRight,~] = Cfcn_ATRIAS3D_ImpactModel_StanceRight_FixParams(S*qe);
    De = transpose(S) * DeRight * S;
else
    [De, E] = Cfcn_ATRIAS3D_ImpactModel_StanceRight_FixParams(qe);
end

if ~yawRestricted % Really Unrestricted yaw. (BAG20141223)
        %------------------------------------------------------------------
        E = E(1:3,:);
        A = [De -E'; E zeros(3,3)];
        b = [De*dqe; zeros(3,1)];
        y = A\b;
        qPlus  = qMinus; % no swapping done here
        dqPlus = y(1:9); % If coordinate system changes, 9 will also.
        velFoot = y(10:12);
        impF    = y(13:end);
        %------------------------------------------------------------------
else
% Page 56: Jessy's book, zeros(3,3), here, the momentum at the impact is
% being computed plus the forces at the end of the swing leg.
%--------------------------------------------------------------------------
% Added by Kaveh on 04/21/2012. When we restrict yaw motion of the robot
% about the stance leg end, we decrease the number of DOFs. To consider 
% this point, we should revise the impact model.
if 1 % TRO 2009 Yaw Restricted
        %------------------------------------------------------------------
        A = [De -E'; E zeros(4,4)]; % (3.20), A 20x20
        b = [De*dqe; zeros(4,1)]; % b 20
        y = A\b;
        qPlus  = qMinus; % no swapping done here
        dqPlus = y(1:9); % If coordinate dimensions changes, y references do as well.
        velFoot = y(10:12);
        impF    = y(13:16);
        %------------------------------------------------------------------
else % Different yaw. (Kaveh suggested not using this)
        %------------------------------------------------------------------
        if leg; [Ee,~] = ATRIAS3D_StanceYaw_Left(qMinus,dqMinus); % E 13.
        else; [Ee,~] = ATRIAS3D_StanceYaw_Right(qMinus,dqMinus); end;
        Ee = [Ee zeros(1,3)];
        A  = [De -E' -Ee'; E zeros(4,4) zeros(4,1); Ee zeros(1,4) 0];
        b  = [De*dqe; zeros(4,1); 0];
        y  = A\b;
        qPlus  = qMinus; % no swapping done here
        dqPlus = y(1:9);
        velFoot = y(10:12);
        impF    = y(13:end-1);
        %------------------------------------------------------------------
end
%--------------------------------------------------------------------------
end

% Just for reference
%  {'TorsoXYZ'}  % Standard SS Model
%  qgr1R=q1R; qgr2R=q2R; qgr1L=q1L; qgr2L=q2L;
%  dqgr1R=dq1R; dqgr2R=dq2R; dqgr1L=dq1L; dqgr2L=dq2L;
%  q = [xT; yT; zT; qzT; qyT; qxT; q1R; q2R; q3R; q1L; q2L; q3L];
%  dq= [dxT; dyT; dzT; dqzT; dqyT; dqxT; dq1R; dq2R; dq3R; dq1L; dq2L; dq3L];
%  disp('Building Flight Phase Model without Springs and with Cartesian Frame Attached to the Base of Torso')
end

