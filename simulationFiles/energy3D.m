function [ KE, PE, KEPendulum, PEStance ] = energy3D( allStep, q, dq, leg, stanceFoot )
% This is a function that given the current state of the robot, will return
% multiple perspectives of potential and kinetic energy.
% BAG20141218

% Initial data.
mTotal=allStep.mTotal; g=allStep.g;
[D,Cdq,G,B] = lagrange3D(q,dq,leg);
% Leg specific calculations
if leg;
    [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,...
        Jac_cm,dJac_cm] = Cfcn_Left_Jacobians(q,dq);
    [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
        Cfcn_ATRIAS3D_Primary_PointsLeft(q);
else; [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,...
            Jac_cm,dJac_cm] = Cfcn_Right_Jacobians(q,dq);
        [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
            Cfcn_ATRIAS3D_Primary_PointsRight(q);  end

% KE Pendulum
vcm = Jac_cm*dq;
KEPendulum = 1/2*mTotal*norm(vcm)^2;
% KE Total
KE = 1/2*dq'*D*dq;
% PE Absolute
PE = mTotal*g*(pcm(3,:)+stanceFoot(3));
% PE Relative
PEStance = mTotal*g*(pcm(3,:));

end

