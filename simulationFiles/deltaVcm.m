function [qPerturbed, dqPerturbed, perturbed] = deltaVcm(step, allStep);
% The purpose of this function is to take a nominal state of the robot and
% return the states corresponding to a series of perturbed CoM velocitires
% from the nominal state.
% BAG20150927

pcm0Idx = find(step(1).pcm(2,:)>0,1,'first');
    % pcm0Idx = 1; % Use xPlus as perturbation point instead of middle of step.
vcmNom = step(1).vcm(:,pcm0Idx);
allStep.vcmOfVertPcmNom = vcmNom;
dqNom = step(1).dq(:,pcm0Idx);
qNom = step(1).q(:,pcm0Idx);

% vcmDes = [0; 0.6; 0];
% vcmDes = allStep.VcmVector';
vcmDes = allStep.VcmVector;

deltaVcm = vcmDes; % Vcm Vector is the delta.
    % deltaVcm = vcmDes - vcmNom*ones(1,size(vcmDes,2));

if step(1).leg;
    [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,dpcmdq,dJ_cm] ...
        = Cfcn_Left_Jacobians(qNom,dqNom);
else;
    [Jac_wStance,dJac_wStance,Jac_wSwing,dJac_wSwing,dpcmdq,dJ_cm] ...
        = Cfcn_Right_Jacobians(qNom,dqNom); end

% deltaVcm = dpcmdq*deltadq
deltadq = dpcmdq\deltaVcm;
% If A is a rectangular m-by-n matrix with m ~= n, and B is a matrix with m rows,
% then A\B returns a least-squares solution to the system of equations A*x= B.

dqPerturbed = dqNom*ones(1,size(vcmDes,2)) + deltadq;
qPerturbed = qNom*ones(1,size(vcmDes,2));
perturbed.initialTime = step(1).t(pcm0Idx);
perturbed.stanceFoot = step(1).stanceFoot;
perturbed.etaxHatMinus = 0;
perturbed.etayHatMinus = 0;
perturbed.etaxHatPlus = 0;
perturbed.etayHatPlus = 0;