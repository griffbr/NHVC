function [ equalityVector ] = equalityConstraints3D( step, allStep, num1, num2 )
% The purpose of this function is to calculate the equality
% constraints after data processing for two steps.
% BAG20140923
% BAG20150511 Updated to work with NHVC 3D. Adding cost to cause symmetric
% lateral velocity of pcm at end of step.
% BAG20150219 Updated to work with AMBVC.
% BAG20150105 Modified to compare steps based on step.leg and not number.
% BAG20141224 Modified to handle kinetic energy comparisons.
% BAG20140930. Modified to send entire q and dq equality vector as opposed
% to the norm of each.
% BAG20140925. Modified to handle same or different stance leg comparisons.

% Constraint parameters.
qWeight = 5000; dqWeight = 2500;
KEWeight = 0; 
VcmWeight = 0;
lateralVelocityWeight = 0; % Need to reel in asymmetry in left and right steps.


%--------------------------------------------------------------------------
% Equality constraints.
%--------------------------------------------------------------------------
% Compare steps with weights.

% Compare first step with last step.
if (step(num1).leg~=step(num2).leg); % Swap needed if one step is l and one is r.
    swap = allStep.swap;
    qCost = step(num1).qMinus-swap*step(num2).qMinus;
    dqCost = step(num1).dqMinus-swap*step(num2).dqMinus;
    lateralVelocityCost = step(num1).vcm(1,end) + step(num2).vcm(1,end);
else % Compare steps without swap.
    qCost = step(num1).qMinus-step(num2).qMinus;
    dqCost = step(num1).dqMinus-step(num2).dqMinus;
    lateralVelocityCost = step(num1).vcm(1,end) - step(num2).vcm(1,end);
end
% Compare Kinetic Energy Minus Conditions.
KECost = step(num1).KEMinus - step(num2).KEMinus;
VcmCost = step(num1).avgCoMVelocity(2) - step(num2).avgCoMVelocity(2);

if allStep.printData
   fprintf('Differences in xMinus between step %d and step %d\n', ...
       num1, num2); 
   fprintf('qMinus Difference: [%g %g %g %g %g %g %g %g %g]\n', qCost)
   fprintf('dqMinus Difference: [%g %g %g %g %g %g %g %g %g]\n', dqCost)
   fprintf('KE Minus Difference: %g\n', KECost);
end

qCost = qWeight * qCost;
dqCost = dqWeight * dqCost;
KECost = KEWeight * KECost;
VcmCost = VcmWeight * VcmCost;
lateralVelocityCost = lateralVelocityWeight * lateralVelocityCost;

equalityVector = [qCost; dqCost];

% q=[qzT; qyT; qxT; q1LinkR; q2LinkR; q1LinkL; q2LinkL; q1R; q2R; q3R; q1L; q2L; q3L];


end

