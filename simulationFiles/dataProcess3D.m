function [ step, allStep ] = dataProcess3D( step, allStep )
% This is the highest level function for data processing post hybrid
% simulation.
% BAG20140806
% BAG20150510 Updated for NHVC 3D.
% BAG20150122 Updated to work for 2D case.

n=allStep.stepNum;

% Reconstruct information.
if step(n).complete
[step, allStep] = dataReconstruction3D(step, allStep, n); end

% Print out information.
if allStep.printData
if allStep.fail;
    fprintf('\nStep Number %i did not complete within %g seconds.\n',...
        n, allStep.simTime)
else
    fprintf('\nStep Number %i\n',n)
    if step(n).leg; fprintf('Left Stance\n');
    else; fprintf('Right Stance\n'); end
    fprintf('Previous step height is %g (m)\n', step(n).prevStepHeight)
    fprintf('Current  step height is  %g (m)\n', step(n).stepHeight)
    fprintf('Stance Foot at [%g; %g; %g]\n', step(n).stanceFoot)
    fprintf('Initial heading is            %g (deg)\n', ...
        step(n).initialHeading)
    fprintf('Change in heading is          %g (deg)\n', ...
        step(n).deltaHeading)
    fprintf('Two step change in heading is %g (deg)\n', ...
        step(n).twoStepDeltaHeading)
    fprintf('Turn diameter is %g (m)\n', step(n).turnDiameter)
    fprintf('qFinal = [%g; %g; %g; %g; %g; %g; %g; %g; %g];\n',...
        step(n).x1(end,1:9))
    fprintf('dqFinal = [%g; %g; %g; %g; %g; %g; %g; %g; %g];\n',...
        step(n).x1(end,10:18))
    fprintf('Pcmx min and max are [%g; %g]\n',step(n).thetaLimits) 
    fprintf('Impact losses from previous step are %g (J)\n',...
        step(n).impactLoss)
    fprintf('KE Minus is %g (J)\n', step(n).KEMinus)
    fprintf('KE End is %g (J)\n', step(n).KE(end))
    fprintf('\n%% Average CoM velocity is [%.3g; %.3g; %.3g] (m/s) at %.3g (s/step)\n',...
        step(n).avgCoMVelocity,step(n).stepTime)
    fprintf('%% Vcm at vertical is [%.3g; %.3g; %.3g] (m/s)\n',step(n).vcmVertical);
    fprintf('%% Final Swing Foot Velocity is [%.3g; %.3g; %.3g] m/s\n',step(n).swingFootVelFinal);
    fprintf('%% Impact losses are %.4g J\n',step(n).impactLoss);
    fprintf('%% Swing foot max is %.3g m\n',max(step(n).swingFoot(3,:)));
    fprintf('%% Sigma max: [%.3g; %.3g; %.3g], min: [%.3g; %.3g; %.3g];\n',max(step(n).sigma'),min(step(n).sigma'));
    fprintf('%% Step distance [%.3g; %.3g; %.3g]; (yaw zeroed out)\n',step(n).xyzDistanceNoYaw);
    fprintf('%% COT  is %g\n',step(n).COT);
    fprintf('%% MCOT is %g\n',step(n).MCOT);
end
end

end

