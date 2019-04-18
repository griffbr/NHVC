function [ step, allStep ] = continuous3D( step, allStep )
% This function handles the continuous portion of simulation.
% BAG20140730
% BAG20150526 Updated for extended control.
% BAG20150510 Updated to work for NHVC 3D.
% BAG20150201 Updated to work with extended hAlpha.
% BAG20150122 Updated to work for 2D.

% Debugging:

% Initialize variables.
n=allStep.stepNum;
x0=step(n).x0; leg=step(n).leg; stanceFoot=step(n).stanceFoot;
hAlpha=step(n).hAlpha;
tStart=step(n).startTime; thetaLimits=step(n).thetaLimits;
thetaLimits2=step(n).thetaLimits2; hAlpha2=step(n).hAlpha2;
terrainVector=step(n).terrainVector;
forceVector=step(n).forceVector;
tEnd=tStart+allStep.simTime; options=allStep.options;

% Desired heading control (yaw control).
deltaHip = [0; 0];
if (n>2 && allStep.yawControl)
   currentHeading = (step(n-1).finalHeading + step(n-2).finalHeading)/2;
   directionError = currentHeading - allStep.directionVector(n);
   allStep.integralDirection = allStep.integralDirection + directionError;
   kp = 0.05; ki = 0.0003;
   hipGain = kp*directionError + ki*allStep.integralDirection;
   hipGain = min(hipGain,3); hipGain = max(hipGain,-3); % Saturate gains.
   deltaHip = [hipGain; -hipGain];
   fprintf('kp component is %g\n',(kp*directionError))
   fprintf('ki component is %g\n',(ki*allStep.integralDirection))
   fprintf('deltaHip = [%g; %g]\n',deltaHip)
   fprintf('Desired direction is %g (deg)\n', allStep.directionVector(n));
   fprintf('Actual direction is %g (deg)\n', currentHeading);
end

if (size(thetaLimits,1)<2)
   display('test'); 
end

% Simulation of continuous dynamics.
[t1, x1, te, xe] = ode113(@SSDynamics3D, ...
    [tStart, tEnd], x0(1:18), options, thetaLimits, hAlpha, ...
    leg, stanceFoot, terrainVector, forceVector, ...
    allStep.sigmaScale,thetaLimits2,hAlpha2,allStep );

if(size(xe,2)>1) % If simulation finishes.
    % Store data for processing and next step.
    step(n).t1=t1; step(n).x1=x1; step(n).startTime =tStart;
    step(n).stepTime=te-tStart; step(n).complete = 1; step(n).xe = xe';
    allStep.time = te; allStep.stepsComplete = allStep.stepsComplete + 1;
    if (~allStep.optimization | ~(allStep.perturbed(n+1))) % S  
        [step(n+1).stanceFoot, ~, ~, ~, step(n+1).pcm0] = ...
            points3DNoYawPcm( xe(1:9)', step(n).leg, step(n).stanceFoot );
        step(n+1).qMinus=xe(1:9)'; step(n+1).dqMinus=xe(10:18)'; 
        step(n+1).etaxHatMinus = 0; step(n+1).etayHatMinus = 0;
        step(n+1).startTime=te; step(n+1).Perturbed = 0; 
        step(n+1).leg=mod(step(n).leg+1,2); step(n+1).prevLeg = step(n).leg;
        step(n+1).impactUpdate = 1; 
        % Correct for difference between R and L x sign convention.
        if step(n+1).leg; step(n+1).pcm0(1) = -step(n+1).pcm0(1); end;
    end
else; step(n).complete=0; allStep.fail=1; end

end

