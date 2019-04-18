function [step, allStep] = xMinusPerturbedVcm(step, allStep);
% The purpose of this function is to collect all of the perturbed
% pre-impact states for robust impact optimization.
% BAG20141003.
% BAG20150924 Updated to collect pre-VcmPerturbed states at mid step.
% BAG20150719 Updated to work with ROLO.
% BAG20150526 Updated to work for extended control.
% BAG20150511 Updated to work for 3D NHVC.
% BAG20150218 Updated to work with 2D AMBVC optimization.

%% Initialization

% Assume odd steps left stance, even steps right stance.
    step(1).leg=1; step(1).prevLeg=0; step(1).stanceFoot=[0;0;0];
    step(1).hAlpha=allStep.hAlpha; step(1).hAlpha2=allStep.hAlpha2;
    step(1).thetaLimits=allStep.thetaLimits;
    step(1).thetaLimits2=allStep.thetaLimits2;
    step(1).startTime=0; step(1).terrainVector=0; step(1).forceVector=[0;0;0];
    
% Initialize variables.
    x0=step(1).x0; leg=step(1).leg; stanceFoot=step(1).stanceFoot;
    hAlpha=step(1).hAlpha; hAlpha2=step(1).hAlpha2;
    thetaLimits=step(1).thetaLimits; thetaLimits2=step(1).thetaLimits2;
    tStart=step(1).startTime; tEnd=tStart+allStep.simTime;
    terrainVector=allStep.terrainVector; forceVector = [0;0;0]; 
    allStep.perturbed = zeros(1,allStep.numSteps-1);

%% Simulate step
% Collect all pre-impact conditions and periodic behavior.

% Simulation of continuous dynamics.
    options = odeset('Events', @eventsSSDynamics3DRIOxMinus,...
        'MaxStep',0.01,'InitialStep',0.001,'Refine',1,'RelTol',10^-5,...
        'AbsTol',10^-8);
    [t1, x1, te, xe] = ode113(@SSDynamics3D, ...
        [tStart, tEnd], x0(1:18), options, thetaLimits, hAlpha, ...
        leg, stanceFoot, terrainVector, forceVector, ...
        allStep.sigmaScale, thetaLimits2, hAlpha2, allStep );
    
% Need to properly store xMinus conditions for terrain disturbances.
    if(size(xe,1)==length(allStep.terrainVector)) % If all impact conditions found.
        % Store data from nominal step
        step(1).t1=t1; step(1).x1=x1; step(1).startTime =tStart;
        step(1).stepTime=te(end)-tStart;
        step(1).complete = 1; allStep.stepsComplete = 1;
        step(1).initialHeading = x0(1); step(1).Perturbed = 0;
        allStep.stepNum=allStep.stepNum+1; allStep.numSteps=allStep.numSteps+1;
        % xMinus data for simulating perturbed steps.
        for i=1:size(terrainVector,1); n=2*i; % Updated to take two steps.
        step(n).startTime = te(i);
        [step(n).stanceFoot, ~, ~, step(n).pcm0 ] = ...
                points3D( xe(i,1:9)', step(1).leg, step(1).stanceFoot);
        step(n).qMinus=xe(i,1:9)'; step(n).dqMinus=xe(i,10:18)';
        step(n).Perturbed = 1; step(n).leg=0; step(n).prevLeg = 1;
        step(n).impactUpdate = 1; allStep.perturbed(n) = 1;
        end
    else; step(1).complete=0; allStep.stepsComplete = 0; allStep.fail=1; end

%% Vcm Perturbations

% Initial Conditions require periodic data.
if ~allStep.fail;
[step, allStep] = dataReconstruction3D(step, allStep, 1);
% Find perturbed states for changes in Vcm.
[qPerturbed, dqPerturbed, perturbed] = deltaVcm(step, allStep);
% xMinus data for simulating perturbed steps.
stepsAfterVcm = allStep.vcmSteps; % Including second half of perturbed step.
for i=1:size(qPerturbed,2); n=stepsAfterVcm*(i-1) + 2 + 2*(size(terrainVector,1)); % Updated to take two steps.
    step(n).startTime = perturbed.initialTime;
    step(n).stanceFoot = perturbed.stanceFoot;
    step(n).qMinus = qPerturbed(:,i); step(n).dqMinus = dqPerturbed(:,i);
    step(n).qPlus = qPerturbed(:,i); step(n).dqPlus = dqPerturbed(:,i);
    step(n).Perturbed = 1; step(n).leg=1; step(n).prevLeg = 0; 
    step(n).impactUpdate = 0; % step(n).impactUpdate = 0; 
    step(n).impF = step(1).impF; step(n).pcm0 = step(1).pcm0;
    step(n).x0=[step(n).qPlus; step(n).dqPlus]; % May need to add additional coordinates.  
    allStep.perturbed(n) = 2;
end

for i=1:stepsAfterVcm-1; allStep.perturbed = [allStep.perturbed 0]; end;
allStep.perturbed = [allStep.perturbed 2];
allStep.numSteps = length(allStep.perturbed)-1; 
end

% % Currently in progress
% display('test')

end