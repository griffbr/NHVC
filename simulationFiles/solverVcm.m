function [ cost, equality, inequality, step, allStep, poincare ] = solverVcm( robot )
% Creating new solver function for 3D simulations with nonholonomic virtual
% constraints.
% BAG20150401
% Based on previous solver2D.
% BAG20150924 Modified to use Vcm perturbations.

%--------------------------------------------------------------------------
% Initialize Variables.
%--------------------------------------------------------------------------
% allStep
allStep=robot; allStep.stepNum=0; allStep.stepsComplete=0;
allStep.yawControl=0;
allStep = modelParametersAtrias3D(allStep);
% Main loop
allStep.fail=0; 
% Misc.
poincare = []; allStep.integralDirection = 0;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Initial Conditions.
%--------------------------------------------------------------------------
[step, allStep] = initialCondition3D(allStep);
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Pre-impact states.
%--------------------------------------------------------------------------
if allStep.optimization;
% Collect pre-VcmPerturbed states if optimizing.
    [step, allStep] = xMinusPerturbedVcm(step, allStep); end
eventsFunction = @eventsSSDynamics3D; 
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Simulate steps.
%--------------------------------------------------------------------------
allStep.options = odeset('Events', eventsFunction,'MaxStep',0.01,...
    'InitialStep',0.001,'Refine',1,'RelTol',10^-5,'AbsTol',10^-8);
while (~allStep.fail && allStep.stepNum < allStep.numSteps)

    % Loop Variables
    allStep.stepNum = allStep.stepNum+1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulate Hybrid Zero Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Hybrid sim updates discrete parameters through impact map for states
    % control and then simulates using continuous integration to next step.
    [step, allStep] = hybridSim3D(step, allStep);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Process Data (if needed)
    if (~(allStep.poincare) && ~(allStep.fail));
    [step, allStep] = dataProcess3D(step, allStep); end
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Optimization Cost
%--------------------------------------------------------------------------
if allStep.optimization;
[cost, equality, inequality, allStep] = optimization3D(step, allStep);
else cost = 0; equality = 0; inequality = 0; end

% Poincare
if allStep.poincare;
    swap = allStep.swap;
    poincare.x = [swap*step(end).qMinus; swap*step(end).dqMinus];
end
%--------------------------------------------------------------------------

    

end
