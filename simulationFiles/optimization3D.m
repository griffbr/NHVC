function [ cost, equality, inequality, allStep, step ] = optimization3D( step, allStep )
% This function is used to take step and allStep data and feedback a cost
% for optimization. In this case, optimization is for flat ground,
% periodic, 3D walking.
% BAG20140815
% BAG20150511 Updated to work for NHVC 3D.
% BAG20150309 Switching to normalized SFP errors.
% BAG20150219 Updated to work with AMBVC optimization.
% BAG20141003 Added robust impact optimization cost.
% BAG20140930. Added energy optimization cost.
% BAG20140923. Updated to include equality and inequality constraints.
% BAG20140909. Updated to calculate cost based on distance traveled.
% BAG20140908. Updated to calculate periodic cost based on multiple steps.

% Choose number of steps that must be complete for evaluation.
stepNum=allStep.numSteps;
    
% Cost and Constraints.
if (allStep.stepsComplete==stepNum) % Ensure simulations are finishing.
    
%% Robust Control Optimization
    if ~allStep.poincareFocus
        temp = []; % Index which steps to calculate RIO costs on.
        for n=2:stepNum; if step(n).impactUpdate; temp = [temp; n]; end; end
        
        % Uncomment the following if using velocity disturbances only.
%         temp = temp([1:2 5:end]);
        
    allStep.RIOIdx = temp;
    [step, allStep] = tauIntegralError(step, allStep);
    xWeight = 1; uWeight = 1;
    RIOCost = 0; nPrev = 0;
    for i=1:length(allStep.RIOIdx);
        n = allStep.RIOIdx(i); scale = 1;
        if (n - nPrev == 1); 
            scale = 3; 
        else; nPrev = n; end % Scale cost if not directly following perturbation.
        RIOCost = RIOCost + scale*(xWeight*step(n).xErrorIntegral ...
         + uWeight*step(n).uErrorIntegral);
    end; else; RIOCost = 0; end
    
%% Equality Constraints
% Perdiodic Ground
periodicGroundWeight = 1500; % was 300000, trying to free up xMinus
periodicGround = periodicGroundWeight* ...
    (allStep.initialSwingFootHeight - step(1).stanceFoot(3));
equality = periodicGround;
% Xminus comparison. Periodic motion for periodic step only.
equalityVector = ...
    equalityConstraints3D(step, allStep, allStep.periodicStep, 1);
equality = [equality; equalityVector];
% Check for first step after 0 cm perturbation as well:
equalityVector = ...
    equalityConstraints3D(step, allStep, allStep.periodicStep-1, 1);
equality = [equality; equalityVector];

%% Inequality Constraints

inequality=[]; for i=1:stepNum; % Check constraint for all data.
    if ~(allStep.perturbed(i) == 2); % No inequality constraints for initial vcm perturbed steps.
        [inequalityVector, step] = inequalityConstraints3D(step, allStep.printData, i, allStep.periodicStep);
        % Added inequality costs for nominal walking.
        if ( i==allStep.periodicStep || (i == (allStep.periodicStep - 1))); 
            inequalityVector = inequalityVector*1;
        else; inequalityVector = inequalityVector*0.5; end
        inequality = [inequality; inequalityVector]; end
end

%% Energy cost
energyWeight = 2000; % energyWeight = 500;
energyCost = energyWeight*...
    sum(step(allStep.periodicStep).energy)/step(allStep.periodicStep).stepDistance;

%% Calculate final cost function
% RIOCost = 0; 
inequalityLowerBound = max([inequality zeros(size(inequality))],[],2);
allStep.RIOCost = RIOCost; allStep.energyCost = energyCost; 
allStep.equalityCost = sum(equality.^2); allStep.inequalityCost = sum(inequalityLowerBound.^2);

cost = allStep.RIOCost + allStep.energyCost + allStep.equalityCost + allStep.inequalityCost;

else % Else, provide a prohibitively high price.
    cost = 10^15*(stepNum-allStep.stepsComplete);
    equality = cost*ones((1+18*1),1);
    equality = cost*ones(109,1);
    inequality = cost*ones((10*(6)),1);
end

% Debugging
if 0
    %% Basic cost information.
    fprintf('%% RIOCost: %g, equalityCost: %g, inequalityCost: %g, energyCost: %g\n',RIOCost,sum(equality.^2),sum(inequalityLowerBound.^2),energyCost)
    %% Specific inequality information.
    inequalityDisplay(step,10,5,allStep.periodicStep)    
    %% RIO Error specifics
    RIOCostDisplay(step,allStep,10)
end

end

