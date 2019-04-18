function [ cost, equality, inequality, step, allStep, poincare ] = solverPoincare( robot, epsilon )
% Creating new solver function for 3D simulations with nonholonomic virtual
% constraints.
% BAG20150401
% Based on previous solver2D.
% BAG20150924 Modified to use Vcm perturbations.

% Misc.
cost = 0; equality = 0; inequality = 0; 

%--------------------------------------------------------------------------
% Initialize Variables.
%--------------------------------------------------------------------------
% allStep
robot.terrainVector = zeros(1000,1); robot.optimization = 1;
robot.perturbed = zeros(size(robot.terrainVector));
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
prevXMinus = [step(end).qMinus; step(end).dqMinus];
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Find periodic step
%--------------------------------------------------------------------------
stepDiffThreshold = 0.000001; 
stepDiffThreshold = 0.0001;
stepDiffThreshold = 0.0005;
% stepDiffThreshold = 0.001;
stepDiffTotal = stepDiffThreshold + 0.001;

stepLimit = 200;
stepDiffMax = 0.3;

eventsFunction = @eventsSSDynamics3D; 
allStep.options = odeset('Events', eventsFunction,'MaxStep',0.001,...
    'InitialStep',0.001,'Refine',1,'RelTol',10^-5,'AbsTol',10^-8);
while (stepDiffTotal > stepDiffThreshold && allStep.stepNum<=stepLimit ...
        && stepDiffTotal < stepDiffMax)
    allStep.stepNum = allStep.stepNum+1;
    n = allStep.stepNum;
    if mod(n,2); step(n).leg=1; step(n).prevLeg=0;
    else step(n).leg=0; step(n).prevLeg=1; end; step(n).impactUpdate = 1;
    [step, allStep] = hybridSim3D(step, allStep);
    if ~step(end).leg; swap = allStep.swap; else; swap = eye(9); end
    currentXMinus = [swap*step(end).qMinus; swap*step(end).dqMinus];
    currentXMinus(1) = 0; % Don't consider yaw.
    stepDiffTotal = norm( currentXMinus - prevXMinus );
    prevXMinus = currentXMinus;
if allStep.printData; fprintf('\nstepDiffTotal is %g\n',stepDiffTotal); end
end

if stepDiffTotal > stepDiffMax;
    poincare = 1 + 10*(stepDiffTotal - stepDiffThreshold); return; end;

periodicXMinus = [step(end).qMinus; step(end).dqMinus]; periodicXMinus(1) = 0;
% periodicXMinus = currentXMinus;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Find epsilon differences (one step, corrected impact map use)
%--------------------------------------------------------------------------
if 1
allStep.stepNum = allStep.stepNum+1;
n = allStep.stepNum;   
% Calculate epsilon differences for each state besides yaw.
swap = allStep.swap;
dPdx=zeros(18,18); poincarePlusOnly = 0;
for i=[1:18]; % Do not need to consider yaw eigenvalue.
        tempVector=zeros(size(periodicXMinus)); tempVector(i)=epsilon;
        allStep.stepNum = n;
        currentXMinus = periodicXMinus + tempVector;
        step(n).qMinus = currentXMinus(1:9); step(n).dqMinus = currentXMinus(10:18);
    % Simulate step
        step(n).impactUpdate = 1; [step, allStep] = hybridSim3D(step, allStep);
        xPlus = [swap*step(n+1).qMinus; swap*step(n+1).dqMinus];
    if ~poincarePlusOnly % Decides how to calculate Poincare map.
        allStep.stepNum = n;
        currentXMinus = periodicXMinus - tempVector;
        step(n).qMinus = currentXMinus(1:9); step(n).dqMinus = currentXMinus(10:18);
        % Simulate step
            step(n).impactUpdate = 1; [step, allStep] = hybridSim3D(step, allStep);
            xMinus = [swap*step(n+1).qMinus; swap*step(n+1).dqMinus];
            dPdx(:,i) = [( xPlus - xMinus )/(2*epsilon)]; 
    else; dPdx(:,i) = [( xPlus - periodicXMinus )/(epsilon)]; end
end
if (sum(sum(isnan(dPdx)+isinf(dPdx)))>0.1) % Do not check eigVector if undefined.
    eigVector = 100*ones(size(dPdx,1),1);
else; eigVector = abs(eig(dPdx)); end;  

%%
poincare = eigVector;
if allStep.printData; fprintf('%% Max Eig.s are %0.3g, %0.3g, and %0.3g, yaw restricted = %d,\n%%    calculated with epsilon of %g and stepDiff of %g and %d step limit\n',eigVector(1:3),allStep.yawRestricted,epsilon,stepDiffTotal,stepLimit); end   

end
