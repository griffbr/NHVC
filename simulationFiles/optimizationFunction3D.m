function [ cost, eq, ineq, step, allStep ] = optimizationFunction3D( parameters )
% This function acts as an interface between fmincon and the solver.
% BAG20140816
% BAG20150930 Modified to handle vector perturbations.
% BAG20150511 Updated to work with NHVC 3D.
% BAG20150216 Modified to work with AMBVC
% BAG20141223 Modified to handle external force vector set up.
% BAG20141118 Modified to incorporate Poincare map into optimization.
% BAG20141009 Update to use MIO outputs.
% BAG20141006 Updated for robust impact optimization.
% BAG20141002 Updated to allow the hip output to be optimized as well.
% BAG20140923 Updated to include equality and inequality constraints.
% BAG20140908 Updated for mutliple steps.
% BAG20140909 Shorter simulation time to avoid ode errors during
% optimization for many steps.

%% Misc. Parameters
robot.SFP = 0; % Will need to revisit to enforce ideal behavior.
robot.poincareFocus = 0; % Minimal simulations besides Poincare.
% robot.yawRestricted = 0; % Yaw restricted at impact or not.

% Little trick for optmization tolerances.
% NHVC k1, k2 optimization trick.
parameters([43 46])=parameters([43 46])/10^3;
parameters([44 47])=parameters([44 47])/10^6;
parameters([45 48])=parameters([45 48])/10^9;

% Initialize constant robot variables.
robot.poincare=0;
robot.swap = [-1 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1;
    0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0];
robot.uSwap = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
         1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];

% Simulation variables.
robot.time=0; robot.simTime=2.5; robot.printData=0;
robot.forceVector = repmat([0;0;0],1,10);

%% Disturbance selection

% Example no velocity disturbances
% yBound = [0];
% xBound = [0];
% terrainHeightMax = 0.04; intermediate = 1;
% robot.yawRestricted = 1;
% poincareOptimization = 0;

% Example no terrain disturbances
% yBound = [0.3; 0.15; -0.15; -0.3];
% xBound = [0.15; 0.075; -0.075; -0.15];
% terrainHeightMax = 0; intermediate = 0;
% robot.yawRestricted = 1;
% poincareOptimization = 0;

% Test Large Disturbances
yBound = [0.5; 0.25; -0.25; -0.5];
xBound = [0.3; 0.15; -0.15; -0.3];
terrainHeightMax = 0.08; intermediate = 1;
robot.yawRestricted = 1;
poincareOptimization = 0;
PoincareWeight = 10^4; maxEig = 0.5; % Only relevant if adding Poincare costs.

% Terrain
if intermediate;
robot.terrainVector = [ 0; terrainHeightMax; terrainHeightMax/2; -terrainHeightMax/2; -terrainHeightMax];
robot.periodicStep = 7;
else;
robot.terrainVector = [ 0; terrainHeightMax; -terrainHeightMax];
robot.periodicStep = 5; end   

% Vcm
tempVectorSet = [];
for i=1:length(yBound); tempVectorSet = [tempVectorSet; [0 yBound(i) 0]]; end;
for i=1:length(xBound); tempVectorSet = [tempVectorSet; [xBound(i) 0 0]]; end;
robot.VcmVector = tempVectorSet';

% Uncomment the following if terrain disturbances only
% robot.VcmVector = zeros(3,1);

% Uncomment the following if velocity disturbances only, will need to make
% an additional change in optimization3D.m (see comment there).
% robot.terrainVector = [0; -0.04]; % -0.04 for collecting nominal trajectory data only.
% robot.periodicStep = 3;

if 0 % Plotting for vcm vector.
%%
    figure(24353); clf; hold on; title('Vcm Directions'); grid on;
for i=1:size(robot.VcmVector,2)
    plot3([0 robot.VcmVector(1,i)],[0 robot.VcmVector(2,i)],[0 robot.VcmVector(3,i)],'-X'); end
end

%% Optimization parameters
robot.x0 = parameters;   
robot.optimization = 1; robot.optimizationOutput = 1;
robot.vcmSteps = 3;
robot.numSteps = 2*size(robot.terrainVector,1) + robot.vcmSteps*size(robot.VcmVector,2); % Two steps AFTER vcm perturbation.

if robot.poincareFocus; 
    robot.poincare = 0; robot.terrainVector = 0; robot.periodicStep = round(size(robot.terrainVector,1))+1; robot.VcmVector = [0;0.5;0];
    robot.numSteps = 2*size(robot.terrainVector,1); [ cost, eq, ineq, step, allStep, poincare ] = solverVcm(robot); robot.poincare=1;
else; cost=10^12;
[ cost, eq, ineq, step, allStep, poincare ] = solverVcm(robot); end

%--------------------------------------------------------------------------
%% Poincare Section (optional)
%--------------------------------------------------------------------------
robot.poincare = poincareOptimization; % Poincare on (1) or off (0).
if(robot.poincare)
[ ~, ~, ~, ~, ~, eigVector ] = solverPoincare(robot, 0.001);
%%
% Take out yaw eigen value (if exists)
    yawIdx = find(eigVector>0.9999 & eigVector<1.00001); eigVector(yawIdx)=0;
% Calculate costs
    poincareIneq = PoincareWeight*(eigVector - maxEig*ones(size(eigVector)));
    poincareIneqLowerBound = ...
        max([poincareIneq zeros(size(poincareIneq))],[],2);
        % ineq = [ineq; poincareIneq]; % Make Poincare eigenvalues an inequality constraint.
    allStep.poincareCost = max(poincareIneqLowerBound.^2); % Focus on maximum eigen value only.
    allStep.maxEigValue = max(eigVector);
%%
cost = cost + allStep.poincareCost; % Add cost for positive Poincare values.
else; allStep.poincareCost = 0; allStep.maxEigValue = 0; end;
%--------------------------------------------------------------------------
if isnan(cost) % If cost is NaN
    cost = 10^12; end
if 0; 
    %%
    fprintf('%% Max Eig. is %g, PoincareCost is %g\n',max(eigVector),allStep.poincareCost); 
end

end

