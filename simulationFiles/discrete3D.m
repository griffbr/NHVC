function [ step, allStep ] = discrete3D( step, allStep )
% This function handles the discrete portion of simulation.
% This includes taking a set of minus step conditions and updating
% both control and states through the 'extended' impact map.
% BAG20140730
% BAG20150526 Updated for extended control.
% BAG20150510 Updated to work with NHVC 3D.
% BAG20150216 Updated to work with AMBVC, no more thetaMin reset.
% BAG20150128 Updated to work with angular momentum outputs.
% BAG20150122 Updated to work in 2D.

% Notes: hAlphaReset can be mexed. Currently control is limited to a single
% bezier polynomial (no extended).

%% Initialize variables.
n=allStep.stepNum;
step(n).hAlpha2=allStep.hAlpha2;

% Assume odd steps left stance, even steps right stance. Can change if
% necessary.
if ~allStep.optimization
if mod(n,2); step(n).leg=1; step(n).prevLeg=0;
else step(n).leg=0; step(n).prevLeg=1; end; step(n).impactUpdate = 1; end;

if step(n).impactUpdate

%% Physical impact map.
[step(n).qPlus, step(n).dqPlus, step(n).impF, step(n).velFoot] = ...
    impact3D(step(n).qMinus, step(n).dqMinus, step(n).prevLeg, allStep.yawRestricted);
step(n).x0=[step(n).qPlus; step(n).dqPlus]; % May need to add additional coordinates.

%% Update thetaMin
if step(n).leg; swap = allStep.swap; else; swap = eye(9); end
    [~, step(n).pcm0, ~, ~] = points3DNoYawPcm( swap*step(n).qPlus, 0);
step(n).thetaMin = step(n).pcm0;
else; step(n).thetaMin = allStep.thetaLimits(1); end;

%% Update thetaMax if delta theta is less than deltaThetaMin
step(n).thetaLimits = [step(n).thetaMin; allStep.thetaLimits(2)];
step(n).thetaLimits2 = [step(n).thetaLimits(2); ...
    step(n).thetaLimits(2)+diff(step(n).thetaLimits)];

%% Update halpha
if step(n).impactUpdate
%% Find xact+
if allStep.optimizationOutput
[sPlus,dsPlus,~,~,~,yPlus,dyPlus,~,~,~,~,~,~,~,~,~,~,~] = outputNHVC_mex(...
    step(n).qPlus, step(n).dqPlus, step(n).leg, step(n).thetaLimits, ...
    zeros(6,6), allStep.sigmaScale, step(n).thetaLimits2, zeros(6,4),...
    allStep.cp);  
else
[sPlus,dsPlus,~,~,~,yPlus,dyPlus,~,~,~,~,~,~,~,~,~,~,~] = outputNHVC(...
    step(n).qPlus, step(n).dqPlus, step(n).leg, step(n).thetaLimits, ...
    zeros(6,6), allStep.sigmaScale, step(n).thetaLimits2, zeros(6,4),...
    allStep.cp); end
qactPlus = yPlus; dqactPlus = dyPlus; % Works because hAlpha is zeros.

%% Control update

% Bezier reset.
step(n).hAlpha = hAlphaReset_mex( allStep.hAlpha, sPlus, dsPlus, qactPlus, dqactPlus);

else % Update control for velocity perturbations (BAG160115)

%% Find xact+
    % allStep.pcm0 = step(n).pcm0;
if allStep.optimizationOutput
[sPlus,dsPlus,~,~,~,yPlus,dyPlus,~,~,~,~,~,~,~,~,~,~,~] = outputNHVC_mex(...
    step(n).qPlus, step(n).dqPlus, step(n).leg, step(n).thetaLimits, ...
    zeros(6,6), allStep.sigmaScale, step(n).thetaLimits2, zeros(6,4),...
    allStep.cp);
else;
[sPlus,dsPlus,~,~,~,yPlus,dyPlus,~,~,~,~,~,~,~,~,~,~,~] = outputNHVC(...
    step(n).qPlus, step(n).dqPlus, step(n).leg, step(n).thetaLimits, ...
    zeros(6,6), allStep.sigmaScale, step(n).thetaLimits2, zeros(6,4),...
    allStep.cp); end
qactPlus = yPlus; dqactPlus = dyPlus; % Works because hAlpha is zeros.

%% Control update

% Bezier reset.
step(n).hAlpha = hAlphaReset_mex( allStep.hAlpha, sPlus, dsPlus, qactPlus, dqactPlus);
end

%% Terrain and Force Vector.
if allStep.optimization;
    step(n).terrainVector = step(n).stanceFoot(3);
    step(n).forceVector = [0;0;0]; 
else; step(n).terrainVector = allStep.terrainVector(n); 
    step(n).forceVector = allStep.forceVector(:,n); end

% Debugging


