function [ inequalityVector, step ] = inequalityConstraints3D( step, printData, n, periodicStepNum )
% The purpose of this function is to calculate the inequality
% constraints after data processing for an individual step(n).
% BAG20140923
% BAG20150930 Updated to include a maximum instantaneous power for each
% link.
% BAG20150903 Updated to include impact coefficient of friction.
% BAG20150511 Updated to work with NHVC 3D.
% BAG20150219 Updated to work with AMBVC2D.
% BAG20150106. Adding forward torso angle as an inequality constraint.
% BAG20140925. Adding velocity as a constraint.
% BAG20140924. Updated to include option of random weights for inequality
% constraints. The idea behind this is to help with avoiding local minimum.

% Improvements:
% Add an inequality constraint for roll torso movements.

% Initialize data.
GRF=step(n).GRF; u=step(n).u; numData=size(step(n).u,2);

% Calculation for inequality constraints
if step(n).leg
    step(n).swingKneeAngle = step(n).q(5,:)-step(n).q(4,:);
    step(n).stanceKneeAngle = step(n).q(8,:)-step(n).q(7,:);
    maxStanceKneeVel = max(step(n).dq(8,:)-step(n).dq(7,:))*180/pi;
    step(n).endSwingHip = step(n).q(6,end);
    step(n).endStanceHip = step(n).q(9,end);
else
    step(n).stanceKneeAngle = step(n).q(5,:)-step(n).q(4,:);
    maxStanceKneeVel = max(step(n).dq(5,:)-step(n).dq(4,:))*180/pi;
    step(n).swingKneeAngle = step(n).q(8,:)-step(n).q(7,:); 
    step(n).endSwingHip = step(n).q(9,end);
    step(n).endStanceHip = step(n).q(6,end);
end;
coefFrictionReq = max(abs(((GRF(1,:).^2+GRF(2,:).^2).^0.5./GRF(3,:))));
impactCoefFriction = norm(step(n).impF(1:2))/step(n).impF(3);    
zGRFMin = min(GRF(3,:));
stanceKAMin = min(step(n).stanceKneeAngle);
swingKAMin = min(step(n).swingKneeAngle);
% Swing foot clearance, ground and with stance.
swingFootRel = step(n).swingFoot - repmat(step(n).stanceFoot,1,numData);
[swingFootLat,clearancePoint] = ...
    min(swingFootRel(1,:).^2 + swingFootRel(2,:).^2);
swingFootLat = swingFootLat^0.5;
swingFootHeight = swingFootRel(3, clearancePoint);
swingFootEndLat = (-1)^(1-step(n).leg)*swingFootRel(1,end);
dqActuatedMax = max(max(abs(step(n).dq([4 5 6 7 8 9],:))'*180/pi));
dqActuatedHipMax = max(max(abs(step(n).dq([6 9],:))'*180/pi));
ddqActuatedMax = max(max(abs(step(n).ddq([4 5 6 7 8 9],:))'*180/pi));
q3RL = max((step(n).q(6,:)+step(n).q(9,:))*180/pi);
deltaStep = swingFootRel(3,end) - swingFootRel(3,1);
hipRange = max(range(step(n).q([6 9],:)'))*180/pi;

% Constraint parameters.
zGRFLowerBound = 250; zGRFMinWeight = 20; % zGRFLowerBound = 250; zGRFMinWeight = 50;
coefFriction = 0.5; coefFrictionWeight = 2000; % coefFriction was 0.6.
torqueMax = 6; torqueMaxWeight = 1000;
dsPositiveWeight = 100000;
% Velocity/ Acceleration/ Speed
stepVelocityMin = 0.5; stepVelocityMax = 1; stepVelocityWeight = 30000; % step(n)VelocityWeight = 20000;
dqMax = 200; dqMaxWeight = 40;
dqSTKAMax = 75; dqSTKAMaxWeight = 100;
ddqMax = 3000; ddqMaxWeight = 2;
stepTimeMax = 0.4; stepTimeWeight = 10000;
dqHipMax = 60; dqHipMaxWeight = 100;
dqHipMaxNom = 25; dqHipMaxNomWeight = 200;
powerWeight = 10; powerMax = 1000; % powerMax = 240; % 60V x 5A x 80% eff. (better than nothing?)
% Posture
swingFootEndLatMin = 0.05; swingFootLatEndWeight = 10000;
swingLegRetVelMin = 0; swingLegRetractionWeight = 5000; % swingLegRetractionWeight = 4000;
swingFootZVelMin = -1; swingFootZVelWeight = 4000;
STKAMax = 30*pi/180; STKAMaxWeight = 15000;
kneeAngleMin = 20*pi/180; kneeAngleWeight = 15000; % To match exp safety limits (was 10 degrees)
swingFootLatMin = 0.1; swingFootLatWeight = 1000;
swingFootClearanceMin = 0.1; swingFootClearanceWeight = 20000;
nomyDistanceMin = 0.3; nomyDistanceWeight = 15000;
xDistanceMax = 1*step(n).xyzDistance(2); xDistanceWeight = 10000;
if (abs(deltaStep)>0.02); swingLegRetractionWeight = 0;
nomyDistanceWeight = 0; STKAMaxWeight = 0; dqSTKAMaxWeight = 0; end
% Torso
torsoAngleMax = 5*pi/180; torsoMaxWeight = 10000; % torsoMaxWeight = 3000;
torsoRollAngleMax = 3*pi/180; torsoRollWeight = 3000;
% Hips
q3RLMax = 10; q3RLMaxWeight = 1000;
q3SWEndMax = 6; q3SWEndMaxWeight = 1000;
q3SWEndMaxNom = 4; q3SWEndMaxWeight = 1000;
q3SWEndMinPerturbed = -6; q3SWEndMinWeight = 1000;
q3SWEndMinNom = 0; q3SWEndMinWeight = 1000;
q3STEndMinNom = 0; q3STEndMinWeight = 1000;
q3RangeMaxNom = 5; q3RangeMaxNomWeight = 1000;

%--------------------------------------------------------------------------
% Inequality constraints.
%--------------------------------------------------------------------------

if printData
   fprintf('Coefficient of friction required is %g\n', coefFrictionReq);
   fprintf('Minimum Vertical Ground Reaction Force is %g N\n', zGRFMin);
end

step(n).ineq.GRFMin = zGRFMinWeight*(zGRFLowerBound - zGRFMin);
step(n).ineq.coefFriction = ...
    coefFrictionWeight*(coefFrictionReq - coefFriction);
step(n).ineq.impactCoefFriction = ...
    coefFrictionWeight*(impactCoefFriction - coefFriction);
step(n).ineq.stanceKneeAngle = kneeAngleWeight*(kneeAngleMin - stanceKAMin);
step(n).ineq.swingKneeAngle = kneeAngleWeight*(kneeAngleMin - swingKAMin);
step(n).ineq.maxLinkTorque = ...
    torqueMaxWeight*( max(max(abs(u))) - torqueMax);
step(n).ineq.swingFootLat = swingFootLatWeight * ...
    (swingFootLatMin - swingFootLat);
step(n).ineq.swingFootClearance = swingFootClearanceWeight * ...
    (swingFootClearanceMin - swingFootHeight);
step(n).ineq.dsPositive = dsPositiveWeight*(-min(step(n).ds));
step(n).ineq.avgVelocityMin = stepVelocityWeight*...
    (stepVelocityMin - step(n).avgCoMVelocity(2));
step(n).ineq.avgVelocityMax = stepVelocityWeight*...
    (step(n).avgCoMVelocity(2) - stepVelocityMax);
step(n).ineq.torsoMax = torsoMaxWeight * (max(step(n).q(3,:)) - torsoAngleMax);
step(n).ineq.torsoRollMax = torsoRollWeight * (max(abs(step(n).q(2,:))) - torsoRollAngleMax);
step(n).ineq.powerMax = powerWeight * (max(max(abs(step(n).power'))) - powerMax);
step(n).ineq.swingFootLatEnd = swingFootLatEndWeight * ( swingFootEndLatMin - swingFootEndLat);
step(n).ineq.swingLegRetraction = swingLegRetractionWeight * ( step(n).swingFootVelFinal(2) - swingLegRetVelMin );
step(n).ineq.swingFootZVel = swingFootZVelWeight * (swingFootZVelMin - step(n).swingFootVelFinal(3));
step(n).ineq.dqMax = dqMaxWeight * ( dqActuatedMax - dqMax );
step(n).ineq.q3RLMax = q3RLMaxWeight * ( q3RL - q3RLMax );
step(n).ineq.q3SWEndMax = q3SWEndMaxWeight * ( step(n).endSwingHip*180/pi - q3SWEndMax );
step(n).ineq.STKAMax = STKAMaxWeight * ( max(step(n).stanceKneeAngle) - STKAMax );
step(n).ineq.yDistanceMin = nomyDistanceWeight * ( nomyDistanceMin - step(n).xyzDistance(2) );
step(n).ineq.ddqMax = ddqMaxWeight * ( ddqActuatedMax - ddqMax );
step(n).ineq.timeMax = stepTimeWeight * ( step(n).stepTime - stepTimeMax );
step(n).ineq.xyDistanceRatio = xDistanceWeight * ( abs(step(n).xyzDistance(1)) - xDistanceMax );
step(n).ineq.q3SWEndMin = q3SWEndMinWeight * ( q3SWEndMinPerturbed - step(n).endSwingHip*180/pi );
step(n).ineq.q3SWEndMinNom = q3SWEndMinWeight * ( q3SWEndMinNom - step(n).endSwingHip*180/pi );
step(n).ineq.q3STEndMinNom = q3STEndMinWeight * ( q3STEndMinNom - step(n).endStanceHip*180/pi );
step(n).ineq.dqHipMax = dqHipMaxWeight * ( dqActuatedHipMax - dqHipMax );
step(n).ineq.dqHipMaxNom = dqHipMaxNomWeight * ( dqActuatedHipMax - dqHipMaxNom );
step(n).ineq.hipRangeMaxNom = q3RangeMaxNomWeight * ( hipRange - q3RangeMaxNom );
step(n).ineq.q3SWEndMaxNom = q3SWEndMaxWeight * ( step(n).endSwingHip*180/pi - q3SWEndMaxNom );
step(n).ineq.dqSTKAMax = dqSTKAMaxWeight * ( maxStanceKneeVel - dqSTKAMax );

inequalityVector = [step(n).ineq.GRFMin; 
    step(n).ineq.coefFriction;... % 2
    step(n).ineq.impactCoefFriction; ...
    step(n).ineq.stanceKneeAngle;... % 4
    step(n).ineq.swingKneeAngle; ...
    step(n).ineq.maxLinkTorque; ...
    step(n).ineq.swingFootClearance;... % 7
    step(n).ineq.dsPositive; 
    step(n).ineq.avgVelocityMin; ... % 9
    step(n).ineq.avgVelocityMax; ... % 10
    step(n).ineq.torsoMax;... % 11
    step(n).ineq.torsoRollMax; 
    step(n).ineq.powerMax; % 13
    step(n).ineq.swingFootLatEnd;
    step(n).ineq.swingLegRetraction; ... % 15
    step(n).ineq.swingFootZVel; 
    step(n).ineq.dqMax; % 17
    step(n).ineq.q3RLMax; 
    step(n).ineq.q3SWEndMax; % 19
    step(n).ineq.STKAMax; % 20
    step(n).ineq.yDistanceMin;
    step(n).ineq.ddqMax; % 22
    step(n).ineq.timeMax;
    step(n).ineq.xyDistanceRatio; % 24
    step(n).ineq.q3SWEndMin;
    step(n).ineq.q3SWEndMinNom; % 26
    step(n).ineq.q3STEndMinNom;
    step(n).ineq.dqHipMax; % 28
    step(n).ineq.dqHipMaxNom;
    step(n).ineq.hipRangeMaxNom; % 30
    step(n).ineq.q3SWEndMaxNom;
    step(n).ineq.dqSTKAMax];

% Some inequalities not enforced on non-periodic steps
if ~(n == periodicStepNum || n == periodicStepNum - 1);
    inequalityVector([9 10 11 15 20 21 23 24 26 27 29 30 31]) = -100; end

step(n).inequalityVector = inequalityVector;

% For reference
% q = [qzT; qyT; qxT; q1R; q2R; q3R; q1L; q2L; q3L];

end

