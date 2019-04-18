function [ step, allStep ] = dataReconstruction3D( step, allStep, n )
% The purpose of this function is to reproduce data from simulations for
% analysis.
% BAG20140923
% BAG20150526 Updated for extended controller.
% BAG20150510 Updated to work for NHVC 3D.
% BAG20150219 Updated to work with AMBVC optimization.
% BAG20150128 Updated to work with angular momentum based outputs.
% BAG20150123 Updated to work for 2D.

% Debugging
AngMomSetTest = [];

% Initialize data.
leg=step(n).leg; hAlpha=step(n).hAlpha; stanceFoot=step(n).stanceFoot;
thetaLimits=step(n).thetaLimits; 
halpha2=step(n).hAlpha2; thetaLimits2=step(n).thetaLimits2;
terrainVector=step(n).terrainVector; forceVector=step(n).forceVector;
mTotal=allStep.mTotal; g=allStep.g; 
swap=allStep.swap; 
sigmaScale = allStep.sigmaScale;
r1=allStep.r1; r2=allStep.r2; r3=allStep.r3;
IndexGearCoordinates=[4 5 6 7 8 9];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%
% Reconstructed Data
%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Evenly sample simulation data.
sampleFreq = 500;
[tSampled, xSampled] = even_sample(step(n).t1, step(n).x1, sampleFreq);
dt=diffBAG(tSampled);
m = size(tSampled,1); midPoint = ceil(m/2); 
% midYaw = xSampled(midPoint,1); % initialYaw = step(n).qMinus(1); %avgYaw = mean(xSampled(:,1)); 
dAngPrev = 0;

% Prepare variables for data collection.
GRFSet=zeros(3,m);swingFootSet=GRFSet;pcmSet=GRFSet;vcmSet=GRFSet;
pcmBodySet=GRFSet; pcmAbsolutSet=GRFSet;
dAngMomSet = zeros(3,m); ddAngMomSet = dAngMomSet;
gravMomSet = dAngMomSet; angMomSet = dAngMomSet;
qSet=zeros(9,m);dqSet=qSet;ddqSet=qSet;
qbarSet=zeros(7,m);dqbarSet=qbarSet;
uSet=zeros(6,m);ySet=uSet;dySet=uSet;powerSet=uSet; hsigmaSet=uSet;
sSet=zeros(1,m);dsSet=sSet;thetaSet=sSet;dthetaSet=sSet;KESet=sSet;
KEPSet=sSet; KEcmXSet=sSet; KEcmYSet=sSet; phipySet=sSet;
sigmaSet=GRFSet; dsigmaSet=GRFSet; ddsigmaSet=GRFSet;
EnergySumTotal=0;
p0Set = GRFSet; p0TSet = GRFSet;

posEnergyTotal = 0;

% Virtual Spring
FvsSet = sSet; qvsSet = sSet;

for i=1:m
    % Get basic data.
    t = tSampled(i); x = xSampled(i,:)';
    
    [dx,u,q,dq,ddq,y,dy,s,ds,theta,dtheta,KE,Jac_cm,dJac_cm,sigma,dsigma,ddsigma,hq,dhqdq,hsigma,pcm,vcm] = SSDynamics3DData( t,x(1:18),thetaLimits,hAlpha,leg,stanceFoot,terrainVector,forceVector,sigmaScale,thetaLimits2,halpha2,allStep );
 
    % Pcm and Vcm
    if leg; q9Link = swap*q; dq9Link = swap*q; else q9Link = q; dq9Link = dq; end
    q9Link(1) = 0; % Zero out yaw for pcm and gait phase calculation.
    pcm = Cfcn_ATRIAS3D_Primary_PointsRight(q9Link);
    [~,~,~,~,dpcmdq9Link,dJ_cm] = Cfcn_Right_Jacobians(q9Link,dq9Link);
    vcm = dpcmdq9Link*[0; dq9Link(2:end)]; % Possible to not consider yaw velocity.


    % Swing foot position
    [swingFoot, phipy, pcmAbsolute, ~, p0, p0T] = points3D( q,leg,stanceFoot );
    
    % GRF
    ddXc = Jac_cm * ddq + dJac_cm * dq;
    grf = mTotal*ddXc + mTotal*[0;0;g];
    
    % Compute Power
    dqgr=dq(IndexGearCoordinates);
    dqm=diag([r1 r2 r3 r1 r2 r3])*dqgr;
    power = dqm.*u;
    energy = power*dt(i);
    % PosPower=max(power,0);
    AbsEnergy=abs(energy);
    % PowerSumPos=PowerSumPos + PosPower;
    EnergySumTotal=EnergySumTotal+AbsEnergy;
    
    % COT
    posEnergy = max(energy,0);
    posEnergyTotal = posEnergyTotal + posEnergy;
    
    % KE Pendulum
    % vcm = Jac_cm*dq;
    KEPendulum = 1/2*mTotal*norm(vcm)^2;
    
    % Store information in sets
    GRFSet(:,i)=grf; qSet(:,i)=q; dqSet(:,i)=dq; ddqSet(:,i)=ddq; pcmSet(:,i)= pcm;
    vcmSet(:,i)=vcm; KEPSet(:,i)=KEPendulum; KESet(:,i)=KE;
    uSet(:,i)=u; sSet(:,i)=s; dsSet(:,i)=ds; ySet(:,i)=y;
    dySet(:,i)=dy;
    swingFootSet(:,i)=swingFoot; pcmAbsolutSet(:,i)=pcmAbsolute;
    thetaSet(:,i)=theta; dthetaSet(:,i)=dtheta;
    qbarSet(:,i)=[theta; hq]; dqbarSet(:,i)=[dtheta; dhqdq*dq];
    
    % KE Pendulum Components
    KEcomX = 1/2*mTotal*vcm(1)^2*sign(vcm(1)); 
    KEcmY = 1/2*mTotal*vcm(2)^2;
    KEcmXSet(:,i)=KEcomX; KEcmYSet(:,i)=KEcmY;
    
    % Momentum data.
    gravityMoment = mTotal*cross(pcm,[0;0;-g]); % (Nm)
    angularMomentum = mTotal*cross(pcm,vcm);
    gravMomSet(:,i) = gravityMoment;
    angMomSet(:,i) = angularMomentum;
    dAngMomSet(:,i) = mTotal*cross(pcm,[0;0;g]);
    ddAngMomSet(:,i) = mTotal*cross(vcm,[0;0;g]);
    sigmaSet(:,i)=sigma; dsigmaSet(:,i)=dsigma; ddsigmaSet(:,i)=ddsigma;
    phipySet(:,i)=phipy; powerSet(:,i)=power;
    hsigmaSet(:,i)=hsigma;
    
    % Torso position
    p0Set(:,i) = p0; p0TSet(:,i) = p0T;
    
end


% Store information in step structure.
step(n).GRF = GRFSet; step(n).u = uSet; step(n).energy = EnergySumTotal;
step(n).posEnergy = posEnergyTotal;
step(n).q = qSet; step(n).dq = dqSet; step(n).ddq = ddqSet;
step(n).s = sSet; step(n).ds = dsSet;
step(n).x = xSampled'; step(n).swingFoot = swingFootSet;
step(n).theta = thetaSet; step(n).dtheta = dthetaSet;
step(n).t = tSampled; step(n).pcm = pcmSet; step(n).KE = KESet;
step(n).KEPendulum = KEPSet; step(n).PE = mTotal*g*pcmSet(2,:);
step(n).KEcmX = KEcmXSet; step(n).KEcmY = KEcmYSet; 
step(n).sigma = sigmaSet; step(n).dsigma = dsigmaSet;
step(n).ddsigma = ddsigmaSet;
step(n).pcmAbsolute = pcmAbsolutSet;
step(n).vcm = vcmSet; step(n).gravityMoment = gravMomSet;
step(n).dAngularMomentum = dAngMomSet; step(n).angularMomentum = angMomSet;
step(n).y = ySet; step(n).dy = dySet;
step(n).qbar = qbarSet; step(n).dqbar = dqbarSet;
step(n).phipy = phipySet; step(n).power = powerSet;
step(n).hsigma = hsigmaSet;

% Create a standard set of state and torque values for RIO costs.
if step(n).leg
    step(n).xRIO = step(n).x(1:18,:); step(n).uRIO = step(n).u;
    step(n).dthetax = dqSet'*[0; 0; -1; 0; 0; 0; -0.5; -0.5; 0];
    step(n).dthetay = dqSet'*[0; 1; 0; 0; 0; 0; 0; 0; -1];
else
    step(n).xRIO = [swap zeros(9,9); zeros(9,9) swap]*step(n).x(1:18,:);
    step(n).uRIO = allStep.uSwap*step(n).u;
    step(n).dthetax = dqSet'*[0; 0; -1; -0.5; -0.5; 0; 0; 0; 0];
    step(n).dthetay = dqSet'*[0; -1; 0; 0; 0; -1; 0; 0; 0];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extra data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Distance and velocity information.
endFoot = step(n).swingFoot(:,end);
step(n).stepDistance = norm(endFoot-step(n).stanceFoot);
step(n).twoStepDistance = norm(endFoot-step(n).swingFoot(:,1));
step(n).comDistance = pcmSet(:,end)-pcmSet(:,1);
step(n).xyzDistance = endFoot-step(n).stanceFoot;
step(n).avgCoMVelocity = ...
    step(n).comDistance/step(n).stepTime;
step(n).avgStepVelocity = ...
    step(n).stepDistance/step(n).stepTime;
step(n).avgyVelocity = step(n).xyzDistance(2)/step(n).stepTime;

% Step height and direction information.
step(n).stepHeight = endFoot(3)-step(n).stanceFoot(3);
step(n).prevStanceFoot = step(n).swingFoot(:,1);
step(n).prevStepHeight = step(n).stanceFoot(3) - step(n).swingFoot(3,1);
step(n).initialHeading = step(n).q(1,1)*180/pi;
step(n).finalHeading = step(n).q(1,end)*180/pi;
step(n).deltaHeading = step(n).finalHeading - step(n).initialHeading;
if (n==1)
    step(n).prevLeg = 0;
    step(n).twoStepDeltaHeading = step(n).deltaHeading;
else
    if (allStep.optimization)
        step(n).twoStepDeltaHeading = ...
        step(n).finalHeading - step(1).initialHeading;
    else
        step(n).prevLeg = step(n-1).leg;
        step(n).twoStepDeltaHeading = ...
            step(n).deltaHeading + step(n-1).deltaHeading;
    end
end
if step(n).twoStepDeltaHeading>10^-3; % Previously got error resulting from essentially zero yaw.
step(n).turnDiameter = step(n).twoStepDistance * ...
    sum(sin(linspace(0,180,ceil(180/abs(step(n).twoStepDeltaHeading)))...
    * pi/180));
else; step(n).turnDiameter = inf; end

% Calculate impact losses of previous step.
[ KEMinus, PEMinus, KEPendulumMinus, PEStanceMinus ] = energy3D( ...
    allStep, step(n).qMinus, step(n).dqMinus, step(n).prevLeg, ...
    step(n).prevStanceFoot );

step(n).impactLoss = KEMinus - step(n).KE(1);
step(n).impactLossPendulum = KEPendulumMinus - step(n).KEPendulum(1);
step(n).KEMinus = KEMinus;

% Calculate final swing foot velocities:
q13 = step(n).q(:,end); dq13 = step(n).dq(:,end);
q13 = [0; q13([2:3 4 5 7 8 4:9])]; dq13 = dq13([1:3 4 5 7 8 4:9]);
if step(n).leg
    [~, ~, J4R] = ATRIAS3D_FootPointPosJacob_Left(q13);
    step(n).swingFootVelFinal = J4R*dq13;
else
    [~, ~, ~, ~, ~, ~, J4L] = ATRIAS3D_FootPointPosJacob_Right(q13);
    step(n).swingFootVelFinal = J4L*dq13;
end

% Calculate vcm at vertical
pcm0Idx = find(step(n).pcm(2,:)>0,1,'first');
step(n).vcmVertical = step(n).vcm(:,pcm0Idx);

% Calculate relative swing foot position with no yaw:
[step(n).xyzDistanceNoYaw] = points3D( [0; step(n).q(2:end,end)],leg);

% Calculate CoT E/(mgd)
% CoMy distance
step(n).COT = sum(step(n).energy)/(mTotal*g*step(n).xyzDistanceNoYaw(2));
step(n).MCOT = sum(step(n).posEnergy)/(mTotal*g*step(n).xyzDistanceNoYaw(2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Debugging and plotting:

if allStep.printData

end

