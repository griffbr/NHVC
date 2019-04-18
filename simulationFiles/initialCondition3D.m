function [step, allStep] = initialCondition3D(allStep)
% This function generates the initial conditions for simulation from the
% more minimalist set of free parameters that are sent into the solver.
% BAG20140806
% BAG20151110 Updated to work with NHVSP qLAKASW outputs.
% BAG20151022 Updated to work with constant theta limits.
% BAG20150928 Updated to work with 3D gait phase (changes in CoM position).
% BAG20150902 Updated to work with Virtual Springs.
% BAG20150719 Updated to work with ROLO.
% BAG20150522 Updated to work with extended Bezier.
% BAG20150509 Switching to format more consistent with experiments.
% BAG20150402. Updated to work with NHVC3D.
% BAG20150216. Updated to work with new AMBVC outputs.
% BAG20150201. Added extended hAlpha map to help with s > 1 control.
% BAG20150127. Updated to work for new angular momentum based outputs.
% BAG20140122. Updated to work in 2D case.

%% Initiliaze variables
x0 = allStep.x0;
qMinus = x0(1:9); dqMinus = x0(10:18);
bezier2 = x0(19:24); bezier3 = x0(25:30);
bezier2Ext = x0(31:36); bezier3Ext = x0(37:42);
allStep.sigmaScale = x0(43:48);
swap = allStep.swap; legPlus = 1; legMinus = 0;
allStep.gaitPhaseScale = [0; 1; 0]; % kx ky kz for 3D gait phase.
allStep.cp = x0(49:end);

%% Find x+.
[qPlus, dqPlus, step(1).impF, ~] = impact3D(qMinus, dqMinus, legMinus, allStep.yawRestricted);

%% Find thetaLimits. (now based on pcmx)
qMinusPcm = [0; qMinus(2:9)]; % Zero out yaw for initial pcm calculation.
[swingFootMinus, phipyMax] = points3DNoYawPcm( qMinusPcm, legMinus);
qPlusPcm = [0; qPlus(2:9)];
[~, phipyMin] = points3DNoYawPcm( swap*qPlusPcm, legMinus);
phipyRange = phipyMax - phipyMin;
phipyLimits = [phipyMin; phipyMax];

%% Find xact+ and xact-
allStep.pcm0 = phipyMin;
if allStep.optimizationOutput;
    [~,dsMinus,~,~,~,yMinus,dyMinus] = outputNHVC_mex(...
        qMinus,dqMinus,legMinus,phipyLimits,zeros(6,6),allStep.sigmaScale,...
        zeros(2,1),zeros(6,4),allStep.cp );
    [~,dsPlus,~,~,~,yPlus,dyPlus] = outputNHVC_mex(...
        qPlus,dqPlus,legPlus,phipyLimits,zeros(6,6),allStep.sigmaScale,...
        zeros(2,1),zeros(6,4),allStep.cp );
else
    [~,dsMinus,~,~,~,yMinus,dyMinus] = outputNHVC(...
        qMinus,dqMinus,legMinus,phipyLimits,zeros(6,6),allStep.sigmaScale,...
        zeros(2,1),zeros(6,4),allStep.cp );
    [~,dsPlus,~,~,~,yPlus,dyPlus] = outputNHVC(...
        qPlus,dqPlus,legPlus,phipyLimits,zeros(6,6),allStep.sigmaScale,...
        zeros(2,1),zeros(6,4),allStep.cp );   
end
qactPlus = yPlus; dqactPlus = dyPlus; % Works because hAlpha is zeros.
qactMinus = yMinus; dqactMinus = dyMinus; % Works because hAlpha is zeros.

%% Find hAlpha finding remaining bezier parameters from xMinus and xPlus.

% First two Bezier columns from xPlus.
b0=qactPlus; b1=b0+dqactPlus/(5*dsPlus);
% Last two Bezier columns from xMinus.
b5=qactMinus; b4=b5-dqactMinus/(5*dsMinus);
halpha = [b0 b1 bezier2 bezier3 b4 b5];

% Find hAlpha extended. Derivation from BAG20150131 Notes.
% Assume extended hAlpha goes from s = 1 to s = 2.
allStep.thetaRange2 = phipyRange; % Keep sdot continuous. 
allStep.thetaLimits2 = ...
    [phipyLimits(2); phipyLimits(2) + allStep.thetaRange2];
% Assume a 6th order polynomial for halpha and 4th order polynomial for
% halpha2.
b0Ext = b5; b1Ext = b0Ext + (5/3)*(b5 - b4)*allStep.thetaRange2/phipyRange;
halpha2 = [b0Ext b1Ext bezier2Ext bezier3Ext];

%% Check periodic foot placement.
allStep.initialSwingFootHeight = swingFootMinus(3);

%% Assign variables to step and allStep.
step(1).pcm0 = phipyMin;
step(1).qMinus=qMinus; step(1).dqMinus=dqMinus; 
step(1).x0=[qPlus; dqPlus]; step(1).stanceFoot=[0;0;0]; 
step(1).startTime = allStep.time;
step(1).impactUpdate = 0;
allStep.hAlpha=halpha; allStep.hAlpha2=halpha2;
allStep.thetaLimits=phipyLimits; allStep.thetaRange=phipyRange;

%% debugging

if 0
    %%
    swap = [-1 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
            0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1;
            0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0];

fprintf('HAlpha = [%4.10g %4.10g %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g %4.10g %4.10g];\n',halpha(1,:),halpha(2,:),halpha(3,:),halpha(4,:),halpha(5,:),halpha(6,:))
fprintf('HAlpha2 = [%4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g;\n %4.10g %4.10g %4.10g %4.10g];\n',halpha2(1,:),halpha2(2,:),halpha2(3,:),halpha2(4,:),halpha2(5,:),halpha2(6,:))
fprintf('PHipLimits = [%4.10g; %4.10g];\n',phipyLimits);
fprintf('kSigma = [%4.10g; %4.10g; %4.10g; %4.10g; %4.10g; %4.10g; %4.10g; %4.10g; %4.10g];\n',[allStep.sigmaScale; zeros(3,1)]);
fprintf('\n')
fprintf('phaseScale = [%g; %g; %g];\n',allStep.gaitPhaseScale);
fprintf('cp = [%g];\n',allStep.cp);
    
end

end

