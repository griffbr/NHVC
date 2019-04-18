function [step, allStep] = tauIntegralError(step, allStep);
% Script for testing different ways of sampling data together for
% calculating error cost as an integral ds.
% August 2nd, 2013. BAG
% BAG20151231 Updated to actually use tau for integral instead of theta.
% Also using weighting parameters for specific states and torques for the
% first time.
% BAG20150930 Updated to only calculate error on specified steps.
% BAG20141030 Updated to use theta instead of s.
% BAG20141021 Fixed error where xNomInterpPoint is NaN.
% BAG20141003 Updated to work with robust impact optimization 3D.

% Initialize parameters.
numSteps = allStep.numSteps;
taupPlusLimit = 0; % Based on theta being horizontal CoM position.
taupPlusLimit = 0.05; % BAG20151112
taupEndLimit = step(1).s(end);
sampleAccuracy = 1000;
epsilon = 1/10000;

%% Error weighting matrices
Q = 1500*eye(18); R = 300*eye(6);
% State specific scales:
torsoPitchAndRoll = 4; hips = 2; velocity = 0.25;
Q([2:3 11:12],:) = Q([2:3 11:12],:)*torsoPitchAndRoll;
Q([6 9 15 18],:) = Q([6 9 15 18],:)*hips;
Q([10:18],:) = Q([10:18],:)*velocity;

%--------------------------------------------------------------------------
% S integral
%--------------------------------------------------------------------------

% Calculate some nominal trajectory values used for comparison.
xNomPlus = step(1).xRIO(:,1); uNomPlus = step(1).uRIO(:,1);

%--------------------------------------------------------------------------
%% Main Loop
%--------------------------------------------------------------------------
% Main loop that runs through each step and calculates integrated errors ds.
for i = 1:length(allStep.RIOIdx); % First step is used to calculate xperturbed minus only.
    n = allStep.RIOIdx(i);
    % Initiate some data gathering parameters.
    taupPlus = step(n).s(1); taupEnd = step(n).s(end); 
    taupRange = taupEnd-taupPlus;
    accuracyOverlap = sampleAccuracy * taupRange;
    
    % Make sure spEnd is not past limit
    if (taupEnd > taupEndLimit)
        % fprintf('error, spEnd greater than spEndLimit\n')
        errorCost = 10^6*(0.05 + taupEnd - taupEndLimit);
        step(n).xErrorIntegral = errorCost;
        step(n).uErrorIntegral = errorCost;
        continue
    end
    
    % Make sure there are sufficient data points.
    if (size(step(n).s,2)<2)
        % fprintf('error, spEnd greater than spEndLimit\n')
        errorCost = 10^9;
        step(n).xErrorIntegral = errorCost;
        step(n).uErrorIntegral = errorCost;
        continue
    end
    
    %----------------------------------------------------------------------
    % If sp+ starts below s+ nominal.
    %----------------------------------------------------------------------
    if (taupPlus < step(1).s(1)) 
        
        % Find first data point at which perturbed step has s greater than
        % s+ nominal. Also find last point of overlapping data.
        j = find(step(n).s>step(1).s(1),1,'first');
        k = find(step(1).s<taupEnd,1,'last');
        
        % Interpolate values for sp = snom+ and spEnd.
        xInterpPoint = interp1(step(n).s',step(n).xRIO',step(1).s(1));
        uInterpPoint = interp1(step(n).s',step(n).uRIO',step(1).s(1));
        xNomInterpPoint = interp1(step(1).s',step(1).xRIO',taupEnd);
        uNomInterpPoint = interp1(step(1).s',step(1).uRIO',taupEnd);
        
        % Build up data set to be used for calculating error costs up until
        % there is overlap between two sets of s data.
        x1 = [step(n).xRIO(:,1:(j-1)), xInterpPoint'];
        u1 = [step(n).uRIO(:,1:(j-1)), uInterpPoint'];
        theta1 = [step(n).s(1:(j-1)), step(1).s(1)];
        
        % Build up data set to be used for calculating error costs once
        % there is overlap between two sets of s data.
        x2 = [xInterpPoint', step(n).xRIO(:,j:end)];
        u2 = [uInterpPoint', step(n).uRIO(:,j:end)];
        theta2 = [step(1).s(1), step(n).s(j:end)];
        xNomOverlap = [step(1).xRIO(:,1:k), xNomInterpPoint'];
        uNomOverlap = [step(1).uRIO(:,1:k), uNomInterpPoint'];
        thetaNomOverlap = [step(1).s(1:k), taupEnd];
        
        % Sample the second set of data so that it corresponds to the same
        % points as the nominal set over the nominal range of s.
        [~, x2Sampled] = even_sampleBAG(theta2, x2, accuracyOverlap);
        [~, u2Sampled] = even_sampleBAG(theta2, u2, accuracyOverlap);
        [~, xNomOverlapSampled] = even_sampleBAG(thetaNomOverlap,...
            xNomOverlap, accuracyOverlap);
        [theta2Sampled, uNomOverlapSampled] = even_sampleBAG...
            (thetaNomOverlap, uNomOverlap, accuracyOverlap);
               
        % Calculate error.
        m1 = size(x1,2);
        s = [theta1 theta2Sampled]; x = [x1 x2Sampled]; 
        u = [u1 u2Sampled];
        dtau = diffBAG(s');
        intScalar = ((s - taupPlus)/(taupRange));
        
        % Have been getting an error. This should prevent.
        if ~(size(x,2)==size([repmat(xNomPlus,1,m1) xNomOverlapSampled],2))
        errorCost = 10^9;
        step(n).xErrorIntegral = errorCost;
        step(n).uErrorIntegral = errorCost;
        continue
        end
        
        % Change made 160217
        xError = sum((Q*(x-[repmat(xNomPlus,1,m1) xNomOverlapSampled])).^2)*...
            diag(dtau)*diag(intScalar);
        uError = sum((R*(u-[repmat(uNomPlus,1,m1) uNomOverlapSampled])).^2)*...
            diag(dtau)*diag(intScalar);
        
    end
    %----------------------------------------------------------------------
        
    %----------------------------------------------------------------------
    % If sp+ starts above s+ nominal.
    %----------------------------------------------------------------------
    if (taupPlus >= step(1).s(1))  
        
        % Make sure spPlus is not past limit
        if (taupPlus > taupPlusLimit)
            % fprintf('error, spPlus greater than spPlusLimit\n')
            errorCost = 10^6*(0.05 + taupPlus - taupPlusLimit);
            step(n).xErrorIntegral = errorCost;
            step(n).uErrorIntegral = errorCost;
            continue
        end
        
        % Find first data point at which nominal step has s greater than
        % sp+ of current step as well as last s less than spEnd.
        j = find(taupPlus < step(1).s,1,'first');
        k = find(taupEnd > step(1).s,1,'last');
                
        % Interpolate values for snominal = sp+ and spEnd
        xNomInterpPoint1 = interp1(step(1).s',step(1).xRIO',taupPlus);
        uNomInterpPoint1 = interp1(step(1).s',step(1).uRIO',taupPlus);
        xNomInterpPoint2 = interp1(step(1).s',step(1).xRIO',taupEnd);
        uNomInterpPoint2 = interp1(step(1).s',step(1).uRIO',taupEnd);
  
        % Build up data set to be used for calculating error costs from sp+
        % to spEnd, where there is overlap in data.
        xNomOverlap = [xNomInterpPoint1', step(1).xRIO(:,j:k), ...
            xNomInterpPoint2'];
        uNomOverlap = [uNomInterpPoint1', step(1).uRIO(:,j:k), ...
            uNomInterpPoint2']; 
        thetaNomOverlap = [taupPlus, step(1).s(j:k), taupEnd];
        
        % Sample both sets of data so that they corresponds to the same
        % points as the over the overlapping range s.
        [~, x] = even_sampleBAG(step(n).s,...
            step(n).xRIO, accuracyOverlap);
        [~, u] = even_sampleBAG(step(n).s,...
            step(n).uRIO, accuracyOverlap);
        [~, xNomOverlapSampled] = even_sampleBAG(thetaNomOverlap,...
            xNomOverlap, accuracyOverlap);
        [s, uNomOverlapSampled] = even_sampleBAG(thetaNomOverlap,...
            uNomOverlap, accuracyOverlap);
        
        % Calculate error.
        dtau = diffBAG(s);
        intScalar = ((s - taupPlus)/(taupRange));
        xWeightedSquareError = (Q*(x-xNomOverlapSampled)).^2;
        uWeightedSquareError = (R*(u-uNomOverlapSampled)).^2;
        xError = sum(xWeightedSquareError)*...
            diag(dtau)*diag(intScalar);
        uError = sum(uWeightedSquareError)*...
            diag(dtau)*diag(intScalar);
    end
    %----------------------------------------------------------------------
           
    % Save information into structure data for later cost evaluation.
    step(n).xError = xError; step(n).xErrorIntegral = sum(xError);
    step(n).uError = uError; step(n).uErrorIntegral = sum(uError);
    step(n).tauProcessed = s; 
    step(n).thetapRange = taupRange; step(n).thetapPlus = taupPlus;  
    step(n).xWeightedSquareError = xWeightedSquareError;
    step(n).uWeightedSquareError = uWeightedSquareError;
    
    % Debugging
    % figure; plot(xError); figure; plot(uError);
    
end
%--------------------------------------------------------------------------







