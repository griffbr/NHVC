function [finalParameters, finalValue] = fminconiterate(options, initialParameters, optimizationFunction, constraintFunction, deltaLower, deltaUpper, nIterations, increaseRatio)
% This function takes in options, initialParameters, optimizationFunction,
% constraintFunction, deltaLower, deltaUpper and nIterations and
% iteratively uses fmincon on function using output of fmincon as the next
% set of input parameters. Arguments returned include the finalParameters 
% and finalValue.
% BAG20150223 Modified to have noise ratio be on a per parameter basis and
% not an overall average.
% BAG20140924 Modified to accept increase ratio externally.
% BAG20140816 Modified for 3D optimization.
% Could be modified to use structure data to print out all iterated
% parameter values at the end of the sequence.
% BAG August 21st, 2013. Added deltaLower and deltaUpper.
% BAG 1/3/13

% Initialization and initial cost display.
fprintf('\nBegining fminconiterate with %d iterations on function %s.\n',nIterations,optimizationFunction);
currentParameters=initialParameters;
functionHandle = str2func(optimizationFunction);
[initialCost,initialEq,initialIneq,~,allStep] = functionHandle(initialParameters);
initialCosts = [allStep.RIOCost; allStep.equalityCost; allStep.inequalityCost; allStep.poincareCost; allStep.energyCost];
initialEigValue = allStep.maxEigValue;
fprintf('%% Initial cost is %g;\n', initialCost);
fprintf('%% Costs: RIO %g, equality %g, inequality %g, Poincare %g, Energy %g\n',initialCosts);
fprintf('%% Maximum Eigenvalue is %g\n',initialEigValue);

%--------------------------------------------------------------------------
% Main loop
%--------------------------------------------------------------------------

for i=1:nIterations;
    
    % Add noise
    %----------------------------------------------------------------------
    % This has been added as an experiment to add noise to parameters
    % between function calls in an attempt to avoid local minimum.
    if 1
    noiseRatio = mean([-deltaLower deltaUpper]')'/25;
    noise = normrnd(0,noiseRatio,size(currentParameters));
    currentParameters = ...
        currentParameters + noise;
    else
        noise = zeros(size(currentParameters));
    end
    %----------------------------------------------------------------------
    
    % Call fmincon
    %----------------------------------------------------------------------
    lowerbound=[currentParameters + deltaLower + abs(noise)];
    upperbound=[currentParameters + deltaUpper - abs(noise)];
    [finalParameters,finalValue]=fmincon(optimizationFunction,currentParameters,[],[],[],[],lowerbound,upperbound,[],options);
    %----------------------------------------------------------------------
    
    % Prepare for next iteration.
    currentParameters = finalParameters;
    % Save parameters for a history of iterations to be printed out.
    parameterHistory(i).parameters = finalParameters;
    parameterHistory(i).value = finalValue;
    [fTemp,eqTemp,ineqTemp, ~, allStep] = functionHandle(finalParameters);
    parameterHistory(i).ineq = ineqTemp;
    parameterHistory(i).eq = eqTemp;
    parameterHistory(i).costs = [allStep.RIOCost; allStep.equalityCost; allStep.inequalityCost; allStep.poincareCost; allStep.energyCost];
    parameterHistory(i).maxEigValue = allStep.maxEigValue;
    
    % Print out information
    %----------------------------------------------------------------------
    fprintf('Current parameters for iteration %d are:\n', i);
    % fprintf('%f\n',currentParameters);
    fprintf('Final parameters = [');
        m = size(finalParameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', finalParameters(j)); end
        fprintf(' %4.4g]; \n', finalParameters(m));
    fprintf('With final value %f\n', finalValue);
    %----------------------------------------------------------------------
    
    % Increasing bounds (optional)
    %-----------------------------
    deltaLower = deltaLower .* increaseRatio;
    deltaUpper = deltaUpper .* increaseRatio;
    %-----------------------------
    
    %%
    % Print out information at each iteration in case one iteration fails
    fprintf('initialParameters = [');
        m = size(initialParameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', initialParameters(j)); end
        fprintf(' %4.4g]; \n', initialParameters(m));
    fprintf('%% initial cost is %g;\n', initialCost);
        
    minValue=10^12;
    for n=1:i;
        if (n==i); fprintf('ParamLast = ['); 
        else; fprintf('Param%d = [', n); end
        m = size(parameterHistory(n).parameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', parameterHistory(n).parameters(j)); end
        fprintf(' %4.4g]; \n', parameterHistory(n).parameters(m));
        fprintf('%% %d final cost is %g;\n', n, parameterHistory(n).value);
        
        % Display specific cost information.
        fprintf('%% Costs: RIO %g, equality %g, inequality %g, Poincare %g, Energy %g\n',parameterHistory(n).costs);
        fprintf('%% Maximum Eigenvalue is %g\n',parameterHistory(n).maxEigValue);
    
        if parameterHistory(n).value<minValue; minIndex = n; 
        minValue=parameterHistory(n).value; end;
    end
    
    % Display minimum.
    n = minIndex; fprintf('\nParamMin = [');
    m = size(parameterHistory(n).parameters,1);
        for j=1:(m-1)
            fprintf(' %4.10g;', parameterHistory(n).parameters(j)); end
        fprintf(' %4.10g]; \n', parameterHistory(n).parameters(m));
        fprintf('%% %d final cost is %g;\n', n, parameterHistory(n).value);
    fprintf('%% %d equality constraints are:', n);
        m = size(parameterHistory(n).eq,1);
        for j=1:(m-1)
            fprintf(' %4.4g,', parameterHistory(n).eq(j)); end
        fprintf(' %4.4g \n', parameterHistory(n).eq(m));
    fprintf('%% %d inequality constraints are:', n);
        m = size(parameterHistory(n).ineq,1);
        for j=1:(m-1)
            fprintf(' %4.4g,', parameterHistory(n).ineq(j)); end
        fprintf(' %4.4g \n', parameterHistory(n).ineq(m));
        if parameterHistory(n).value<minValue; minIndex = n; end;
    % Display specific cost information.
    fprintf('%% Costs: RIO %g, equality %g, inequality %g, Poincare %g, Energy %g\n',parameterHistory(n).costs);
    fprintf('%% Maximum Eigenvalue is %g\n',parameterHistory(n).maxEigValue);
    % Start next optimization from current minimum.
    currentParameters = parameterHistory(minIndex).parameters;
        
    % Display delta: 
    % i is current iteration, minIndex
    k = 1; n = minIndex; 
    fprintf('\n%% delta final cost: %g; (%0.3g%% decrease)\n', parameterHistory(n).value - initialCost, 100*(initialCost - parameterHistory(n).value)/initialCost);
    fprintf('%% delta ParamMin = [');
    m = size(parameterHistory(n).parameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', parameterHistory(n).parameters(j) - initialParameters(j)); end
        fprintf(' %4.4g]; \n', parameterHistory(n).parameters(j) - initialParameters(j));
    fprintf('%% delta equality constraints:', n);
        m = size(parameterHistory(n).eq,1);
        for j=1:(m-1)
            fprintf(' %4.4g,', parameterHistory(n).eq(j) - initialEq(j)); end
        fprintf(' %4.4g \n', parameterHistory(n).eq(m) - initialEq(m));
    fprintf('%% delta inequality constraints:', n);
        m = size(parameterHistory(n).ineq,1);
        for j=1:(m-1)
            fprintf(' %4.4g,', parameterHistory(n).ineq(j) - initialIneq(j)); end
        fprintf(' %4.4g \n', parameterHistory(n).ineq(m) - initialIneq(m));
    [change, changeidx] = sort(abs(parameterHistory(n).parameters - initialParameters),'descend');
        changeNum = 10;
        fprintf('\nLargest changes from initial to iteration %d:\n',n)
        for j=1:changeNum
        changeTemp = parameterHistory(n).parameters(changeidx(j)) - initialParameters(changeidx(j));
        fprintf('Param %d from %4.10g to %4.10g (%4.4g)\n',changeidx(j),initialParameters(changeidx(j)),parameterHistory(n).parameters(changeidx(j)),changeTemp); end
    [changeBound, changeBoundidx] = sort(abs(parameterHistory(n).parameters - initialParameters)./deltaUpper,'descend');
        changeNum = 10;
        fprintf('\nLargest changes relative to bound size from initial to iteration %d:\n',n)
        for j=1:changeNum
        changeTemp = parameterHistory(n).parameters(changeBoundidx(j)) - initialParameters(changeBoundidx(j));
        fprintf('Param %d changed %4.4g with upper bound of %4.4g (%4.4g)\n',changeBoundidx(j),changeTemp,deltaUpper(changeBoundidx(j)),changeBound(j)); end
    %%
    save(sprintf('mat/currentFminconIteration%d.mat',i))
    
    if (finalValue==0) % End if there has been an optimization failure.
        nIterations = i;
        break
    end
end

%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Summary print out
%--------------------------------------------------------------------------

% Print out history of iterations
fprintf('\nIteration Summary\n')
fprintf('initialParameters = [%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g];\n', initialParameters)
for i=1:nIterations;
    fprintf('Param%d = [', i);
        m = size(parameterHistory(i).parameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', parameterHistory(i).parameters(j)); end
        fprintf(' %4.4g]; \n', parameterHistory(i).parameters(m));
    fprintf('%% Param%d final cost is %g;\n', i, parameterHistory(i).value);
end

% Print out bound information
fprintf('deltaLower = [', n);
        m = size(deltaLower,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', deltaLower(j)); end
        fprintf(' %4.4g]; \n', deltaLower(m));
fprintf('deltaUpper = [', n);
        m = size(deltaUpper,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', deltaUpper(j)); end
        fprintf(' %4.4g]; \n', deltaUpper(m));
        
% Print out final parameters and values from all iterations.
fprintf('Final parameters for iteration %d of %d:\n', i, nIterations);
fprintf('Parameters = [');
        m = size(finalParameters,1);
        for j=1:(m-1)
            fprintf(' %4.4g;', finalParameters(j)); end
        fprintf(' %4.4g]; \n', finalParameters(m));
fprintf('With final value %f\n', finalValue);

%--------------------------------------------------------------------------

end

