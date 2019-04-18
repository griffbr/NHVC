function [ step, allStep ] = hybridSim3D( step, allStep )
% This function will take the previous state conditions and apply the 
% impact map for both states and control, swapping, and simulation of the 
% next step.
% BAG20140730
% BAG20150510 Updated for NHVC 3D.
% BAG20150122. Updated to work for 2D.

% Discrete Simulation
[step, allStep] = discrete3D(step, allStep);

% Continuous Simulation
[step, allStep] = continuous3D(step, allStep);

end

