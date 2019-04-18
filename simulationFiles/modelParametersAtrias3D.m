function [ allStep ] = modelParametersAtrias3D( allStep )
% The purpose of this function is to store model parameters inside of
% structured variable.
% BAG20140923
% BAG20150510 Modified to work for NHVC 3D.
% BAG20150122. Modified to work in 2D.

physic = ATRIAS3D_Physic;

allStep.g  = 9.81;
m1 = physic.shin.mass;
m2 = physic.thigh.mass;
m3 = physic.four_bar_link.mass;
m4 = physic.lower_leg.mass;
m5 = physic.shinspring.mass;
m6 = physic.thighspring.mass;
mT = physic.torso.mass;
mH = physic.hip.mass;
L1 = physic.shin.length;
L2 = physic.thigh.length;
L3 = physic.four_bar_link.length;
L4 = physic.lower_leg.length;
LT = physic.torso.length;
W  = physic.hip.length;
Jrotor1 = physic.Jrotor1;
Jrotor2 = physic.Jrotor2;
Jrotor3 =  physic.Jrotor3;
Jgear1  = physic.Jgear1;
Jgear2  = physic.Jgear2;
Jgear3  = physic.Jgear3;
allStep.r1 = physic.r1;
allStep.r2 = physic.r2;
allStep.r3 = physic.r3;
K1 = physic.K1;
K2 = physic.K2;

allStep.mTotal = 2*(m1 + m2 + m3 + m4 + m5 + m6 + mH) + mT;

end

