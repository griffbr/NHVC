function [ hAlphaReset ] = hAlphaReset( hAlphaInitial, sPlus, dsPlus, qactPlus, dqactPlus )
% The purpose of this function is to reset Bezier parameters using the
% method derived in Quals II and ACC 2015.
% BAG20150216
% BAG20150510 Updated to work for general size of hAlpha.
% June 28th, 2013 BAG
% The point of this function is to be
% able to calculate correctly the first two columns of bezier parameters
% given qplus but s not equal to zero. This is relevant for unexpected
% impacts which attempt to create the best controls possible for after
% impact and still conform the standard periodic orbit near end of
% perturbed step.

[m,n] = size(hAlphaInitial);

halphaTemp = [zeros(m,2) hAlphaInitial(:,3:n)];
qactFromBezier2thru5 = bezier(halphaTemp,sPlus);
dqactFromBezier2thru5 = bezierd(halphaTemp,sPlus)*dsPlus;
% qact = (1-s)^5*a10 + 5*(1-s)^4*s*a11 + qactFromBezier2thru5
% dqact = dsplus * ( -5(1-s)^4*a10 + 5*(-4*(1-s)^3*s*a11 + (1-s)^4a11) ) + dqactFromBezier2thru5
% Need to solve for a10 and a11 to have correct h_alpha for qplus & dsplus.
% (see notes from June 28th, 2013 for derivation of equations)
ai1 = ( (dqactPlus - dqactFromBezier2thru5)/dsPlus + ...
    5*(qactPlus - qactFromBezier2thru5)/(1-sPlus) ) / ...
    (5*( (1-sPlus)^3*sPlus + (1-sPlus)^4 ));
ai0 = (qactPlus - qactFromBezier2thru5 - 5*(1-sPlus)^4*sPlus*ai1) / (1-sPlus)^5;

hAlphaReset = [ai0 ai1 halphaTemp(:,3:6)];

end

