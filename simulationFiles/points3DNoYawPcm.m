function [ swingFoot, phipy, pcmAbs, pcmNoYaw, pcmPlusNoYaw ] = points3DNoYawPcm( q, leg, stanceFoot)
% This function takes a set of coordinates q, and returns the swing foot
% position and other needed positional information. BAG20140909
% BAG20150509 Updated to work for NHVC 3D simulation.
% BAG20150122 Updated to work in 2D.

 if nargin<3
    stanceFoot = [0; 0; 0];
 end

% Find pcm and other positions.
if leg;
    [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
        Cfcn_ATRIAS3D_Primary_PointsLeft(q);
    swingFoot = p4R + stanceFoot; pcmAbs = pcm + stanceFoot;
    [pcmNoYaw p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
        Cfcn_ATRIAS3D_Primary_PointsLeft([0; q(2:end)]);
    pcmPlusNoYaw = pcmNoYaw - p4R;
    q1R=q(7); q2R=q(8); % Use left leg links to calculate phipy below.
else; [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
        Cfcn_ATRIAS3D_Primary_PointsRight(q);
    swingFoot = p4L + stanceFoot; pcmAbs = pcm + stanceFoot; 
    [pcmNoYaw p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
        Cfcn_ATRIAS3D_Primary_PointsRight([0; q(2:end)]);
    pcmPlusNoYaw = pcmNoYaw - p4L;
    q1R=q(4); q2R=q(5); 
end

L2 = 0.5; L4 = 0.5; % Original Leg.
qxT = q(3);
phipy = L2*sin(q2R + qxT) + L4*sin(q1R + qxT);


% display(leg)
% fprintf('p2x is %g and pcmx is %g\n',swingFoot(1),pcm(1))
% display(swingFoot(1)-pcm(1))

end

