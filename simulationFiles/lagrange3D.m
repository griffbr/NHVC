function [D,Cdq,G,B] = lagrange3D(q,dq,leg);
% Lagrange calculation for 3D. Assumes 13 DOF model.
% BAG20150510.

if 1
    %% Normal symmetric operation.
if leg % Consider symmetric robot.
    % Swap D, Cdq, B, and G to work for left leg.
    % Need a swap from L to R to multiple by D, Cdq, B, G.
    swap = [-1 0 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1;
    0 0 0 1 0 0 0 0 0; 0 0 0 0 1 0 0 0 0; 0 0 0 0 0 1 0 0 0];
    % Find swapped coordinates.
    q=swap*q; dq=swap*dq;
    % Find matrices using swapped coordinates.
    [D] = Cfcn_Right_Atrias_D(q, dq);
    [Cdq] = Cfcn_Right_Atrias_Cdq(q, dq);
    [G] = Cfcn_Right_Atrias_G(q, dq);
    [B] = Cfcn_Right_Atrias_B(q, dq);
    % Use swap matrix to find appropriate swapped D, Cdq, and G
    % matrices.
    D   = transpose(swap) * D * swap;
    Cdq = transpose(swap) * Cdq;
    G   = transpose(swap) * G;
else
    [D] = Cfcn_Right_Atrias_D(q, dq);
    [Cdq] = Cfcn_Right_Atrias_Cdq(q, dq);
    [G] = Cfcn_Right_Atrias_G(q, dq);
    [B] = Cfcn_Right_Atrias_B(q, dq);
end
else
    %% Debugging
    [D] = ATRIAS3D_D_Left(q);
    [Cdq] = ATRIAS3D_Cdq_Left(q, dq);
    [G] = ATRIAS3D_G_Left(q);
    [B] = ATRIAS3D_B_Left(q);
end

