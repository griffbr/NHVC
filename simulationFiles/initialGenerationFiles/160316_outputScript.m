clear all; close all;

% BAG160316
%--------------------------------------
%% New outputs and actuated coordinates.
%--------------------------------------

% Physical parameters
%     g  = 9.81;
%     m1 = physic.shin.mass;
%     m2 = physic.thigh.mass;
%     m3 = physic.four_bar_link.mass;
%     m4 = physic.lower_leg.mass;
%     m5 = physic.shinspring.mass;
%     m6 = physic.thighspring.mass;
%     mT = physic.torso.mass;
%     mH = physic.hip.mass;
%     L1 = physic.shin.length;
%     L2 = physic.thigh.length;
%     L3 = physic.four_bar_link.length;
%     L4 = physic.lower_leg.length;
%     LT = physic.torso.length;
%     W  = physic.hip.length;
%     Jrotor1 = physic.Jrotor1;
%     Jrotor2 = physic.Jrotor2;
%     Jrotor3 =  physic.Jrotor3;
%     Jgear1  = physic.Jgear1;
%     Jgear2  = physic.Jgear2;
%     Jgear3  = physic.Jgear3;
%     r1 = physic.r1;
%     r2 = physic.r2;
%     r3 = physic.r3;
%     K1 = physic.K1;
%     K2 = physic.K2;

swap = [-1 0 0  0 0 0 0  0 0 0 0 0 0;
        0 -1 0  0 0 0 0  0 0 0 0 0 0;
        0  0 1  0 0 0 0  0 0 0 0 0 0;
        ...
        0 0 0  0 0 1 0  0 0 0 0 0 0;
        0 0 0  0 0 0 1  0 0 0 0 0 0;
        0 0 0  1 0 0 0  0 0 0 0 0 0;
        0 0 0  0 1 0 0  0 0 0 0 0 0;
        ...
        0 0 0  0 0 0 0  0 0 0 1 0 0;
        0 0 0  0 0 0 0  0 0 0 0 1 0;
        0 0 0  0 0 0 0  0 0 0 0 0 1;
        0 0 0  0 0 0 0  1 0 0 0 0 0;
        0 0 0  0 0 0 0  0 1 0 0 0 0;
        0 0 0  0 0 0 0  0 0 1 0 0 0];

syms qT q1 q2 q1L q2L sigma0 dqT dq1 dq2 dq1L dq2L
syms L2 L4 mTotal g w
syms Letax Letay etaxHat etayHat

syms qzT qyT qxT q1R q2R q1L q2L qgr1R qgr2R q3R qgr1L qgr2L q3L
syms dqzT dqyT dqxT dq1R dq2R dq1L dq2L dqgr1R dqgr2R dq3R dqgr1L dqgr2L dq3L

% q13DOF
q = [qzT; qyT; qxT; q1R; q2R; q1L; q2L; qgr1R; qgr2R; q3R; qgr1L; qgr2L; q3L];
dq = [dqzT; dqyT; dqxT; dq1R; dq2R; dq1L; dq2L; dqgr1R; dqgr2R; dq3R; dqgr1L; dqgr2L; dq3L];
    
% qunactuated coordinate:
phipy = L2*sin(q2R + qxT) + L4*sin(q1R + qxT);

% qactuated coordinates:
qLAST = (qgr1R+qgr2R)/2;
qKAST = -qgr1R + qgr2R;
qKASW = -qgr1L + qgr2L;
qLASW = (qgr1L + qgr2L)/2;

% 160330
syms cpq3R
q3LAbs = q3L - qyT;
qbar = [phipy;
        qKAST;
        qLAST;
        cpq3R*q3R - qyT;
        q3LAbs;
        qLASW;
        qKASW];

% Calculate partial derivative:
dqbardq = jacobian(qbar, q);
d2qbardq2_qdot2 = jacobian(dqbardq*dq,q)*dq;

%%
fprintf('\nqbar = [%s;\n    %s;\n    %s;\n    %s;\n    %s;\n    %s;\n    %s];\n',char(qbar(1)),char(qbar(2)),char(qbar(3)),char(qbar(4)),char(qbar(5)),char(qbar(6)),char(qbar(7)));

fprintf('\ndqbardq = [')
for j=1:size(dqbardq,1);
for i=1:size(dqbardq,2);
    if i==size(dqbardq,2); fprintf('%s',char(dqbardq(j,i))); else fprintf('%s,    ',char(dqbardq(j,i))); end
end; 
if j==size(dqbardq,1); fprintf('];\n'); else; fprintf(';\n    '); end; end;

fprintf('\nd2qbardq2_qdot2 = [%s;\n    %s;\n    %s;\n    %s;\n    %s;\n    %s;\n    %s];\n',char(d2qbardq2_qdot2(1)),char(d2qbardq2_qdot2(2)),char(d2qbardq2_qdot2(3)),char(d2qbardq2_qdot2(4)),char(d2qbardq2_qdot2(5)),char(d2qbardq2_qdot2(6)),char(d2qbardq2_qdot2(7)));

%% Debugging
if 0
[pcm, p0, p0T, pHR, p1R, p2R, p3R, p4R, pHL, p1L, p2L, p3L, p4L] = ATRIAS3D_Positions_RightSym(q)
 
q = [0;0;0;pi/2;pi/2;pi/2;pi/2;pi/2;pi/2;0;pi/2;pi/2;0];
q = [0;0;0;pi/2;pi/2;pi/2;pi/2;pi/2;pi/2;0;pi/2;pi/2;pi/2];
[pcm, p0, p0T, pHR, p1R, p2R, p3R, p4R, pHL, p1L, p2L, p3L, p4L] = ATRIAS3D_Positions_Right(q)
end

%% Final symbollic values to use inside of new function.
if 0
    

    
    
    
    
    
    
    
    
    
    
    
    
end
 