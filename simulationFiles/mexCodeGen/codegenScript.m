% These are some example commands that can be used to create mex output
% functions (in order of occurance from test script).

%% 
basePath=pwd;
addpath([basePath,'/simulationFiles/mexCodeGen'])

%%
tic; codegen -g ATRIAS3D_ImpactModel_StanceRight_FixParams -args {qe}; toc;

%%
tic; codegen -g ATRIAS3D_Primary_PointsRight -args {q}; toc;

%%
tic; codegen -g  outputNHVC -args {qMinus,dqMinus,legMinus,phipyLimits,zeros(6,6),allStep.sigmaScale,zeros(2,1),zeros(6,4),allStep.cp}; toc;

%%
tic; codegen -g hAlphaReset -args {allStep.hAlpha, sPlus, dsPlus, qactPlus, dqactPlus}; toc;

%%
tic; codegen -g fcn_Right_Atrias_D -args {q, dq}; toc;
%%
tic; codegen -g fcn_Right_Atrias_Cdq -args {q, dq}; toc;
%%
tic; codegen -g fcn_Right_Atrias_G -args {q, dq}; toc;
tic; codegen -g fcn_Right_Atrias_B -args {q, dq}; toc;

%%
tic; codegen -g ATRIAS3D_Primary_PointsLeft -args {q}; toc;

%%
tic; codegen -g fcn_Left_Jacobians -args {q,dq}; toc;
tic; codegen -g fcn_Right_Jacobians -args {q,dq}; toc;

%%
tic; codegen -g ATRIAS3D_ImpactModel_StanceLeft_FixParams -args {qe}; toc;

%%
qe = [q; stanceFoot];
tic; codegen -g ATRIAS3D_Primary_Points_Anim_StanceLeft -args {qe}; toc;