function [ u ] = calculateU( dhqdq,D,B,H,d2h0dq2_qdot2,d2hdesdq2_qdot2,dhsigmadsigma,ddsigma,ddhsigmaddsigma_sigmadot2,kp,y,kd,dy )
% The purpose of this function is to calculate the control torques, u,
% given all the necessary variables in a feedback and feedforward sense.
% BAG160106

decouple = dhqdq*(D\B); % Theoretical decoupling matrix.
ffComponents = dhqdq*(D\H) -d2h0dq2_qdot2 + d2hdesdq2_qdot2 ...
        + dhsigmadsigma*ddsigma + ddhsigmaddsigma_sigmadot2; 
u = decouple\(ffComponents -kp*y - kd*dy);

end

