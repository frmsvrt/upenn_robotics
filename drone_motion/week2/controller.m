function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

res = s_des - s;
za_des = 1;
K_p = 110;
K_v = 15;
u = params.mass * (za_des + K_p*res(1) + K_v*res(2) + params.gravity);

% FILL IN YOUR CODE HERE


end

