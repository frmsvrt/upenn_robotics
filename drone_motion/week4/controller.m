function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

% =================== Your code ends here ===================
k_p = 400;
k_d = 40;

k_p_att = 100;
k_d_att = 2;

res_pos = des_state.pos - state.pos;
res_vel = des_state.vel - state.vel;

r_des_ddot = des_state.acc + k_d*res_vel + k_p*res_pos;

psi_t = des_state.yaw;
phi_des = (r_des_ddot(1)*psi_t - r_des_ddot(2))/params.gravity;
theta_des = (r_des_ddot(1) + r_des_ddot(2)*psi_t)/params.gravity;

e_rot = [phi_des; theta_des; des_state.yaw] - state.rot;
e_omega = [0; 0; des_state.yawdot] - state.omega;

F = params.mass*(params.gravity + r_des_ddot(3));
M = k_p_att*e_rot + k_d_att*e_omega;
end
