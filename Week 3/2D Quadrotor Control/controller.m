function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

g = params.gravity;
m = params.mass;
Ixx = params.Ixx;
l = params.arm_length;
minF = params.minF;
maxF = params.maxF;

u1_max = 2*maxF;
u1_min = 2*minF;
u2_max = l*(maxF-minF);
u2_min = l*(minF-maxF);
phi_c_dot = 0;

[K_d_z, K_d_phi, K_d_y] = deal(5, 15, 5);
[K_p_z, K_p_phi, K_p_y] = deal(100, 1000, 100);
[e_z_dot, e_y_dot] = deal(des_state.vel(2)-state.vel(2), des_state.vel(1) - state.vel(1));
[e_z, e_y] = deal(des_state.pos(2)-state.pos(2), des_state.pos(1)-state.pos(1));

phi_c = (-1/g)*(des_state.acc(1) + K_d_y *e_y_dot + K_p_y*e_y);
u1 = m*(g + des_state.acc(2) + K_d_z*e_z_dot + K_p_z*e_z);
u2 = Ixx*(K_d_phi*(phi_c_dot-state.omega) + K_p_phi*(phi_c-state.rot));

if(u1>u1_max)
    u1 = u1_max;
end
if(u1<u1_min)
    u1 = u1_min;
end
if(u2>u2_max)
    u2 = u2_max;
end
if(u2<u2_min)
    u2 = u2_min;

end

