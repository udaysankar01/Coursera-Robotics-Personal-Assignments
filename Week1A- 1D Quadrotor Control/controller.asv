function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE
g = params.gravity;
m = params.mass;
l = params.arm_length;
u_min = params.u_min;
u_max = params.u_max;

z_des_dot = 0;
Kv = 19;
Kp = 200;

e = s_des - s;

u = m * (z_des_dot + Kp*e(1) + Kv*e(2) + g);
if(u > u_max)
    u = u_max;
end
if(u < u_min)
    u = u_min;
end


end

