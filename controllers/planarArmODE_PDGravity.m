function dx = planarArmODE_PDGravity(t, x, q_des, load)
% ODE function for a 3-DOF arm with PD control and gravity compensation.
%
% Inputs:
%   t      - Time (unused, for ODE solver compatibility)
%   x      - 6×1 state vector [theta; dtheta]
%   q_des  - 3×1 desired joint positions
%
% Output:
%   dx     - 6×1 state derivative [dtheta; ddtheta]
%
% Used with MATLAB ODE solvers like ode45.

    theta_d = [q_des(1); q_des(2); q_des(3)];
    dtheta_d = [0; 0; 0];
    theta = x(1:3);
    dtheta = x(4:6);
    
    M = computeM(theta, load);
    C = computeC(theta, dtheta, load);
    G = computeG(theta, load);
    tau = PD(theta_d, dtheta_d, theta, dtheta) + G;
    
    dx = zeros(6,1);
    dx(1:3) = dtheta;
    dx(4:6) = M \ (tau - C - G);
end