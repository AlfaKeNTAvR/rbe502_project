function [tau, e] = PID(t, theta_d, dtheta_d, theta, dtheta)
% PID controller for a 3-DOF robotic arm.
%
% Inputs:
%   t         - Current time (scalar)
%   theta_d   - Desired joint positions (3×1)
%   dtheta_d  - Desired joint velocities (3×1)
%   theta     - Current joint positions (3×1)
%   dtheta    - Current joint velocities (3×1)
%
% Outputs:
%   tau       - Control torque output (3×1)
%   e         - Position error (3×1)

    persistent I prev_t

    if isempty(I)
        I = zeros(3,1);
    end
    if isempty(prev_t)
        prev_t = t;
    end

    % Time step
    dt = t - prev_t;
    prev_t = t;

    % Gains
    Kp = diag([10 10 10]);
    Kv = diag([10 10 10]);
    Ki = diag([0.1 0.1 0.1]);

    % Errors
    e = theta_d - theta;
    de = dtheta_d - dtheta;

    % Update integral term
    I = I + e * dt;

    % Control law
    tau = Kp * e + Kv * de + Ki * I;
end