function tau = PD(theta_des, dtheta_des, theta, dtheta)
% Computes control torque using a PD (Proportional-Derivative) controller.
%
% Inputs:
%   theta_des   - Desired joint positions (3×1)
%   dtheta_des  - Desired joint velocities (3×1)
%   theta       - Current joint positions (3×1)
%   dtheta      - Current joint velocities (3×1)
%
% Output:
%   tau         - Control torque output (3×1)

    Kp = diag([10 10 10]);
    Kv = diag([10 10 10]);
    
    e = theta_des - theta;
    de = dtheta_des - dtheta;
    
    tau = Kp*e + Kv*de;
end