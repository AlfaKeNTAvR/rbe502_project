function tau = CTC(theta_d, dtheta_d, ddtheta_d, theta, dtheta, M, C, G)
    % Computed Torque Controller for a 3-DOF robotic arm.
    %
    % Inputs:
    %   theta_d    - Desired joint positions (3×1)
    %   dtheta_d   - Desired joint velocities (3×1)
    %   ddtheta_d  - Desired joint accelerations (3×1)
    %   theta      - Current joint positions (3×1)
    %   dtheta     - Current joint velocities (3×1)
    %   M          - Inertia matrix at current state (3×3)
    %   C          - Coriolis/centrifugal vector at current state (3×1)
    %   G          - Gravity vector at current state (3×1)
    %
    % Output:
    %   tau        - Control torque command (3×1)

    global jointPositionErrors;
    
    Kp = diag([1.0 1.0 1.0]);
    Kv = diag([1.0 1.0 1.0]);
    
    e = theta_d - theta;
    de = dtheta_d - dtheta;
    
    tau = M * (ddtheta_d + Kp * e + Kv * de) + C + G;

    jointPositionErrors = [jointPositionErrors; e']; % Store the error for plotting
end