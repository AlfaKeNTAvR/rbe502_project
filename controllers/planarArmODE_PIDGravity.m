function dx = planarArmODE_PIDGravity(t, x, q_des)
    % Global variables for logging and integral state
    global I

    theta_d = [q_des(1); q_des(2); q_des(3)];
    dtheta_d = [0; 0; 0];
    theta = x(1:3);
    dtheta = x(4:6);

    M = computeM(theta);
    C = computeC(theta, dtheta);
    G = computeG(theta);
    [tau, err] = PID(t, theta_d, dtheta_d, theta, dtheta);

    % === State Derivatives ===
    dx = zeros(6, 1);
    dx(1:3) = dtheta;
    dx(4:6) = M \ (tau - C - G);
end