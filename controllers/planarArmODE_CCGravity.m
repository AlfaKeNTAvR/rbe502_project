function dx = planarArmODE_CCGravity(t, x, q_des, load, S, M_home)
    % ODE function for a 3-DOF arm with Cartesian compliance control and obstacle

    global desiredJointPositions jointTorques;

    % === Control gains ===
    Kp = 0.2;
    Kd = 1.0;
    K  = 30;

    % === Extract and ensure vector shapes ===
    theta     = x(1:3); theta = theta(:);
    dtheta    = x(4:6); dtheta = dtheta(:);
    theta_d   = q_des(:);
    dtheta_d  = zeros(3,1);

    % === Dynamics ===
    M = computeM(theta, load);
    C = computeC(theta, dtheta, load);
    G = computeG(theta, load);

    % === Forward kinematics ===
    T_cur = fkine(S, M_home, theta, 'space');
    T_des = fkine(S, M_home, theta_d, 'space');

    x_cur = T_cur(1:3, 4);
    x_des = T_des(1:3, 4);
    pose_error = x_des - x_cur;

    % === Virtual wall: obstacle at x = -0.25 ===
    xr = -0.25;
    he = [0; 0; 0];
    if x_cur(1) <= xr
        he(1) = Kp * K / (Kp + K) * (x_des(1) - xr);
    end

    % === Jacobian ===
    J = jacoba(S, M_home, theta);

    % === Debug Output ===
    % disp('--- DEBUG STEP ---');
    % fprintf('Time: %.3f\n', t);
    % fprintf('Current Pose (EE):  [% .3f  % .3f  % .3f]\n', x_cur);
    % fprintf('Desired Pose (EE):  [% .3f  % .3f  % .3f]\n', x_des);
    % fprintf('Pose Error:         [% .3f  % .3f  % .3f]\n', pose_error);
    % fprintf('External Force (he):[% .3f  % .3f  % .3f]\n', he);
    % fprintf('Torque (tau):       [% .3f  % .3f  % .3f]\n', ...
    %     (G + J' * Kp * pose_error - J' * Kd * J * dtheta));
    % fprintf('Joint Velocities:   [% .3f  % .3f  % .3f]\n\n', dtheta);

    % === Control torque ===
    tau = G + J' * Kp * pose_error - J' * Kd * J * dtheta;

    % === State update ===
    dx = zeros(6,1);
    dx(1:3) = dtheta;
    dx(4:6) = M \ (tau - C - J' * he);

    % === Logging ===
    desiredJointPositions = [desiredJointPositions; theta_d'];
    jointTorques = [jointTorques; tau'];
end
