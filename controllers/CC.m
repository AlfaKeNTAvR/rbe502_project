function tau = CC(theta_d, dtheta_d, theta, dtheta, S, M_home)
    % Cartesian impedance (compliance) control with obstacle at x = 0.25

    % Gains
    Kx = diag([200, 200, 200]);  % Task-space stiffness
    Dx = diag([30, 30, 30]);     % Task-space damping
    K_wall = 1000;               % Obstacle stiffness (contact gain)

    % FK for actual and desired EE poses
    T   = fkine(S, M_home, theta, 'space');
    T_d = fkine(S, M_home, theta_d, 'space');

    x  = T(1:3, 4);   % Actual EE position
    xd = T_d(1:3, 4); % Desired EE position
    e  = xd - x;

    % Jacobian
    J  = jacoba(S, M_home, theta');

    % Cartesian velocities
    v  = J * dtheta;
    vd = J * dtheta_d;
    de = vd - v;

    % Impedance force
    F = Kx * e + Dx * de;

    % === Obstacle reaction ===
    wall_x = 0.25;
    if x(1) >= wall_x
        F_contact_x = -K_wall * (x(1) - wall_x);
        F(1) = F(1) + F_contact_x;  % Modify X force only
    end

    % Map force to torque
    tau = J' * F;
end