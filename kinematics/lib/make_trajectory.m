function traj = make_trajectory(type, params)
    % Extract parameters
    t_0 = params.t(1);
    t_f = params.t(2);
    dt = params.dt;
    q_0 = params.q(1);
    q_f = params.q(2);
    v_0 = params.v(1);
    v_f = params.v(2);

    % Create the time vector
    traj.t = t_0:dt:t_f;
    if traj.t(end) ~= t_f
        traj.t = [traj.t t_f];
    end
    
    if strcmp(type, 'cubic')
        % Solve for cubic coefficients
        % Coefficients for cubic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        % Boundary conditions: q(t_0), q(t_final), v(t_0), v(t_final)
        T = [1   t_0   t_0^2   t_0^3;
             0   1     2*t_0   3*t_0^2;
             1   t_f   t_f^2   t_f^3;
             0   1     2*t_f   3*t_f^2];
        b = [q_0 v_0 q_f v_f]';
        a = T\b; % Solve linear equations

        % Calculate trajectory
        traj.q = a(1) + a(2) * traj.t + a(3) * traj.t.^2 + a(4) * traj.t.^3;
        traj.v = a(2) + 2 * a(3) * traj.t + 3 * a(4) * traj.t.^2;
        traj.a = 2 * a(3) + 6 * a(4) * traj.t;

    elseif strcmp(type, 'quintic')
        % Extract additional parameters for quintic trajectory
        a_0 = params.a(1);
        a_f = params.a(2);

        % Coefficients for quintic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        % Boundary conditions: q(t_0), q(t_f), v(t_0), v(t_f), a(t_0), a(t_f)
        T = [1   t_0   t_0^2   t_0^3    t_0^4     t_0^5;
            0    1     2*t_0   3*t_0^2  4*t_0^3   5*t_0^4;
            0    0     2       6*t_0    12*t_0^2  20*t_0^3;
            1    t_f   t_f^2   t_f^3    t_f^4     t_f^5;
            0    1     2*t_f   3*t_f^2  4*t_f^3   5*t_f^4;
            0    0     2       6*t_f    12*t_f^2  20*t_f^3];
        b = [q_0 v_0 a_0 q_f v_f a_f]';
        a = T\b; % Solve linear equations

        % Calculate trajectory
        traj.q = a(1) + a(2) * traj.t + a(3) * traj.t.^2 + a(4) * traj.t.^3 + a(5) * traj.t.^4 + a(6) * traj.t.^5;
        traj.v = a(2) + 2 * a(3) * traj.t + 3 * a(4) * traj.t.^2 + 4 * a(5) * traj.t.^3 + 5 * a(6) * traj.t.^4;
        traj.a = 2 * a(3) + 6 * a(4) * traj.t + 12 * a(5) * traj.t.^2 + 20 * a(6) * traj.t.^3;
    end
end