function dx = planarArmODE_CTCGravity(t, x, t_local, jointPos, jointVel, jointAcc, load)
    % Ensure t_local is a row vector
    t_local = t_local(:)';   % force row if needed

    % Interpolate
    q_des   = interp1(t_local, jointPos.', t, 'linear', 'extrap')';
    dq_des  = interp1(t_local, jointVel.', t, 'linear', 'extrap')';
    ddq_des = interp1(t_local, jointAcc.', t, 'linear', 'extrap')';

    theta  = x(1:3);    % joint positions
    dtheta = x(4:6);    % joint velocities

    M = computeM(theta, load);
    C = computeC(theta, dtheta, load);
    G = computeG(theta, load);

    tau = CTC(q_des, dq_des, ddq_des, theta, dtheta, M, C, G);
    
    dx = zeros(6,1);
    dx(1:3) = dtheta;
    dx(4:6) = M \ (tau - C - G);
end