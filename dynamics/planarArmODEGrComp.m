function dx = planarArmODEGrComp(t, x)

    q1_des = 1.57;
    q2_des = 0;
    q3_des = 0;

    theta_des = [q1_des; q2_des; q3_des];
    dtheta_des = [0; 0; 0];
    theta = x(1:3);
    dtheta = x(4:6);
    
    M = computeM(theta);
    C = computeC(theta, dtheta);
    G = computeG(theta);
    tau = PDPos(theta_des, dtheta_des, theta, dtheta) + G; % add gravity feedforward
    
    dx = zeros(6,1);
    dx(1:3) = dtheta;
    dx(4:6) = M \ (tau - C - G); % correct dynamics
end