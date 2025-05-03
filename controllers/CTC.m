function tau = CTC(theta_d, dtheta_d, ddtheta_d, theta, dtheta, M, C, G)

    
    Kp = diag([10 10 10]);
    Kv = diag([10 10 10]);
    
    e = theta_d - theta;
    de = dtheta_d - dtheta;
    
    tau = M * (ddtheta_d + Kp * e + Kv * de) + C + G;
end