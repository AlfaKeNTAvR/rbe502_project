function tau = PDPos(theta_des, dtheta_des, theta, dtheta)
    Kp = diag([10 10 10]);
    Kv = diag([10 10 10]);
    
    e = theta_des - theta;
    de = dtheta_des - dtheta;
    
    tau = Kp*e + Kv*de;
end