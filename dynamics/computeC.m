function C = computeC(q, qd)
    % computeC returns the Coriolis & centrifugal vector C(q, q̇)
    % q, qd, are 3x1 vectors: [q1; q2; q3], [q1_d; q2_d; q3_d]
    
    q2 = q(2); 
    q3 = q(3);
    q1_d = qd(1); 
    q2_d = qd(2); 
    q3_d = qd(3);
    
    C = [0;
        (9*q1_d^2*(4*sin(2*q2 + q3) + 12*sin(2*q2) + sin(2*q2 + 2*q3)))/3200;
        (9*q1_d^2*sin(q3))/1600 + (9*q2_d^2*sin(q3))/800 + (9*q1_d^2*sin(2*q2 + q3))/1600 + (9*q1_d^2*sin(2*q2 + 2*q3))/3200 + (9*q2_d*q3_d*sin(q3))/800];
 
end