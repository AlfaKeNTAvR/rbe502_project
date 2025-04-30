function M = computeM(q, qd, qdd)
    % computeM returns the inertia matrix M(q, q̇, q̈)
    % q, qd, qdd are 3x1 vectors
    
    q1 = q(1); q2 = q(2); q3 = q(3);
    q1_d = qd(1); q2_d = qd(2); q3_d = qd(3);
    q1_dd = qdd(1); q2_dd = qdd(2); q3_dd = qdd(3);
    
    % Paste your full symbolic expression for M here
    M = [(9*q1_dd*(4*cos(2*q2 + q3) + 12*cos(2*q2) + cos(2*q2 + 2*q3) + 4*cos(q3) + 13))/3200, ...
        (9*q1_dd*(q2_dd - 1)*(4*cos(2*q2 + q3) + 12*cos(2*q2) + cos(2*q2 + 2*q3) + 4*cos(q3) + 13))/(3200*q2_dd), ...                                                                                                                                                                                                                                                                                                                           
        (9*q1_dd*(q3_dd - 1)*(4*cos(2*q2 + q3) + 12*cos(2*q2) + cos(2*q2 + 2*q3) + 4*cos(q3) + 13))/(3200*q3_dd);
        (((9*q1_dd)/2 - 9/2)*(130*q2_dd + 10*q3_dd + 654*cos(q2 + q3) + 3924*cos(q2) + 20*q1_d^2*sin(2*q2 + q3) + 60*q1_d^2*sin(2*q2) + 5*q1_d^2*sin(2*q2 + 2*q3) + 40*q2_dd*cos(q3) + 20*q3_dd*cos(q3)))/(8000*q1_dd), ...
        (180*q2_dd^2*cos(q3) - 2943*cos(q2 + q3) - 17658*cos(q2) - 45*q3_dd + 45*q2_dd*q3_dd - 90*q1_d^2*sin(2*q2 + q3) - 270*q1_d^2*sin(2*q2) + 2943*q2_dd*cos(q2 + q3) - (45*q1_d^2*sin(2*q2 + 2*q3))/2 + 17658*q2_dd*cos(q2) - 90*q3_dd*cos(q3) + 585*q2_dd^2 + (45*q1_d^2*q2_dd*sin(2*q2 + 2*q3))/2 + 90*q2_dd*q3_dd*cos(q3) + 90*q1_d^2*q2_dd*sin(2*q2 + q3) + 270*q1_d^2*q2_dd*sin(2*q2))/(8000*q2_dd),                              
        (90*q3_dd^2*cos(q3) - 2943*cos(q2 + q3) - 17658*cos(q2) - 585*q2_dd + 585*q2_dd*q3_dd - 90*q1_d^2*sin(2*q2 + q3) - 270*q1_d^2*sin(2*q2) + 2943*q3_dd*cos(q2 + q3) - (45*q1_d^2*sin(2*q2 + 2*q3))/2 - 180*q2_dd*cos(q3) + 17658*q3_dd*cos(q2) + 45*q3_dd^2 + (45*q1_d^2*q3_dd*sin(2*q2 + 2*q3))/2 + 180*q2_dd*q3_dd*cos(q3) + 90*q1_d^2*q3_dd*sin(2*q2 + q3) + 270*q1_d^2*q3_dd*sin(2*q2))/(8000*q3_dd);
        (((9*q1_dd)/2 - 9/2)*(10*q2_dd + 10*q3_dd + 654*cos(q2 + q3) + 10*q1_d^2*sin(q3) + 20*q2_d^2*sin(q3) + 10*q1_d^2*sin(2*q2 + q3) + 5*q1_d^2*sin(2*q2 + 2*q3) + 20*q2_dd*cos(q3) + 20*q2_d*q3_d*sin(q3)))/(8000*q1_dd), ...
        (90*q3_dd^2*cos(q3) - 2943*cos(q2 + q3) - 17658*cos(q2) - 585*q2_dd + 585*q2_dd*q3_dd - 90*q1_d^2*sin(2*q2 + q3) - 270*q1_d^2*sin(2*q2) + 2943*q3_dd*cos(q2 + q3) - (45*q1_d^2*sin(2*q2 + 2*q3))/2 - 180*q2_dd*cos(q3) + 17658*q3_dd*cos(q2) + 45*q3_dd^2 + (45*q1_d^2*q3_dd*sin(2*q2 + 2*q3))/2 + 180*q2_dd*q3_dd*cos(q3) + 90*q1_d^2*q3_dd*sin(2*q2 + q3) + 270*q1_d^2*q3_dd*sin(2*q2))/(8000*q3_dd), 
        (45*q2_dd*q3_dd - 2943*cos(q2 + q3) - 45*q1_d^2*sin(q3) - 90*q2_d^2*sin(q3) - 45*q2_dd - 45*q1_d^2*sin(2*q2 + q3) + 2943*q3_dd*cos(q2 + q3) - (45*q1_d^2*sin(2*q2 + 2*q3))/2 - 90*q2_dd*cos(q3) + 45*q3_dd^2 + (45*q1_d^2*q3_dd*sin(2*q2 + 2*q3))/2 + 90*q2_dd*q3_dd*cos(q3) - 90*q2_d*q3_d*sin(q3) + 45*q1_d^2*q3_dd*sin(q3) + 90*q2_d^2*q3_dd*sin(q3) + 45*q1_d^2*q3_dd*sin(2*q2 + q3) + 90*q2_d*q3_d*q3_dd*sin(q3))/(8000*q3_dd)];

end